#include "modules/dynamics/adapter/PinocchioDynamicsBackendAdapter.h"

#include "core/errors/ErrorCode.h"
#include "core/kinematics/SharedRobotKernelRegistry.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#if defined(ROBOSDP_HAVE_PINOCCHIO)
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#endif

namespace RoboSDP::Dynamics::Adapter
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double DegToRad(double degrees)
{
    return degrees * kPi / 180.0;
}

double Sign(double value)
{
    if (value > 0.0)
    {
        return 1.0;
    }
    if (value < 0.0)
    {
        return -1.0;
    }
    return 0.0;
}

bool IsFiniteValue(double value)
{
    return std::isfinite(value);
}

void LogDynamicsMessage(
    RoboSDP::Logging::ILogger* logger,
    RoboSDP::Logging::LogLevel level,
    const QString& actionName,
    const QString& message)
{
    if (logger == nullptr)
    {
        return;
    }

    logger->Log(
        level,
        message,
        RoboSDP::Errors::ErrorCode::Ok,
        {
            QStringLiteral("Dynamics"),
            actionName,
            QStringLiteral("PinocchioDynamicsBackendAdapter")});
}

} // namespace

struct PinocchioDynamicsBackendAdapter::NativeKernelState
{
#if defined(ROBOSDP_HAVE_PINOCCHIO)
    /// @brief 从共享注册表获取的不可变原生模型，跨模块复用同一份静态结构。
    std::shared_ptr<const RoboSDP::Core::Kinematics::SharedPinocchioModel> model;

    /// @brief Dynamics 模块私有的 Data 草稿纸，确保和 Kinematics 之间没有副作用。
    std::unique_ptr<RoboSDP::Core::Kinematics::SharedPinocchioData> data;

    /// @brief 记录每个关节的零位偏移，供 q 向量映射时统一加回原生模型语义。
    std::vector<double> native_position_offsets_deg;
#endif

    QString cache_key;
    int build_count = 0;
    bool ready = false;
    QString last_message = QStringLiteral("尚未执行 Dynamics 共享内核构建。");
};

PinocchioDynamicsBackendAdapter::PinocchioDynamicsBackendAdapter(RoboSDP::Logging::ILogger* logger)
    : m_logger(logger)
    , m_kinematics_backend_adapter(logger)
    , m_native_kernel_state(std::make_unique<NativeKernelState>())
{
}

PinocchioDynamicsBackendAdapter::~PinocchioDynamicsBackendAdapter() = default;

QString PinocchioDynamicsBackendAdapter::BackendId() const
{
    return QStringLiteral("pinocchio_dynamics_backend");
}

QString PinocchioDynamicsBackendAdapter::BackendDescription() const
{
    return QStringLiteral("Pinocchio 动力学共享内核适配器（唯一后端）。");
}

bool PinocchioDynamicsBackendAdapter::UsesSharedRobotKernel() const
{
    return m_native_kernel_state != nullptr && m_native_kernel_state->ready;
}

std::uintptr_t PinocchioDynamicsBackendAdapter::NativeModelAddressForDiagnostics() const
{
#if defined(ROBOSDP_HAVE_PINOCCHIO)
    return (m_native_kernel_state != nullptr && m_native_kernel_state->model != nullptr)
        ? reinterpret_cast<std::uintptr_t>(m_native_kernel_state->model.get())
        : 0U;
#else
    return 0U;
#endif
}

std::uintptr_t PinocchioDynamicsBackendAdapter::NativeDataAddressForDiagnostics() const
{
#if defined(ROBOSDP_HAVE_PINOCCHIO)
    return (m_native_kernel_state != nullptr && m_native_kernel_state->data != nullptr)
        ? reinterpret_cast<std::uintptr_t>(m_native_kernel_state->data.get())
        : 0U;
#else
    return 0U;
#endif
}

RoboSDP::Dynamics::Dto::InverseDynamicsTrajectoryResultDto PinocchioDynamicsBackendAdapter::Analyze(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& kinematicModel,
    const RoboSDP::Dynamics::Dto::DynamicModelDto& dynamicModel,
    const RoboSDP::Dynamics::Dto::ParameterizedTrajectoryDto& trajectory) const
{
    const auto failWithReason =
        [this, &trajectory](const QString& reason) -> RoboSDP::Dynamics::Dto::InverseDynamicsTrajectoryResultDto
    {
        RoboSDP::Dynamics::Dto::InverseDynamicsTrajectoryResultDto result;
        result.trajectory_id = trajectory.trajectory_id;
        result.name = trajectory.name;
        result.solver_backend = QStringLiteral("pinocchio_shared_kernel");
        result.used_fallback = false;
        result.success = false;
        result.message = QStringLiteral("共享 Pinocchio 内核轨迹逆动力学计算失败：%1").arg(reason);
        LogDynamicsMessage(m_logger, RoboSDP::Logging::LogLevel::Warning, QStringLiteral("Analyze"), result.message);
        return result;
    };

    QString syncMessage;
    if (!SyncNativeModel(kinematicModel, dynamicModel, syncMessage))
    {
        return failWithReason(syncMessage);
    }

#if !defined(ROBOSDP_HAVE_PINOCCHIO)
    return failWithReason(QStringLiteral("当前构建未启用 Pinocchio C++ 依赖。"));
#else
    if (trajectory.samples.empty())
    {
        return failWithReason(QStringLiteral("轨迹采样点为空，无法执行逆动力学计算。"));
    }

    if (m_native_kernel_state == nullptr ||
        !m_native_kernel_state->ready ||
        m_native_kernel_state->model == nullptr ||
        m_native_kernel_state->data == nullptr)
    {
        return failWithReason(QStringLiteral("共享 Pinocchio Model/Data 尚未就绪。"));
    }

    const int expectedQCount = static_cast<int>(m_native_kernel_state->model->nq);
    const int expectedVCount = static_cast<int>(m_native_kernel_state->model->nv);
    if (static_cast<int>(dynamicModel.joints.size()) < expectedVCount)
    {
        return failWithReason(
            QStringLiteral("DynamicModel 中 joints 数量不足：期望至少 %1，实际 %2。")
                .arg(expectedVCount)
                .arg(dynamicModel.joints.size()));
    }

    for (std::size_t sampleIndex = 0; sampleIndex < trajectory.samples.size(); ++sampleIndex)
    {
        const auto& sample = trajectory.samples[sampleIndex];
        if (static_cast<int>(sample.positions_deg.size()) != expectedQCount)
        {
            return failWithReason(
                QStringLiteral("第 %1 个轨迹点的位置维度=%2，但共享内核 nq=%3。")
                    .arg(sampleIndex + 1)
                    .arg(sample.positions_deg.size())
                    .arg(expectedQCount));
        }

        if (static_cast<int>(sample.velocities_deg_s.size()) != expectedVCount)
        {
            return failWithReason(
                QStringLiteral("第 %1 个轨迹点的速度维度=%2，但共享内核 nv=%3。")
                    .arg(sampleIndex + 1)
                    .arg(sample.velocities_deg_s.size())
                    .arg(expectedVCount));
        }

        if (static_cast<int>(sample.accelerations_deg_s2.size()) != expectedVCount)
        {
            return failWithReason(
                QStringLiteral("第 %1 个轨迹点的加速度维度=%2，但共享内核 nv=%3。")
                    .arg(sampleIndex + 1)
                    .arg(sample.accelerations_deg_s2.size())
                    .arg(expectedVCount));
        }
    }

    try
    {
        RoboSDP::Dynamics::Dto::InverseDynamicsTrajectoryResultDto result;
        result.trajectory_id = trajectory.trajectory_id;
        result.name = trajectory.name;
        result.solver_backend = QStringLiteral("pinocchio_shared_kernel");
        result.used_fallback = false;
        result.joint_curves.resize(static_cast<std::size_t>(expectedVCount));

        for (int jointIndex = 0; jointIndex < expectedVCount; ++jointIndex)
        {
            const QString jointId =
                jointIndex < static_cast<int>(dynamicModel.joints.size()) &&
                    !dynamicModel.joints[static_cast<std::size_t>(jointIndex)].joint_id.trimmed().isEmpty()
                ? dynamicModel.joints[static_cast<std::size_t>(jointIndex)].joint_id
                : QStringLiteral("joint_%1").arg(jointIndex + 1);
            auto& curve = result.joint_curves[static_cast<std::size_t>(jointIndex)];
            curve.joint_id = jointId;
            curve.samples.reserve(trajectory.samples.size());
        }

        // 中文说明：q / v / a 在循环外预分配，避免批量轨迹计算中重复申请 Eigen 内存。
        Eigen::VectorXd q = Eigen::VectorXd::Zero(expectedQCount);
        Eigen::VectorXd v = Eigen::VectorXd::Zero(expectedVCount);
        Eigen::VectorXd a = Eigen::VectorXd::Zero(expectedVCount);

        for (const auto& sample : trajectory.samples)
        {
            for (int jointIndex = 0; jointIndex < expectedQCount; ++jointIndex)
            {
                const double offsetDeg =
                    jointIndex < static_cast<int>(m_native_kernel_state->native_position_offsets_deg.size())
                        ? m_native_kernel_state->native_position_offsets_deg[static_cast<std::size_t>(jointIndex)]
                        : 0.0;
                q[static_cast<Eigen::Index>(jointIndex)] =
                    DegToRad(sample.positions_deg[static_cast<std::size_t>(jointIndex)] + offsetDeg);
            }

            for (int jointIndex = 0; jointIndex < expectedVCount; ++jointIndex)
            {
                v[static_cast<Eigen::Index>(jointIndex)] =
                    DegToRad(sample.velocities_deg_s[static_cast<std::size_t>(jointIndex)]);
                a[static_cast<Eigen::Index>(jointIndex)] =
                    DegToRad(sample.accelerations_deg_s2[static_cast<std::size_t>(jointIndex)]);
            }

            pinocchio::rnea(*m_native_kernel_state->model, *m_native_kernel_state->data, q, v, a);
            const auto& tau = m_native_kernel_state->data->tau;

            for (int jointIndex = 0; jointIndex < expectedVCount; ++jointIndex)
            {
                const auto& drive = dynamicModel.joints[static_cast<std::size_t>(jointIndex)];
                const double velocityDegS = sample.velocities_deg_s[static_cast<std::size_t>(jointIndex)];
                const double velocityRadS = DegToRad(velocityDegS);
                const double frictionTorque =
                    drive.friction.viscous * velocityRadS + drive.friction.coulomb * Sign(velocityRadS);
                const double efficiency = std::max(0.1, drive.efficiency);
                const double jointTorque =
                    (tau[static_cast<Eigen::Index>(jointIndex)] + frictionTorque) / efficiency;

                result.joint_curves[static_cast<std::size_t>(jointIndex)].samples.push_back({
                    sample.time_s,
                    jointTorque,
                    velocityDegS,
                    std::abs(jointTorque * velocityRadS)});
            }
        }

        result.success = true;
        result.message = QStringLiteral("共享 Pinocchio 内核轨迹逆动力学计算完成。");
        LogDynamicsMessage(m_logger, RoboSDP::Logging::LogLevel::Info, QStringLiteral("Analyze"), result.message);
        return result;
    }
    catch (const std::exception& exception)
    {
        return failWithReason(
            QStringLiteral("共享 Pinocchio 内核轨迹计算异常：%1").arg(QString::fromUtf8(exception.what())));
    }
    catch (...)
    {
        return failWithReason(QStringLiteral("共享 Pinocchio 内核轨迹计算出现未知异常。"));
    }
#endif
}

QString PinocchioDynamicsBackendAdapter::ExplainWhySharedKernelNotReady() const
{
    return m_native_kernel_state == nullptr
        ? QStringLiteral("Dynamics 共享内核尚未初始化。")
        : m_native_kernel_state->last_message;
}

int PinocchioDynamicsBackendAdapter::GetNativeModelBuildCount() const
{
    return m_native_kernel_state == nullptr ? 0 : m_native_kernel_state->build_count;
}

bool PinocchioDynamicsBackendAdapter::SyncNativeModel(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& kinematicModel,
    const RoboSDP::Dynamics::Dto::DynamicModelDto& dynamicModel,
    QString& message) const
{
    if (m_native_kernel_state == nullptr)
    {
        m_native_kernel_state = std::make_unique<NativeKernelState>();
    }

    const auto buildContextResult = m_kinematics_backend_adapter.BuildNormalizedContext(kinematicModel);
    if (!buildContextResult.IsSuccess() || !buildContextResult.status.shared_robot_kernel_ready)
    {
        message = QStringLiteral("Kinematics build-context 未就绪，无法复用统一机器人模型语义：%1")
                      .arg(buildContextResult.status.status_message.trimmed().isEmpty()
                               ? QStringLiteral("未知原因")
                               : buildContextResult.status.status_message);
        m_native_kernel_state->ready = false;
        m_native_kernel_state->last_message = message;
        return false;
    }

    if (buildContextResult.context.normalized_model.joint_count <= 0)
    {
        message = QStringLiteral("统一机器人模型中的 joint_count 无效，无法构建 Dynamics 共享内核。");
        m_native_kernel_state->ready = false;
        m_native_kernel_state->last_message = message;
        return false;
    }

    const RoboSDP::Core::Kinematics::SharedRobotKernelRequest request {
        &kinematicModel,
        &dynamicModel,
        buildContextResult.context.normalized_model.unified_robot_model_ref,
        buildContextResult.context.normalized_model.modeling_mode,
        buildContextResult.context.normalized_model.joint_order_signature,
        QString(),
        true};

#if !defined(ROBOSDP_HAVE_PINOCCHIO)
    (void)request;
    message = QStringLiteral("当前构建未启用 Pinocchio C++ 依赖。");
    m_native_kernel_state->ready = false;
    m_native_kernel_state->last_message = message;
    return false;
#else
    const auto acquireResult =
        RoboSDP::Core::Kinematics::SharedRobotKernelRegistry::Instance().GetOrBuildKernel(request);

    if (!acquireResult.success || acquireResult.model == nullptr)
    {
        message = acquireResult.metadata.status_message.trimmed().isEmpty()
            ? QStringLiteral("共享 Pinocchio Model 获取失败。")
            : acquireResult.metadata.status_message;
        m_native_kernel_state->ready = false;
        m_native_kernel_state->last_message = message;
        return false;
    }

    if (m_native_kernel_state->ready &&
        m_native_kernel_state->cache_key == acquireResult.metadata.cache_key &&
        m_native_kernel_state->model == acquireResult.model &&
        m_native_kernel_state->data != nullptr)
    {
        m_native_kernel_state->build_count = acquireResult.metadata.registry_build_count;
        message = QStringLiteral("Dynamics 已命中共享 Pinocchio Model 缓存，并复用本模块私有 Data。");
        m_native_kernel_state->last_message = message;
        return true;
    }

    try
    {
        m_native_kernel_state->model = acquireResult.model;
        m_native_kernel_state->data =
            std::make_unique<RoboSDP::Core::Kinematics::SharedPinocchioData>(*acquireResult.model);
        m_native_kernel_state->native_position_offsets_deg =
            acquireResult.metadata.native_position_offsets_deg;
        m_native_kernel_state->cache_key = acquireResult.metadata.cache_key;
        m_native_kernel_state->build_count = acquireResult.metadata.registry_build_count;
        m_native_kernel_state->ready = true;
        message = acquireResult.cache_hit
            ? QStringLiteral("Dynamics 已复用 SharedRobotKernelRegistry 中的共享 Model，并刷新私有 Data。")
            : QStringLiteral("Dynamics 已从 SharedRobotKernelRegistry 获取共享 Model，并初始化私有 Data。");
        m_native_kernel_state->last_message = message;
        return true;
    }
    catch (const std::exception& exception)
    {
        message = QStringLiteral("Dynamics 私有 Data 初始化异常：%1").arg(QString::fromUtf8(exception.what()));
        m_native_kernel_state->ready = false;
        m_native_kernel_state->last_message = message;
        return false;
    }
    catch (...)
    {
        message = QStringLiteral("Dynamics 私有 Data 初始化出现未知异常。");
        m_native_kernel_state->ready = false;
        m_native_kernel_state->last_message = message;
        return false;
    }
#endif
}

RoboSDP::Dynamics::Dto::NativeRneaDryRunResultDto PinocchioDynamicsBackendAdapter::EvaluateNativeRneaDryRun(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& kinematicModel,
    const RoboSDP::Dynamics::Dto::DynamicModelDto& dynamicModel,
    const std::vector<double>& joint_positions_deg,
    const std::vector<double>& joint_velocities_deg_s,
    const std::vector<double>& joint_accelerations_deg_s2) const
{
    RoboSDP::Dynamics::Dto::NativeRneaDryRunResultDto result;
    result.joint_positions_deg = joint_positions_deg;
    result.joint_velocities_deg_s = joint_velocities_deg_s;
    result.joint_accelerations_deg_s2 = joint_accelerations_deg_s2;

    QString syncMessage;
    if (!SyncNativeModel(kinematicModel, dynamicModel, syncMessage))
    {
        result.message = syncMessage;
        return result;
    }

#if !defined(ROBOSDP_HAVE_PINOCCHIO)
    result.message = QStringLiteral("当前构建未启用 Pinocchio C++ 依赖。");
    return result;
#else
    if (m_native_kernel_state == nullptr ||
        !m_native_kernel_state->ready ||
        m_native_kernel_state->model == nullptr ||
        m_native_kernel_state->data == nullptr)
    {
        result.message = QStringLiteral("Dynamics 共享 Pinocchio Model/Data 尚未就绪。");
        return result;
    }

    const int expectedQCount = static_cast<int>(m_native_kernel_state->model->nq);
    const int expectedVCount = static_cast<int>(m_native_kernel_state->model->nv);

    if (static_cast<int>(joint_positions_deg.size()) != expectedQCount)
    {
        result.message = QStringLiteral("输入关节位置数量=%1，但原生模型 nq=%2。")
                             .arg(joint_positions_deg.size())
                             .arg(expectedQCount);
        return result;
    }

    if (static_cast<int>(joint_velocities_deg_s.size()) != expectedVCount)
    {
        result.message = QStringLiteral("输入关节速度数量=%1，但原生模型 nv=%2。")
                             .arg(joint_velocities_deg_s.size())
                             .arg(expectedVCount);
        return result;
    }

    if (static_cast<int>(joint_accelerations_deg_s2.size()) != expectedVCount)
    {
        result.message = QStringLiteral("输入关节加速度数量=%1，但原生模型 nv=%2。")
                             .arg(joint_accelerations_deg_s2.size())
                             .arg(expectedVCount);
        return result;
    }

    if (static_cast<int>(m_native_kernel_state->native_position_offsets_deg.size()) != expectedQCount)
    {
        result.message = QStringLiteral("内部零位偏移数量=%1，但原生模型 nq=%2。")
                             .arg(m_native_kernel_state->native_position_offsets_deg.size())
                             .arg(expectedQCount);
        return result;
    }

    try
    {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(expectedQCount);
        Eigen::VectorXd v = Eigen::VectorXd::Zero(expectedVCount);
        Eigen::VectorXd a = Eigen::VectorXd::Zero(expectedVCount);

        result.native_positions_rad.resize(static_cast<std::size_t>(expectedQCount), 0.0);
        result.native_velocities_rad_s.resize(static_cast<std::size_t>(expectedVCount), 0.0);
        result.native_accelerations_rad_s2.resize(static_cast<std::size_t>(expectedVCount), 0.0);

        for (int index = 0; index < expectedQCount; ++index)
        {
            const double positionDeg = joint_positions_deg[static_cast<std::size_t>(index)];
            if (!IsFiniteValue(positionDeg))
            {
                result.message = QStringLiteral("第 %1 个关节位置不是有效数值。").arg(index + 1);
                return result;
            }

            const double nativePositionRad = DegToRad(
                positionDeg +
                m_native_kernel_state->native_position_offsets_deg[static_cast<std::size_t>(index)]);
            q[static_cast<Eigen::Index>(index)] = nativePositionRad;
            result.native_positions_rad[static_cast<std::size_t>(index)] = nativePositionRad;
        }

        for (int index = 0; index < expectedVCount; ++index)
        {
            const double velocityDegS = joint_velocities_deg_s[static_cast<std::size_t>(index)];
            const double accelerationDegS2 = joint_accelerations_deg_s2[static_cast<std::size_t>(index)];
            if (!IsFiniteValue(velocityDegS))
            {
                result.message = QStringLiteral("第 %1 个关节速度不是有效数值。").arg(index + 1);
                return result;
            }
            if (!IsFiniteValue(accelerationDegS2))
            {
                result.message = QStringLiteral("第 %1 个关节加速度不是有效数值。").arg(index + 1);
                return result;
            }

            const double nativeVelocityRadS = DegToRad(velocityDegS);
            const double nativeAccelerationRadS2 = DegToRad(accelerationDegS2);
            v[static_cast<Eigen::Index>(index)] = nativeVelocityRadS;
            a[static_cast<Eigen::Index>(index)] = nativeAccelerationRadS2;
            result.native_velocities_rad_s[static_cast<std::size_t>(index)] = nativeVelocityRadS;
            result.native_accelerations_rad_s2[static_cast<std::size_t>(index)] = nativeAccelerationRadS2;
        }

        const Eigen::VectorXd tau =
            pinocchio::rnea(*m_native_kernel_state->model, *m_native_kernel_state->data, q, v, a);

        result.joint_torques_nm.resize(static_cast<std::size_t>(tau.size()), 0.0);
        for (Eigen::Index index = 0; index < tau.size(); ++index)
        {
            result.joint_torques_nm[static_cast<std::size_t>(index)] = tau[index];
        }

        result.success = true;
        result.message = QStringLiteral("Dynamics Pinocchio 原生 RNEA Dry-Run 计算完成。");
        return result;
    }
    catch (const std::exception& exception)
    {
        result.message = QStringLiteral("Dynamics Pinocchio 原生 RNEA Dry-Run 异常：%1")
                             .arg(QString::fromUtf8(exception.what()));
        return result;
    }
    catch (...)
    {
        result.message = QStringLiteral("Dynamics Pinocchio 原生 RNEA Dry-Run 出现未知异常。");
        return result;
    }
#endif
}

} // namespace RoboSDP::Dynamics::Adapter
