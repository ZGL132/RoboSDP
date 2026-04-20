#include "modules/kinematics/adapter/PinocchioIkSolverAdapter.h"

#include "core/errors/ErrorCode.h"
#include "modules/kinematics/adapter/PinocchioKinematicBackendAdapter.h"

#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include <cmath>
#include <limits>

namespace RoboSDP::Kinematics::Adapter
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double DegToRad(double valueDeg)
{
    return valueDeg * kPi / 180.0;
}

double RadToDeg(double valueRad)
{
    return valueRad * 180.0 / kPi;
}

bool IsFiniteValue(double value)
{
    return std::isfinite(value);
}

double NormalizeAngleDeg(double angleDeg)
{
    double normalized = std::fmod(angleDeg + 180.0, 360.0);
    if (normalized < 0.0)
    {
        normalized += 360.0;
    }
    return normalized - 180.0;
}

double ClampToRange(double value, const std::array<double, 2>& range)
{
    return std::max(range[0], std::min(range[1], value));
}

QString BuildCacheKey(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    return QStringLiteral("%1|%2|%3|%4")
        .arg(model.modeling_mode.trimmed())
        .arg(model.parameter_convention.trimmed())
        .arg(model.joint_order_signature.trimmed())
        .arg(model.urdf_source_path.trimmed());
}

} // namespace

struct PinocchioIkSolverAdapter::NativeKernelState
{
    QString cache_key;
    int build_count = 0;
    bool ready = false;
    QString last_message;
};

PinocchioIkSolverAdapter::PinocchioIkSolverAdapter(RoboSDP::Logging::ILogger* logger)
    : m_logger(logger)
    , m_native_kernel_state(std::make_unique<NativeKernelState>())
{
}

PinocchioIkSolverAdapter::~PinocchioIkSolverAdapter() = default;

QString PinocchioIkSolverAdapter::SolverId() const
{
    return QStringLiteral("pinocchio_numeric_ik");
}

QString PinocchioIkSolverAdapter::SolverDescription() const
{
    return QStringLiteral("Pinocchio 原生数值 IK 求解器（纯 Pinocchio 模式）。");
}

RoboSDP::Kinematics::Dto::IkResultDto PinocchioIkSolverAdapter::SolveIk(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const RoboSDP::Kinematics::Dto::IkRequestDto& request) const
{
    RoboSDP::Kinematics::Dto::IkResultDto result;
    result.joint_positions_deg = request.seed_joint_positions_deg;

    const auto failWithReason = [this, &request](const QString& reason)
        -> RoboSDP::Kinematics::Dto::IkResultDto
    {
        RoboSDP::Kinematics::Dto::IkResultDto failureResult;
        failureResult.joint_positions_deg = request.seed_joint_positions_deg;
        failureResult.success = false;
        failureResult.message = QStringLiteral("Pinocchio IK 求解失败：%1").arg(reason);
        LogMessage(
            RoboSDP::Logging::LogLevel::Warning,
            QStringLiteral("SolveIk"),
            failureResult.message);
        return failureResult;
    };

    QString failureReason;
    if (!SyncNativeModel(model, failureReason))
    {
        return failWithReason(failureReason);
    }

    const std::vector<double> seedJointPositionsDeg = BuildSeedJointPositionsDeg(model, request);
    if (seedJointPositionsDeg.size() != model.joint_limits.size())
    {
        return failWithReason(QStringLiteral("IK 初始种子数量=%1，但 joint_limits=%2。")
            .arg(seedJointPositionsDeg.size())
            .arg(model.joint_limits.size()));
    }

    const int maxIterations = std::max(1, model.ik_solver_config.max_iterations);
    const double positionToleranceM =
        IsFiniteValue(model.ik_solver_config.position_tolerance_mm) &&
            model.ik_solver_config.position_tolerance_mm > 0.0
        ? model.ik_solver_config.position_tolerance_mm / 1000.0
        : 0.001;
    const double orientationToleranceRad =
        IsFiniteValue(model.ik_solver_config.orientation_tolerance_deg) &&
            model.ik_solver_config.orientation_tolerance_deg > 0.0
        ? DegToRad(model.ik_solver_config.orientation_tolerance_deg)
        : DegToRad(1.0);
    const double stepGain =
        IsFiniteValue(model.ik_solver_config.step_gain) && model.ik_solver_config.step_gain > 0.0
        ? std::min(model.ik_solver_config.step_gain, 1.0)
        : 0.35;
    const double damping = 1.0e-6;

    PinocchioKinematicBackendAdapter backend(m_logger);
    std::vector<double> qDeg = seedJointPositionsDeg;
    double finalPositionErrorM = std::numeric_limits<double>::infinity();
    double finalOrientationErrorRad = std::numeric_limits<double>::infinity();

    for (int iteration = 0; iteration < maxIterations; ++iteration)
    {
        const auto fkResult = backend.EvaluateNativeFkDryRun(model, qDeg);
        if (!fkResult.success)
        {
            return failWithReason(fkResult.message);
        }

        const auto jacobianResult = backend.EvaluateNativeJacobianDryRun(model, qDeg);
        if (!jacobianResult.success)
        {
            return failWithReason(jacobianResult.message);
        }

        Eigen::Matrix<double, 6, 1> errorVector = Eigen::Matrix<double, 6, 1>::Zero();
        errorVector[0] = request.target_pose.position_m[0] - fkResult.tcp_pose.position_m[0];
        errorVector[1] = request.target_pose.position_m[1] - fkResult.tcp_pose.position_m[1];
        errorVector[2] = request.target_pose.position_m[2] - fkResult.tcp_pose.position_m[2];
        errorVector[3] = DegToRad(NormalizeAngleDeg(request.target_pose.rpy_deg[0] - fkResult.tcp_pose.rpy_deg[0]));
        errorVector[4] = DegToRad(NormalizeAngleDeg(request.target_pose.rpy_deg[1] - fkResult.tcp_pose.rpy_deg[1]));
        errorVector[5] = DegToRad(NormalizeAngleDeg(request.target_pose.rpy_deg[2] - fkResult.tcp_pose.rpy_deg[2]));

        finalPositionErrorM = errorVector.head<3>().norm();
        finalOrientationErrorRad = errorVector.tail<3>().norm();
        if (finalPositionErrorM <= positionToleranceM && finalOrientationErrorRad <= orientationToleranceRad)
        {
            result.success = true;
            result.joint_positions_deg = qDeg;
            result.position_error_mm = finalPositionErrorM * 1000.0;
            result.orientation_error_deg = RadToDeg(finalOrientationErrorRad);
            result.iteration_count = iteration + 1;
            result.message = QStringLiteral("IK 求解完成：由 Pinocchio 原生数值迭代求解器收敛。");
            LogMessage(
                RoboSDP::Logging::LogLevel::Info,
                QStringLiteral("SolveIk"),
                result.message);
            return result;
        }

        if (jacobianResult.rows != 6 ||
            jacobianResult.cols != static_cast<int>(qDeg.size()) ||
            jacobianResult.jacobian_matrix.size() != static_cast<std::size_t>(jacobianResult.rows * jacobianResult.cols))
        {
            return failWithReason(QStringLiteral("Pinocchio Jacobian 维度异常，无法执行数值 IK。"));
        }

        Eigen::MatrixXd jacobian(6, static_cast<Eigen::Index>(qDeg.size()));
        for (int row = 0; row < 6; ++row)
        {
            for (int col = 0; col < static_cast<int>(qDeg.size()); ++col)
            {
                jacobian(row, col) = jacobianResult.jacobian_matrix[static_cast<std::size_t>(row * jacobianResult.cols + col)];
            }
        }

        Eigen::Matrix<double, 6, 6> jjTranspose = jacobian * jacobian.transpose();
        jjTranspose.diagonal().array() += damping;
        const Eigen::VectorXd deltaQRad = jacobian.transpose() * jjTranspose.ldlt().solve(errorVector);

        for (std::size_t index = 0; index < qDeg.size(); ++index)
        {
            const double candidateDeg = qDeg[index] + RadToDeg(stepGain * deltaQRad[static_cast<Eigen::Index>(index)]);
            qDeg[index] = ClampToRange(candidateDeg, model.joint_limits[index].hard_limit);
        }
    }

    return failWithReason(
        QStringLiteral("Pinocchio IK 在 %1 次迭代内未收敛（位置误差=%2 mm，姿态误差=%3 deg）。")
            .arg(maxIterations)
            .arg(finalPositionErrorM * 1000.0, 0, 'f', 6)
            .arg(RadToDeg(finalOrientationErrorRad), 0, 'f', 6));
}

bool PinocchioIkSolverAdapter::SyncNativeModel(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    QString& failureReason) const
{
    if (m_native_kernel_state == nullptr)
    {
        m_native_kernel_state = std::make_unique<NativeKernelState>();
    }

    const QString cacheKey = BuildCacheKey(model);
    if (m_native_kernel_state->ready && m_native_kernel_state->cache_key == cacheKey)
    {
        failureReason.clear();
        return true;
    }

    PinocchioKinematicBackendAdapter backend(m_logger);
    const auto buildResult = backend.BuildNormalizedContext(model);
    if (!buildResult.IsSuccess() || !buildResult.status.shared_robot_kernel_ready)
    {
        m_native_kernel_state->ready = false;
        m_native_kernel_state->last_message = buildResult.status.status_message.isEmpty()
            ? QStringLiteral("Pinocchio IK 原生上下文未就绪。")
            : buildResult.status.status_message;
        failureReason = m_native_kernel_state->last_message.isEmpty()
            ? QStringLiteral("Pinocchio IK 原生上下文未就绪。")
            : m_native_kernel_state->last_message;
        return false;
    }

    m_native_kernel_state->cache_key = cacheKey;
    m_native_kernel_state->ready = true;
    ++m_native_kernel_state->build_count;
    m_native_kernel_state->last_message = QStringLiteral("Pinocchio IK 原生上下文已就绪。");
    failureReason.clear();
    return true;
}

std::vector<double> PinocchioIkSolverAdapter::BuildSeedJointPositionsDeg(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const RoboSDP::Kinematics::Dto::IkRequestDto& request) const
{
    std::vector<double> seed = request.seed_joint_positions_deg;
    if (seed.size() == model.joint_limits.size())
    {
        return seed;
    }

    // 中文说明：当外部未提供完整 seed 时，退回到软限位中点，保持求解起点稳定。
    seed.assign(model.joint_limits.size(), 0.0);
    for (std::size_t index = 0; index < model.joint_limits.size(); ++index)
    {
        const auto& softLimit = model.joint_limits[index].soft_limit;
        seed[index] = 0.5 * (softLimit[0] + softLimit[1]);
    }
    return seed;
}

void PinocchioIkSolverAdapter::LogMessage(
    RoboSDP::Logging::LogLevel level,
    const QString& actionName,
    const QString& message) const
{
    if (m_logger == nullptr)
    {
        return;
    }

    m_logger->Log(
        level,
        message,
        RoboSDP::Errors::ErrorCode::Ok,
        {
            QStringLiteral("Kinematics"),
            actionName,
            QStringLiteral("PinocchioIkSolverAdapter")});
}

} // namespace RoboSDP::Kinematics::Adapter
