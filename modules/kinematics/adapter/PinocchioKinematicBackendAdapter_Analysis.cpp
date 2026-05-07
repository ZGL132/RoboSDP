#include "modules/kinematics/adapter/ComputeSvdMetrics.h"
#include "modules/kinematics/adapter/PinocchioKinematicBackendAdapter.h"
#include "modules/kinematics/adapter/PinocchioKinematicBackendAdapterInternal.h"

#if defined(ROBOSDP_HAVE_PINOCCHIO)

#include <Eigen/SVD>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <vector>

namespace RoboSDP::Kinematics::Adapter
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double DegToRad(double deg) { return deg * kPi / 180.0; }
constexpr double RadToDeg(double rad) { return rad * 180.0 / kPi; }

/// @brief 生成确定性的伪随机比例（范围 0.0 到 1.0），使用 SplitMix 散列保证相邻样本不相关。
double DeterministicRatio(int sampleIndex, int jointIndex)
{
    unsigned int h = static_cast<unsigned int>(sampleIndex) * 1664525U
                   + static_cast<unsigned int>(jointIndex) * 1013904223U
                   + 2654435761U;
    h = ((h >> 16) ^ h) * 0x45d9f3bU;
    h = (h >> 16) ^ h;
    return static_cast<double>(h) / 4294967296.0;
}

/// @brief 将 DTO 中的平移 + RPY 转换为 Pinocchio 使用的 SE3。
pinocchio::SE3 BuildSe3FromPose(const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    const Eigen::AngleAxisd roll(DegToRad(pose.rpy_deg[0]), Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(DegToRad(pose.rpy_deg[1]), Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(DegToRad(pose.rpy_deg[2]), Eigen::Vector3d::UnitZ());
    const Eigen::Matrix3d rotation = (yaw * pitch * roll).toRotationMatrix();
    const Eigen::Vector3d translation(pose.position_m[0], pose.position_m[1], pose.position_m[2]);
    return pinocchio::SE3(rotation, translation);
}

/// @brief 将 Pinocchio SE3 位姿转换为 DTO。
RoboSDP::Kinematics::Dto::CartesianPoseDto BuildPoseFromSe3(const pinocchio::SE3& tf)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m[0] = tf.translation()[0];
    pose.position_m[1] = tf.translation()[1];
    pose.position_m[2] = tf.translation()[2];
    const Eigen::Matrix3d rot = tf.rotation();
    double roll, pitch, yaw;
    pitch = std::asin(-rot(2, 0));
    if (std::abs(pitch) < kPi / 2.0 - 1e-6)
    {
        roll = std::atan2(rot(2, 1), rot(2, 2));
        yaw = std::atan2(rot(1, 0), rot(0, 0));
    }
    else
    {
        roll = 0.0;
        yaw = std::atan2(-rot(0, 1), rot(1, 1));
    }
    pose.rpy_deg = {RadToDeg(roll), RadToDeg(pitch), RadToDeg(yaw)};
    return pose;
}

} // anonymous namespace

// =====================================================================
// SampleWorkspaceWithSingularity — 带奇异区识别的工作空间采样
// =====================================================================

RoboSDP::Kinematics::Dto::WorkspaceResultDto
PinocchioKinematicBackendAdapter::SampleWorkspaceWithSingularity(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const RoboSDP::Kinematics::Dto::SingularityAnalysisRequestDto& request) const
{
    RoboSDP::Kinematics::Dto::WorkspaceResultDto result;
    result.requested_sample_count = std::max(1, request.sample_count);
    const auto failWithReason = [this, &request](const QString& reason)
        -> RoboSDP::Kinematics::Dto::WorkspaceResultDto
    {
        RoboSDP::Kinematics::Dto::WorkspaceResultDto failureResult;
        failureResult.requested_sample_count = std::max(1, request.sample_count);
        failureResult.success = false;
        failureResult.message = QStringLiteral("Pinocchio 奇异区分析采样失败：%1").arg(reason);
        return failureResult;
    };

    const auto buildResult = BuildNormalizedContext(model);
    if (!buildResult.IsSuccess() || !buildResult.status.shared_robot_kernel_ready)
        return failWithReason(QStringLiteral("共享内核未就绪。"));

    if (m_native_kernel_state == nullptr || !m_native_kernel_state->ready)
        return failWithReason(QStringLiteral("原生 Model/Data 未实例化。"));

    const auto nq = static_cast<std::size_t>(m_native_kernel_state->model->nq);
    const auto nv = static_cast<std::size_t>(m_native_kernel_state->model->nv);
    if (nq == 0) return failWithReason(QStringLiteral("nq=0。"));

    try
    {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(m_native_kernel_state->model->nq);
        Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, m_native_kernel_state->model->nv);
        std::vector<double> jointPosDeg(nq, 0.0);
        result.sampled_points.reserve(result.requested_sample_count);

        int singularCount = 0;
        for (int si = 0; si < result.requested_sample_count; ++si)
        {
            for (std::size_t ji = 0; ji < nq; ++ji)
            {
                const auto& limits = model.joint_limits[ji].hard_limit;
                const double ratio = DeterministicRatio(si, static_cast<int>(ji));
                jointPosDeg[ji] = limits[0] + (limits[1] - limits[0]) * ratio;
                q[static_cast<Eigen::Index>(ji)] = DegToRad(
                    jointPosDeg[ji] + m_native_kernel_state->native_position_offsets_deg[ji]);
            }

            pinocchio::forwardKinematics(*m_native_kernel_state->model, *m_native_kernel_state->data, q);
            pinocchio::updateFramePlacements(*m_native_kernel_state->model, *m_native_kernel_state->data);

            // 提取 TCP 位姿
            const pinocchio::SE3& tf = m_native_kernel_state->data->oMf[m_native_kernel_state->tcp_frame_id];
            RoboSDP::Kinematics::Dto::WorkspacePointDto point;
            point.joint_positions_deg = jointPosDeg;
            point.tcp_pose = BuildPoseFromSe3(tf);

            // 计算 Jacobian 条件数
            jacobian.setZero();
            pinocchio::computeJointJacobians(*m_native_kernel_state->model, *m_native_kernel_state->data, q);
            pinocchio::getFrameJacobian(
                *m_native_kernel_state->model, *m_native_kernel_state->data,
                m_native_kernel_state->tcp_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

            {
                std::vector<double> tmpSv;
                ComputeSvdMetrics(jacobian, point.condition_number, point.manipulability, tmpSv);
            }

            if (point.condition_number > request.condition_threshold)
                ++singularCount;

            result.sampled_points.push_back(point);
            ++result.reachable_sample_count;

            // 更新包围盒
            result.max_radius_m = std::max(result.max_radius_m,
                std::sqrt(tf.translation()[0] * tf.translation()[0] +
                          tf.translation()[1] * tf.translation()[1] +
                          tf.translation()[2] * tf.translation()[2]));
            if (result.reachable_sample_count == 1)
            {
                result.min_position_m = {tf.translation()[0], tf.translation()[1], tf.translation()[2]};
                result.max_position_m = result.min_position_m;
            }
            else
            {
                for (int a = 0; a < 3; ++a)
                {
                    result.min_position_m[a] = std::min(result.min_position_m[a], tf.translation()[a]);
                    result.max_position_m[a] = std::max(result.max_position_m[a], tf.translation()[a]);
                }
            }
        }

        result.success = (result.reachable_sample_count > 0);
        result.message = QStringLiteral("奇异区分析完成：采样 %1 点，奇异 %2 点（阈值 κ>%3）")
            .arg(result.reachable_sample_count).arg(singularCount).arg(request.condition_threshold, 0, 'f', 0);
        return result;
    }
    catch (const std::exception& e)
    {
        return failWithReason(QStringLiteral("异常：%1").arg(QString::fromUtf8(e.what())));
    }
}

// =====================================================================
// ComputeJacobianAnalysis — Jacobian 矩阵分析（奇异值、条件数、可操作度）
// =====================================================================

RoboSDP::Kinematics::Dto::JacobianAnalysisDto
PinocchioKinematicBackendAdapter::ComputeJacobianAnalysis(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const std::vector<double>& joint_positions_deg) const
{
    RoboSDP::Kinematics::Dto::JacobianAnalysisDto result;
    result.joint_positions_deg = joint_positions_deg;

    // 复用 build-context 和原生 Model/Data 同步
    const auto buildResult = BuildNormalizedContext(model);
    if (!buildResult.IsSuccess() || !buildResult.status.shared_robot_kernel_ready)
    {
        result.message = QStringLiteral("共享内核未就绪：%1").arg(buildResult.status.status_message);
        return result;
    }

    if (m_native_kernel_state == nullptr || !m_native_kernel_state->ready)
    {
        result.message = QStringLiteral("原生 Model/Data 未实例化。");
        return result;
    }

    try
    {
        const auto nq = static_cast<std::size_t>(m_native_kernel_state->model->nq);
        const auto nv = static_cast<std::size_t>(m_native_kernel_state->model->nv);
        if (joint_positions_deg.size() != nq)
        {
            result.message = QStringLiteral("关节角数量=%1 与模型 nq=%2 不匹配。")
                .arg(joint_positions_deg.size()).arg(nq);
            return result;
        }

        result.cols = static_cast<int>(nv);

        Eigen::VectorXd q = Eigen::VectorXd::Zero(m_native_kernel_state->model->nq);
        for (std::size_t i = 0; i < nq; ++i)
        {
            const double nativeDeg = joint_positions_deg[i]
                + m_native_kernel_state->native_position_offsets_deg[i];
            q[static_cast<Eigen::Index>(i)] = DegToRad(nativeDeg);
        }

        pinocchio::computeJointJacobians(
            *m_native_kernel_state->model,
            *m_native_kernel_state->data,
            q);
        pinocchio::updateFramePlacements(
            *m_native_kernel_state->model,
            *m_native_kernel_state->data);

        Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, m_native_kernel_state->model->nv);
        jacobian.setZero();
        pinocchio::getFrameJacobian(
            *m_native_kernel_state->model,
            *m_native_kernel_state->data,
            m_native_kernel_state->tcp_frame_id,
            pinocchio::LOCAL_WORLD_ALIGNED,
            jacobian);

        // 使用辅助函数计算奇异值、条件数和可操作度
        result.is_singular = ComputeSvdMetrics(
            jacobian,
            result.condition_number,
            result.manipulability,
            result.singular_values);

        if (!result.singular_values.empty())
        {
            result.max_singular_value = result.singular_values.front();
            result.min_singular_value = result.singular_values.back();
        }

        result.success = true;
        result.message = QStringLiteral("Jacobian 分析完成。可操作度=%1，条件数=%2%3")
            .arg(result.manipulability, 0, 'f', 6)
            .arg(result.condition_number, 0, 'f', 2)
            .arg(result.is_singular ? QStringLiteral(" [奇异]") : QString());
        return result;
    }
    catch (const std::exception& e)
    {
        result.message = QStringLiteral("Jacobian 分析异常：%1").arg(QString::fromUtf8(e.what()));
        return result;
    }
}

} // namespace RoboSDP::Kinematics::Adapter
#endif // defined(ROBOSDP_HAVE_PINOCCHIO)
