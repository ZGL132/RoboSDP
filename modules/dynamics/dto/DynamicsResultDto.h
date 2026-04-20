#pragma once

#include "modules/dynamics/dto/DynamicModelDto.h"

#include <QString>

#include <vector>

namespace RoboSDP::Dynamics::Dto
{

/// @brief 时间参数化之后的单时刻关节采样 DTO。
struct JointStateSampleDto
{
    double time_s = 0.0;
    std::vector<double> positions_deg;
    std::vector<double> velocities_deg_s;
    std::vector<double> accelerations_deg_s2;
};

/// @brief 单条轨迹的时间参数化结果 DTO。
struct ParameterizedTrajectoryDto
{
    QString trajectory_id;
    QString name;
    double duration_s = 0.0;
    std::vector<JointStateSampleDto> samples;
};

/// @brief 单个关节单时刻的逆动力学负载样本 DTO。
struct JointLoadSampleDto
{
    double time_s = 0.0;
    double torque_nm = 0.0;
    double speed_deg_s = 0.0;
    double power_w = 0.0;
};

/// @brief 单个关节的负载曲线 DTO。
struct JointLoadCurveDto
{
    QString joint_id;
    std::vector<JointLoadSampleDto> samples;
};

/// @brief 单条轨迹的逆动力学结果 DTO。
struct InverseDynamicsTrajectoryResultDto
{
    QString trajectory_id;
    QString name;
    bool success = false;
    /// @brief Stage 16 之后固定标记为共享 Pinocchio 内核。
    QString solver_backend = QStringLiteral("pinocchio_shared_kernel");
    /// @brief 为兼容既有 UI/日志结果结构保留该字段；纯 Pinocchio 模式下固定为 false。
    bool used_fallback = false;
    QString message;
    std::vector<JointLoadCurveDto> joint_curves;
};

/**
 * @brief Pinocchio 原生 RNEA 干跑结果 DTO。
 * @details
 * 本 DTO 只承载经过安全转换后的轻量结果，
 * 不向外暴露 Eigen::VectorXd、pinocchio::Model/Data 或任何三方原生对象。
 * 当前仅用于动力学共享内核的诊断验证，不接入额外旧后端。
 */
struct NativeRneaDryRunResultDto
{
    /// @brief 干跑是否成功；失败时 message 必须返回明确中文原因。
    bool success = false;

    /// @brief 中文状态说明，用于测试、日志和后续诊断面板。
    QString message;

    /// @brief 输入关节位置，单位为度，顺序遵循 joint_order_signature。
    std::vector<double> joint_positions_deg;

    /// @brief 输入关节速度，单位为度每秒，顺序遵循 joint_order_signature。
    std::vector<double> joint_velocities_deg_s;

    /// @brief 输入关节加速度，单位为度每二次方秒，顺序遵循 joint_order_signature。
    std::vector<double> joint_accelerations_deg_s2;

    /// @brief 叠加零位偏移后映射到原生模型的 q，单位为弧度。
    std::vector<double> native_positions_rad;

    /// @brief 映射到原生模型的速度向量 v，单位为弧度每秒。
    std::vector<double> native_velocities_rad_s;

    /// @brief 映射到原生模型的加速度向量 a，单位为弧度每二次方秒。
    std::vector<double> native_accelerations_rad_s2;

    /// @brief 原生 RNEA 计算得到的关节力矩，单位为牛米。
    std::vector<double> joint_torques_nm;
};

/// @brief 峰值统计 DTO。
struct PeakStatDto
{
    QString joint_id;
    QString trajectory_id;
    double peak_torque_nm = 0.0;
    double peak_speed_deg_s = 0.0;
    double peak_power_w = 0.0;
};

/// @brief RMS 统计 DTO。
struct RmsStatDto
{
    QString joint_id;
    QString trajectory_id;
    double rms_torque_nm = 0.0;
};

/// @brief 负载包络中的单关节摘要 DTO。
struct LoadEnvelopeJointDto
{
    QString joint_id;
    QString source_trajectory_id;
    double peak_torque_nm = 0.0;
    double rms_torque_nm = 0.0;
    double peak_power_w = 0.0;
};

/// @brief 负载包络 DTO。
struct LoadEnvelopeDto
{
    std::vector<LoadEnvelopeJointDto> joints;
};

/// @brief Dynamics 页面与服务工作态 DTO。
struct DynamicsWorkspaceStateDto
{
    DynamicModelDto current_model;
    std::vector<ParameterizedTrajectoryDto> parameterized_trajectories;
    std::vector<InverseDynamicsTrajectoryResultDto> trajectory_results;
    std::vector<PeakStatDto> peak_stats;
    std::vector<RmsStatDto> rms_stats;
    LoadEnvelopeDto load_envelope;

    static DynamicsWorkspaceStateDto CreateDefault()
    {
        DynamicsWorkspaceStateDto dto;
        dto.current_model = DynamicModelDto::CreateDefault();
        return dto;
    }
};

} // namespace RoboSDP::Dynamics::Dto
