#pragma once

#include "modules/kinematics/dto/KinematicModelDto.h"

#include <QString>

#include <array>
#include <vector>

namespace RoboSDP::Kinematics::Dto
{

/// @brief 单个连杆位姿摘要 DTO，用于向结果摘要和后续可视化传递各连杆坐标系结果。
struct LinkPoseDto
{
    QString link_id;
    CartesianPoseDto pose;
};

/// @brief FK 请求 DTO，按当前模型关节顺序提供角度输入。
struct FkRequestDto
{
    std::vector<double> joint_positions_deg = std::vector<double>(6, 0.0);
};

/// @brief FK 结果 DTO，保存末端位姿、连杆位姿和齐次变换矩阵摘要。
struct FkResultDto
{
    bool success = false;
    QString message;
    std::vector<double> joint_positions_deg;
    CartesianPoseDto tcp_pose;
    std::vector<LinkPoseDto> link_poses;
    std::array<double, 16> tcp_transform_matrix {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0};
};

/// @brief IK 请求 DTO，提供目标 TCP 位姿和求解种子。
struct IkRequestDto
{
    CartesianPoseDto target_pose;
    std::vector<double> seed_joint_positions_deg = std::vector<double>(6, 0.0);
};

/// @brief IK 结果 DTO，保存求解状态、关节解和误差指标。
struct IkResultDto
{
    bool success = false;
    QString message;
    std::vector<double> joint_positions_deg;
    double position_error_mm = 0.0;
    double orientation_error_deg = 0.0;
    int iteration_count = 0;
};

/// @brief 工作空间采样请求 DTO，仅描述采样规模，不承载具体后端状态。
struct WorkspaceRequestDto
{
    int sample_count = 128;
};

/// @brief 单个工作空间采样点 DTO，保存采样关节值和对应 TCP 位姿。
struct WorkspacePointDto
{
    std::vector<double> joint_positions_deg;
    CartesianPoseDto tcp_pose;
    double condition_number = 0.0;  // Jacobian 条件数（用于奇异区识别，0 表示未计算）
    double manipulability = 0.0;    // 可操作度（0 表示未计算）
};

/// @brief 工作空间奇异区分析请求 DTO。
struct SingularityAnalysisRequestDto
{
    int sample_count = 2000;
    double condition_threshold = 1000.0;  // 条件数阈值，超过则标记为奇异
};

/// @brief 工作空间基础采样结果 DTO，用于保存当前可达空间的最小统计摘要。
struct WorkspaceResultDto
{
    bool success = false;
    QString message;
    int requested_sample_count = 0;
    int reachable_sample_count = 0;
    double max_radius_m = 0.0;
    std::array<double, 3> min_position_m {0.0, 0.0, 0.0};
    std::array<double, 3> max_position_m {0.0, 0.0, 0.0};
    std::vector<WorkspacePointDto> sampled_points;
};

/// @brief 运动学后端状态摘要 DTO，仅保存轻量诊断信息，不暴露 Pinocchio 原生对象。
struct KinematicBackendStateSummaryDto
{
    /// @brief 当前工作区默认采用的运动学后端类型，用于说明保存草稿时的主路径选择。
    QString default_backend_type = QStringLiteral("pinocchio_kinematic_backend");

    /// @brief 最近一次参与业务编排的后端类型，用于帮助定位 FK 与 Workspace 结果的来源。
    QString active_backend_type = QStringLiteral("pinocchio_kinematic_backend");

    /// @brief 共享机器人内核当前所处的准备阶段，用于表达“仅预留入口”还是“已具备切换条件”。
    QString shared_kernel_stage = QStringLiteral("pinocchio_primary");

    /// @brief 标记共享机器人内核是否已经具备可用状态，当前阶段默认保持 false。
    bool shared_robot_kernel_ready = false;

    /// @brief 面向日志和排障的信息摘要，说明为何当前仍未切换到共享 Pinocchio 后端。
    QString status_message = QStringLiteral("当前默认使用 Pinocchio 共享运动学后端。");
};

/// @brief Kinematics 页面工作态 DTO，保存当前模型与最近一次分析结果摘要。
struct KinematicsWorkspaceStateDto
{
    KinematicModelDto current_model;
    FkResultDto last_fk_result;
    IkResultDto last_ik_result;
    WorkspaceResultDto last_workspace_result;

    /// @brief 后端状态摘要，仅用于业务诊断与后续替换准备，不参与复杂算法计算。
    KinematicBackendStateSummaryDto backend_summary;

    static KinematicsWorkspaceStateDto CreateDefault()
    {
        KinematicsWorkspaceStateDto dto;
        dto.current_model = KinematicModelDto::CreateDefault();
        return dto;
    }
};

/// @brief Jacobian 分析结果 DTO（2.5.3 奇异区域识别 + 2.5.4 可操作度分析）
/// 包含奇异值、条件数、Yoshikawa 可操作度等指标。
struct JacobianAnalysisDto
{
    bool success = false;
    QString message;
    int rows = 6;
    int cols = 0;
    std::vector<double> singular_values;   // 从大到小排列
    double condition_number = 0.0;          // σ_max / σ_min
    double manipulability = 0.0;            // Yoshikawa w = sqrt(det(J*J^T))
    double min_singular_value = 0.0;
    double max_singular_value = 0.0;
    bool is_singular = false;
    std::vector<double> joint_positions_deg;
};

/// @brief 单点可达性检测结果 DTO（2.5.1 关键工位可达性检测）
struct ReachabilityCheckResultDto
{
    bool reachable = false;
    QString message;
    CartesianPoseDto target_pose;
    int total_seeds = 20;
    int converged_count = 0;
    double best_position_error_mm = 0.0;
    double best_orientation_error_deg = 0.0;
    std::vector<double> best_joint_positions_deg;
    int best_seed_iterations = 0;
    std::vector<double> position_errors_mm;  // 各种子的位置误差列表
    std::vector<double> orientation_errors_deg; // 各种子的姿态误差列表
};

/// @brief 姿态可达性分析结果 DTO（2.5.2 姿态可达性分析）
struct OrientationReachabilityResultDto
{
    bool success = false;
    QString message;
    std::array<double, 3> position_m {0.0, 0.0, 0.0};
    int total_samples = 0;
    int reachable_count = 0;
    double reachability_ratio = 0.0;  // 0~1
    /// @brief 各轴步进数（如 7 表示每轴分 7 步 → 343 个姿态）
    int steps_per_axis = 7;
    /// @brief 每个样本的 Euler RPY 范围 [deg]
    double range_deg = 180.0;
    /// @brief 可达姿态的 RPY 列表
    std::vector<std::array<double, 3>> reachable_rpy_deg;
    /// @brief 不可达姿态的 RPY 列表
    std::vector<std::array<double, 3>> unreachable_rpy_deg;
};

} // namespace RoboSDP::Kinematics::Dto
