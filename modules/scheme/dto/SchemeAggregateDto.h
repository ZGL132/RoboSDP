#pragma once

#include <QString>

namespace RoboSDP::Scheme::Dto
{

/// 方案快照引用的上游对象索引。
struct SchemeUpstreamRefsDto
{
    /// Requirement 当前没有独立 ID，本轮优先记录稳定文件引用。
    QString requirement_ref = QStringLiteral("requirements/requirement-model.json");
    QString topology_ref;
    QString kinematic_ref;
    QString dynamic_ref;
    QString selection_ref = QStringLiteral("selection/drivetrain-selection.json");
    QString planning_scene_ref;
    QString planning_result_ref;
};

/// Requirement 摘要，仅保留方案层需要展示的核心信息。
struct RequirementSummaryDto
{
    bool available = false;
    QString project_name;
    QString scenario_type;
    double rated_payload_kg = 0.0;
    double max_payload_kg = 0.0;
    int key_pose_count = 0;
    double takt_time_s = 0.0;
    QString summary_text;
};

/// Topology 摘要，仅保留推荐构型和关节规模等关键字段。
struct TopologySummaryDto
{
    bool available = false;
    QString topology_id;
    QString topology_name;
    QString template_id;
    QString robot_type;
    int joint_count = 0;
    int candidate_count = 0;
    QString recommended_candidate_id;
    QString summary_text;
};

/// Kinematics 摘要，仅保留模型约定、工作空间和最近一次求解摘要。
struct KinematicsSummaryDto
{
    bool available = false;
    QString kinematic_id;
    QString parameter_convention;
    int joint_count = 0;
    bool last_fk_success = false;
    bool last_ik_success = false;
    int workspace_reachable_sample_count = 0;
    double workspace_max_radius_m = 0.0;
    QString summary_text;
};

/// Dynamics 摘要，仅保留负载包络和轨迹统计核心结果。
struct DynamicsSummaryDto
{
    bool available = false;
    QString dynamic_id;
    int trajectory_count = 0;
    int peak_stat_count = 0;
    int rms_stat_count = 0;
    int envelope_joint_count = 0;
    double max_peak_torque_nm = 0.0;
    QString summary_text;
};

/// Selection 摘要，仅保留联合驱动链结果的关节覆盖情况。
struct SelectionSummaryDto
{
    bool available = false;
    bool success = false;
    int joint_selection_count = 0;
    int recommended_joint_count = 0;
    QString summary_text;
};

/// Planning 摘要，仅保留可行性、碰撞和节拍结论。
struct PlanningSummaryDto
{
    bool available = false;
    bool success = false;
    int request_count = 0;
    int verification_count = 0;
    int collision_issue_count = 0;
    int self_collision_issue_count = 0;
    bool cycle_time_within_target = false;
    QString summary_text;
};

/// 方案层记录的最小导出元信息。
struct SchemeExportMetaDto
{
    QString default_export_format = QStringLiteral("json");
    QString default_export_relative_path = QStringLiteral("exports/scheme-export.json");
    QString last_export_relative_path;
    QString last_exported_at;
};

/**
 * @brief Scheme 聚合对象。
 *
 * 本对象严格只保存：
 * 1. 上游对象引用；
 * 2. 各模块关键摘要字段；
 * 3. 必要的导出元信息。
 * 不复制各模块完整原始 DTO，避免方案层演化成二次持久化中心。
 */
struct SchemeAggregateDto
{
    SchemeUpstreamRefsDto refs;
    RequirementSummaryDto requirement_summary;
    TopologySummaryDto topology_summary;
    KinematicsSummaryDto kinematics_summary;
    DynamicsSummaryDto dynamics_summary;
    SelectionSummaryDto selection_summary;
    PlanningSummaryDto planning_summary;
    SchemeExportMetaDto export_meta;
    int available_module_count = 0;
    QString completeness_summary;
};

} // namespace RoboSDP::Scheme::Dto
