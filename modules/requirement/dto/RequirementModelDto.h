#pragma once

#include <QJsonArray>
#include <QJsonObject>
#include <QString>

#include <array>
#include <vector>

namespace RoboSDP::Requirement::Dto
{

/**
 * @brief Requirement 模块中的关键工位 DTO。
 *
 * 保留 dto-schema-spec.md 中的基础字段，并为后续多工位扩展预留数组形态。
 */
struct RequirementKeyPoseDto
{
    QString pose_id = QStringLiteral("pose_001");
    QString name = QStringLiteral("工位1");
    std::array<double, 6> pose {0.65, 0.0, 0.45, 180.0, 0.0, 180.0};
    double position_tol = 0.2;
    double orientation_tol = 0.5;
    bool has_required_direction = false;
    std::array<double, 3> required_direction {0.0, 0.0, -1.0};
};

/// 项目元信息 DTO。
struct RequirementProjectMetaDto
{
    QString project_name;
    QString scenario_type = QStringLiteral("handling");
    QString description;
    QString selected_template_id;
    QString selected_template_name;
    QString selected_template_brand;
};

/// 负载需求 DTO。
struct RequirementLoadRequirementsDto
{
    double rated_payload = 10.0;
    double max_payload = 11.1;
    double tool_mass = 2.0;
    double fixture_mass = 1.0;
    std::array<double, 3> payload_cog {0.0, 0.0, 0.08};
    std::array<double, 6> payload_inertia {0.08, 0.08, 0.04, 0.0, 0.0, 0.0};
    bool off_center_load = false;
    double cable_drag_load = 15.0;

    /**
     * @brief 原样透传的负载变体集合。
     *
     * 页面暂不提供复杂编辑器，但在 JSON 加载后仍保留其内容，避免保存草稿时丢失未编辑数据。
     */
    QJsonArray load_variants;
};

/// 工作空间需求 DTO。
struct RequirementWorkspaceRequirementsDto
{
    double max_radius = 1.101;
    double min_radius = 0.18;
    double max_height = 1.15;
    double min_height = -0.35;
    std::vector<RequirementKeyPoseDto> key_poses;
    QJsonArray forbidden_regions;
    QJsonArray obstacle_regions;
    QJsonObject base_constraints;
};

/// 运动性能需求 DTO。
struct RequirementMotionRequirementsDto
{
    double max_linear_speed = 2.0;
    double max_angular_speed = 360.0;
    double max_acceleration = 8.0;
    double max_angular_acceleration = 720.0;
    double jerk_limit = 100.0;
    double takt_time = 1.2;
};

/// 精度需求 DTO。
struct RequirementAccuracyRequirementsDto
{
    double absolute_accuracy = 0.2;
    double repeatability = 0.02;
    double tracking_accuracy = 0.15;
    double orientation_accuracy = 0.1;
    double tcp_position_tol = 0.2;
    double tcp_orientation_tol = 0.5;
};

/// 可靠性需求 DTO。
struct RequirementReliabilityRequirementsDto
{
    double design_life = 80000.0;
    int cycle_count = 10000000;
    double duty_cycle = 0.6;
    double operating_hours_per_day = 16.0;
    double mtbf_target = 20000.0;
};

/// 派生工况摘要 DTO，仅做只读透传占位。
struct RequirementDerivedConditionsDto
{
    QJsonObject rated_case;
    QJsonObject peak_case;
};

/**
 * @brief Requirement 主 DTO。
 *
 * 字段命名、分组结构与 JSON 键命名严格对齐 dto-schema-spec.md，
 * 便于后续持久化、校验器和页面绑定保持一致。
 */
struct RequirementModelDto
{
    RequirementProjectMetaDto project_meta;
    RequirementLoadRequirementsDto load_requirements;
    RequirementWorkspaceRequirementsDto workspace_requirements;
    RequirementMotionRequirementsDto motion_requirements;
    RequirementAccuracyRequirementsDto accuracy_requirements;
    RequirementReliabilityRequirementsDto reliability_requirements;
    RequirementDerivedConditionsDto derived_conditions;

    /// 创建默认 DTO，保证页面首次打开即可基于典型 10 kg 级 6R 机器人编辑。
    static RequirementModelDto CreateDefault()
    {
        RequirementModelDto dto;
        dto.workspace_requirements.key_poses.push_back(RequirementKeyPoseDto {});
        dto.project_meta.description = QStringLiteral("参考 KUKA KR 10 R1100-2 级别 6R 工业机器人任务需求默认值");
        dto.workspace_requirements.base_constraints.insert(QStringLiteral("base_mount_type"), QStringLiteral("floor"));
        dto.workspace_requirements.base_constraints.insert(QStringLiteral("hollow_wrist_required"), false);
        dto.workspace_requirements.base_constraints.insert(QStringLiteral("reserved_channel_diameter_mm"), 35.0);
        return dto;
    }
};

} // namespace RoboSDP::Requirement::Dto
