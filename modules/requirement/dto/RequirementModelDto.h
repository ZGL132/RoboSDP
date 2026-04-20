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
 * 本轮仅实现最小可用的单工位录入能力，因此该结构保留
 * dto-schema-spec.md 中的基础字段，并为后续多工位扩展预留数组形态。
 */
struct RequirementKeyPoseDto
{
    QString pose_id = QStringLiteral("pose_001");
    QString name = QStringLiteral("工位1");
    std::array<double, 6> pose {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double position_tol = 0.0;
    double orientation_tol = 0.0;
    bool has_required_direction = false;
    std::array<double, 3> required_direction {0.0, 0.0, 0.0};
};

/// 项目元信息 DTO。
struct RequirementProjectMetaDto
{
    QString project_name;
    QString scenario_type = QStringLiteral("custom");
    QString description;
};

/// 负载需求 DTO。
struct RequirementLoadRequirementsDto
{
    double rated_payload = 0.0;
    double max_payload = 0.0;
    double tool_mass = 0.0;
    double fixture_mass = 0.0;
    std::array<double, 3> payload_cog {0.0, 0.0, 0.0};
    std::array<double, 6> payload_inertia {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool off_center_load = false;
    double cable_drag_load = 0.0;

    /**
     * @brief 原样透传的负载变体集合。
     *
     * 本轮页面不提供复杂编辑器，但在 JSON 加载后仍保留其内容，
     * 避免保存 Requirement 草稿时意外丢失未编辑数据。
     */
    QJsonArray load_variants;
};

/// 工作空间需求 DTO。
struct RequirementWorkspaceRequirementsDto
{
    double max_radius = 0.0;
    double min_radius = 0.0;
    double max_height = 0.0;
    double min_height = 0.0;
    std::vector<RequirementKeyPoseDto> key_poses;
    QJsonArray forbidden_regions;
    QJsonArray obstacle_regions;
    QJsonObject base_constraints;
};

/// 运动性能需求 DTO。
struct RequirementMotionRequirementsDto
{
    double max_linear_speed = 0.0;
    double max_angular_speed = 0.0;
    double max_acceleration = 0.0;
    double max_angular_acceleration = 0.0;
    double jerk_limit = 0.0;
    double takt_time = 0.0;
};

/// 精度需求 DTO。
struct RequirementAccuracyRequirementsDto
{
    double absolute_accuracy = 0.0;
    double repeatability = 0.0;
    double tracking_accuracy = 0.0;
    double orientation_accuracy = 0.0;
    double tcp_position_tol = 0.0;
    double tcp_orientation_tol = 0.0;
};

/// 可靠性需求 DTO。
struct RequirementReliabilityRequirementsDto
{
    double design_life = 0.0;
    int cycle_count = 0;
    double duty_cycle = 0.0;
    double operating_hours_per_day = 0.0;
    double mtbf_target = 0.0;
};

/// 派生工况摘要 DTO，本轮仅做只读透传占位。
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

    /// 创建最小默认 DTO，保证页面首次打开即可编辑。
    static RequirementModelDto CreateDefault()
    {
        RequirementModelDto dto;
        dto.workspace_requirements.key_poses.push_back(RequirementKeyPoseDto {});
        return dto;
    }
};

} // namespace RoboSDP::Requirement::Dto
