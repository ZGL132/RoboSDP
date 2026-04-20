#include "modules/requirement/validator/RequirementValidator.h"

#include <QJsonObject>
#include <QSet>

namespace RoboSDP::Requirement::Validation
{

namespace
{

void AddIssue(
    RequirementValidationResult& result,
    const QString& field,
    const QString& code,
    const QString& message,
    ValidationSeverity severity = ValidationSeverity::Error)
{
    result.issues.push_back({field, code, message, severity});
}

bool IsSupportedScenarioType(const QString& scenarioType)
{
    static const QSet<QString> kSupportedScenarioTypes {
        QStringLiteral("handling"),
        QStringLiteral("welding"),
        QStringLiteral("assembly"),
        QStringLiteral("grinding"),
        QStringLiteral("spraying"),
        QStringLiteral("custom")
    };

    return kSupportedScenarioTypes.contains(scenarioType);
}

bool IsSupportedBaseMountType(const QString& baseMountType)
{
    static const QSet<QString> kSupportedBaseMountTypes {
        QStringLiteral("floor"),
        QStringLiteral("wall"),
        QStringLiteral("ceiling"),
        QStringLiteral("pedestal")
    };

    return kSupportedBaseMountTypes.contains(baseMountType);
}

} // namespace

bool RequirementValidationResult::IsValid() const
{
    return ErrorCount() == 0;
}

int RequirementValidationResult::ErrorCount() const
{
    int count = 0;
    for (const ValidationIssue& issue : issues)
    {
        if (issue.severity == ValidationSeverity::Error)
        {
            ++count;
        }
    }

    return count;
}

int RequirementValidationResult::WarningCount() const
{
    int count = 0;
    for (const ValidationIssue& issue : issues)
    {
        if (issue.severity == ValidationSeverity::Warning)
        {
            ++count;
        }
    }

    return count;
}

QString ToString(ValidationSeverity severity)
{
    switch (severity)
    {
    case ValidationSeverity::Info:
        return QStringLiteral("INFO");
    case ValidationSeverity::Warning:
        return QStringLiteral("WARNING");
    case ValidationSeverity::Error:
        return QStringLiteral("ERROR");
    }

    return QStringLiteral("ERROR");
}

RequirementValidationResult RequirementValidator::Validate(
    const RoboSDP::Requirement::Dto::RequirementModelDto& model) const
{
    RequirementValidationResult result;

    if (model.project_meta.project_name.trimmed().isEmpty())
    {
        AddIssue(
            result,
            QStringLiteral("project_meta.project_name"),
            QStringLiteral("REQ_PROJECT_NAME_REQUIRED"),
            QStringLiteral("项目名称不能为空。"));
    }
    else if (model.project_meta.project_name.trimmed().size() > 128)
    {
        AddIssue(
            result,
            QStringLiteral("project_meta.project_name"),
            QStringLiteral("REQ_PROJECT_NAME_TOO_LONG"),
            QStringLiteral("项目名称长度不能超过 128 个字符。"));
    }

    if (!IsSupportedScenarioType(model.project_meta.scenario_type))
    {
        AddIssue(
            result,
            QStringLiteral("project_meta.scenario_type"),
            QStringLiteral("REQ_SCENARIO_TYPE_INVALID"),
            QStringLiteral("场景类型不在允许枚举范围内。"));
    }

    if (model.load_requirements.rated_payload < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("load_requirements.rated_payload"),
            QStringLiteral("REQ_RATED_PAYLOAD_NEGATIVE"),
            QStringLiteral("额定负载必须大于或等于 0。"));
    }

    if (model.load_requirements.max_payload < model.load_requirements.rated_payload)
    {
        AddIssue(
            result,
            QStringLiteral("load_requirements.max_payload"),
            QStringLiteral("REQ_MAX_PAYLOAD_LT_RATED"),
            QStringLiteral("最大负载必须大于或等于额定负载。"));
    }

    if (model.load_requirements.tool_mass < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("load_requirements.tool_mass"),
            QStringLiteral("REQ_TOOL_MASS_NEGATIVE"),
            QStringLiteral("工具质量必须大于或等于 0。"));
    }

    if (model.load_requirements.fixture_mass < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("load_requirements.fixture_mass"),
            QStringLiteral("REQ_FIXTURE_MASS_NEGATIVE"),
            QStringLiteral("工装质量必须大于或等于 0。"));
    }

    if (model.load_requirements.cable_drag_load < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("load_requirements.cable_drag_load"),
            QStringLiteral("REQ_CABLE_DRAG_NEGATIVE"),
            QStringLiteral("电缆拖曳负载必须大于或等于 0。"));
    }

    if (model.workspace_requirements.max_radius <= 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("workspace_requirements.max_radius"),
            QStringLiteral("REQ_MAX_RADIUS_INVALID"),
            QStringLiteral("最大工作半径必须大于 0。"));
    }

    if (model.workspace_requirements.min_radius < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("workspace_requirements.min_radius"),
            QStringLiteral("REQ_MIN_RADIUS_NEGATIVE"),
            QStringLiteral("最小工作半径必须大于或等于 0。"));
    }

    if (model.workspace_requirements.min_radius >= model.workspace_requirements.max_radius &&
        model.workspace_requirements.max_radius > 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("workspace_requirements.min_radius"),
            QStringLiteral("REQ_MIN_RADIUS_GE_MAX"),
            QStringLiteral("最小工作半径必须小于最大工作半径。"));
    }

    if (model.workspace_requirements.max_height <= model.workspace_requirements.min_height)
    {
        AddIssue(
            result,
            QStringLiteral("workspace_requirements.max_height"),
            QStringLiteral("REQ_MAX_HEIGHT_LE_MIN"),
            QStringLiteral("最大工作高度必须大于最小工作高度。"));
    }

    const QJsonObject baseConstraints = model.workspace_requirements.base_constraints;
    const QString baseMountType =
        baseConstraints.value(QStringLiteral("base_mount_type")).toString().trimmed();
    if (!baseMountType.isEmpty() && !IsSupportedBaseMountType(baseMountType))
    {
        AddIssue(
            result,
            QStringLiteral("workspace_requirements.base_constraints.base_mount_type"),
            QStringLiteral("REQ_BASE_MOUNT_TYPE_INVALID"),
            QStringLiteral("基座安装方式不在允许枚举范围内。"));
    }

    if (baseConstraints.contains(QStringLiteral("reserved_channel_diameter_mm")) &&
        baseConstraints.value(QStringLiteral("reserved_channel_diameter_mm")).toDouble() < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("workspace_requirements.base_constraints.reserved_channel_diameter_mm"),
            QStringLiteral("REQ_RESERVED_CHANNEL_NEGATIVE"),
            QStringLiteral("预留通道直径必须大于或等于 0。"));
    }

    if (model.workspace_requirements.key_poses.empty())
    {
        AddIssue(
            result,
            QStringLiteral("workspace_requirements.key_poses"),
            QStringLiteral("REQ_KEY_POSES_EMPTY"),
            QStringLiteral("关键工位至少需要 1 项。"));
    }
    else
    {
        QSet<QString> poseIds;
        for (std::size_t index = 0; index < model.workspace_requirements.key_poses.size(); ++index)
        {
            const auto& keyPose = model.workspace_requirements.key_poses.at(index);
            const QString prefix = QStringLiteral("workspace_requirements.key_poses[%1]").arg(index);

            if (keyPose.pose_id.trimmed().isEmpty())
            {
                AddIssue(
                    result,
                    prefix + QStringLiteral(".pose_id"),
                    QStringLiteral("REQ_KEY_POSE_ID_REQUIRED"),
                    QStringLiteral("关键工位 ID 不能为空。"));
            }
            else if (poseIds.contains(keyPose.pose_id.trimmed()))
            {
                AddIssue(
                    result,
                    prefix + QStringLiteral(".pose_id"),
                    QStringLiteral("REQ_KEY_POSE_ID_DUPLICATED"),
                    QStringLiteral("关键工位 ID 必须唯一。"));
            }
            else
            {
                poseIds.insert(keyPose.pose_id.trimmed());
            }

            if (keyPose.name.trimmed().isEmpty())
            {
                AddIssue(
                    result,
                    prefix + QStringLiteral(".name"),
                    QStringLiteral("REQ_KEY_POSE_NAME_REQUIRED"),
                    QStringLiteral("关键工位名称不能为空。"));
            }

            if (keyPose.position_tol < 0.0)
            {
                AddIssue(
                    result,
                    prefix + QStringLiteral(".position_tol"),
                    QStringLiteral("REQ_KEY_POSE_POSITION_TOL_NEGATIVE"),
                    QStringLiteral("工位位置容限必须大于或等于 0。"));
            }

            if (keyPose.orientation_tol < 0.0)
            {
                AddIssue(
                    result,
                    prefix + QStringLiteral(".orientation_tol"),
                    QStringLiteral("REQ_KEY_POSE_ORIENTATION_TOL_NEGATIVE"),
                    QStringLiteral("工位姿态容限必须大于或等于 0。"));
            }
        }
    }

    if (model.motion_requirements.max_linear_speed <= 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("motion_requirements.max_linear_speed"),
            QStringLiteral("REQ_MAX_LINEAR_SPEED_INVALID"),
            QStringLiteral("最大线速度必须大于 0。"));
    }

    if (model.motion_requirements.max_angular_speed <= 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("motion_requirements.max_angular_speed"),
            QStringLiteral("REQ_MAX_ANGULAR_SPEED_INVALID"),
            QStringLiteral("最大角速度必须大于 0。"));
    }

    if (model.motion_requirements.max_acceleration <= 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("motion_requirements.max_acceleration"),
            QStringLiteral("REQ_MAX_ACCELERATION_INVALID"),
            QStringLiteral("最大线加速度必须大于 0。"));
    }

    if (model.motion_requirements.max_angular_acceleration <= 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("motion_requirements.max_angular_acceleration"),
            QStringLiteral("REQ_MAX_ANGULAR_ACCELERATION_INVALID"),
            QStringLiteral("最大角加速度必须大于 0。"));
    }

    if (model.motion_requirements.jerk_limit < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("motion_requirements.jerk_limit"),
            QStringLiteral("REQ_JERK_LIMIT_NEGATIVE"),
            QStringLiteral("jerk 限制不能为负数。"));
    }

    if (model.motion_requirements.jerk_limit == 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("motion_requirements.jerk_limit"),
            QStringLiteral("REQ_JERK_LIMIT_NOT_SET"),
            QStringLiteral("当前 jerk 限制为 0，后续规划阶段可能需要补充该约束。"),
            ValidationSeverity::Warning);
    }

    if (model.motion_requirements.takt_time <= 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("motion_requirements.takt_time"),
            QStringLiteral("REQ_TAKT_TIME_INVALID"),
            QStringLiteral("节拍时间必须大于 0。"));
    }

    if (model.accuracy_requirements.absolute_accuracy < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("accuracy_requirements.absolute_accuracy"),
            QStringLiteral("REQ_ABSOLUTE_ACCURACY_NEGATIVE"),
            QStringLiteral("绝对定位精度必须大于或等于 0。"));
    }

    if (model.accuracy_requirements.repeatability < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("accuracy_requirements.repeatability"),
            QStringLiteral("REQ_REPEATABILITY_NEGATIVE"),
            QStringLiteral("重复定位精度必须大于或等于 0。"));
    }

    if (model.accuracy_requirements.tracking_accuracy < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("accuracy_requirements.tracking_accuracy"),
            QStringLiteral("REQ_TRACKING_ACCURACY_NEGATIVE"),
            QStringLiteral("跟踪精度必须大于或等于 0。"));
    }

    if (model.accuracy_requirements.orientation_accuracy < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("accuracy_requirements.orientation_accuracy"),
            QStringLiteral("REQ_ORIENTATION_ACCURACY_NEGATIVE"),
            QStringLiteral("姿态精度必须大于或等于 0。"));
    }

    if (model.accuracy_requirements.tcp_position_tol < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("accuracy_requirements.tcp_position_tol"),
            QStringLiteral("REQ_TCP_POSITION_TOL_NEGATIVE"),
            QStringLiteral("TCP 位置容限必须大于或等于 0。"));
    }

    if (model.accuracy_requirements.tcp_orientation_tol < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("accuracy_requirements.tcp_orientation_tol"),
            QStringLiteral("REQ_TCP_ORIENTATION_TOL_NEGATIVE"),
            QStringLiteral("TCP 姿态容限必须大于或等于 0。"));
    }

    if (model.reliability_requirements.design_life < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("reliability_requirements.design_life"),
            QStringLiteral("REQ_DESIGN_LIFE_NEGATIVE"),
            QStringLiteral("设计寿命不能为负数。"));
    }

    if (model.reliability_requirements.cycle_count < 0)
    {
        AddIssue(
            result,
            QStringLiteral("reliability_requirements.cycle_count"),
            QStringLiteral("REQ_CYCLE_COUNT_NEGATIVE"),
            QStringLiteral("循环次数必须大于或等于 0。"));
    }

    if (model.reliability_requirements.duty_cycle < 0.0 ||
        model.reliability_requirements.duty_cycle > 1.0)
    {
        AddIssue(
            result,
            QStringLiteral("reliability_requirements.duty_cycle"),
            QStringLiteral("REQ_DUTY_CYCLE_OUT_OF_RANGE"),
            QStringLiteral("占空比必须在 0 到 1 之间。"));
    }

    if (model.reliability_requirements.operating_hours_per_day < 0.0 ||
        model.reliability_requirements.operating_hours_per_day > 24.0)
    {
        AddIssue(
            result,
            QStringLiteral("reliability_requirements.operating_hours_per_day"),
            QStringLiteral("REQ_OPERATING_HOURS_OUT_OF_RANGE"),
            QStringLiteral("每日运行时长必须在 0 到 24 小时之间。"));
    }

    if (model.reliability_requirements.mtbf_target < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("reliability_requirements.mtbf_target"),
            QStringLiteral("REQ_MTBF_NEGATIVE"),
            QStringLiteral("MTBF 目标不能为负数。"));
    }

    return result;
}

} // namespace RoboSDP::Requirement::Validation
