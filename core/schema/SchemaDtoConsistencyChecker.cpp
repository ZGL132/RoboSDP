#include "core/schema/SchemaDtoConsistencyChecker.h"

#include "core/errors/ErrorCode.h"

#include <QJsonObject>
#include <QStringList>

namespace RoboSDP::Schema
{

namespace
{

struct DocumentCheckSpec
{
    QString relative_path;
    QString field_prefix;
    QStringList required_top_level_keys;
    ValidationSeverity missing_file_severity = ValidationSeverity::Warning;
};

ValidationResult CheckDocument(
    RoboSDP::Repository::IJsonRepository& repository,
    const DocumentCheckSpec& spec)
{
    ValidationResult result;
    QJsonObject document;
    const auto errorCode = repository.ReadDocument(spec.relative_path, document);
    if (errorCode == RoboSDP::Errors::ErrorCode::RepositoryDocumentNotFound)
    {
        // 第一阶段允许部分模块尚未执行到，因此文件缺失先按 warning 处理。
        result.Add(
            spec.field_prefix,
            QStringLiteral("SCHEMA_FILE_MISSING"),
            QStringLiteral("未找到持久化 JSON：%1").arg(spec.relative_path),
            spec.missing_file_severity);
        return result;
    }

    if (errorCode != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.Add(
            spec.field_prefix,
            QStringLiteral("SCHEMA_FILE_READ_FAILED"),
            QStringLiteral("读取 JSON 失败：%1").arg(spec.relative_path),
            ValidationSeverity::Error);
        return result;
    }

    // 统一检查关键顶层键是否存在，作为 DTO 与序列化结构的最小一致性口径。
    for (const auto& key : spec.required_top_level_keys)
    {
        if (!document.contains(key))
        {
            result.Add(
                QStringLiteral("%1.%2").arg(spec.field_prefix, key),
                QStringLiteral("SCHEMA_REQUIRED_KEY_MISSING"),
                QStringLiteral("JSON 缺少关键顶层键：%1").arg(key),
                ValidationSeverity::Error);
        }
    }

    return result;
}

QList<DocumentCheckSpec> BuildSpecs()
{
    return {
        {QStringLiteral("requirements/requirement-model.json"),
         QStringLiteral("requirement_json"),
         {QStringLiteral("project_meta"),
          QStringLiteral("load_requirements"),
          QStringLiteral("workspace_requirements"),
          QStringLiteral("motion_requirements"),
          QStringLiteral("accuracy_requirements"),
          QStringLiteral("reliability_requirements"),
          QStringLiteral("derived_conditions")}},
        {QStringLiteral("topology/topology-model.json"),
         QStringLiteral("topology_json"),
         {QStringLiteral("meta"),
          QStringLiteral("robot_definition"),
          QStringLiteral("layout"),
          QStringLiteral("joints"),
          QStringLiteral("topology_graph"),
          QStringLiteral("candidates"),
          QStringLiteral("recommendation")}},
        {QStringLiteral("kinematics/kinematic-model.json"),
         QStringLiteral("kinematics_model_json"),
         {QStringLiteral("meta"),
          QStringLiteral("parameter_convention"),
          QStringLiteral("joint_count"),
          QStringLiteral("base_frame"),
          QStringLiteral("links"),
          QStringLiteral("joint_limits"),
          QStringLiteral("tcp_frame"),
          QStringLiteral("ik_solver_config"),
          QStringLiteral("last_fk_result"),
          QStringLiteral("last_ik_result")}},
        {QStringLiteral("kinematics/workspace-cache.json"),
         QStringLiteral("kinematics_workspace_json"),
         {QStringLiteral("success"),
          QStringLiteral("message"),
          QStringLiteral("requested_sample_count"),
          QStringLiteral("reachable_sample_count"),
          QStringLiteral("max_radius_m"),
          QStringLiteral("min_position_m"),
          QStringLiteral("max_position_m"),
          QStringLiteral("sampled_points")}},
        {QStringLiteral("dynamics/dynamic-model.json"),
         QStringLiteral("dynamics_model_json"),
         {QStringLiteral("meta"),
          QStringLiteral("gravity"),
          QStringLiteral("installation_pose"),
          QStringLiteral("trajectories")}},
        {QStringLiteral("dynamics/mass-properties.json"),
         QStringLiteral("dynamics_mass_json"),
         {QStringLiteral("links"),
          QStringLiteral("joints"),
          QStringLiteral("end_effector")}},
        {QStringLiteral("dynamics/load-envelopes.json"),
         QStringLiteral("dynamics_envelope_json"),
         {QStringLiteral("parameterized_trajectories"),
          QStringLiteral("trajectory_results"),
          QStringLiteral("peak_stats"),
          QStringLiteral("rms_stats"),
          QStringLiteral("load_envelope")}},
        {QStringLiteral("selection/motor-selection.json"),
         QStringLiteral("selection_motor_json"),
         {QStringLiteral("motor_results")}},
        {QStringLiteral("selection/reducer-selection.json"),
         QStringLiteral("selection_reducer_json"),
         {QStringLiteral("reducer_results")}},
        {QStringLiteral("selection/drivetrain-selection.json"),
         QStringLiteral("selection_drive_json"),
         {QStringLiteral("dynamic_ref"),
          QStringLiteral("kinematic_ref"),
          QStringLiteral("topology_ref"),
          QStringLiteral("requirement_ref"),
          QStringLiteral("joint_selections"),
          QStringLiteral("success"),
          QStringLiteral("message")}},
        {QStringLiteral("planning/planning-scene.json"),
         QStringLiteral("planning_scene_json"),
         {QStringLiteral("meta"),
          QStringLiteral("robot_collision_model_ref"),
          QStringLiteral("joint_limits"),
          QStringLiteral("start_state"),
          QStringLiteral("goal_state"),
          QStringLiteral("environment_objects"),
          QStringLiteral("planning_config")}},
        {QStringLiteral("planning/planning-requests.json"),
         QStringLiteral("planning_request_json"),
         {QStringLiteral("requests")}},
        {QStringLiteral("planning/planning-results.json"),
         QStringLiteral("planning_result_json"),
         {QStringLiteral("results")}},
        {QStringLiteral("snapshots/scheme-snapshot.json"),
         QStringLiteral("scheme_snapshot_json"),
         {QStringLiteral("meta"),
          QStringLiteral("aggregate")}},
        // 当前导出 JSON 没有 Load 接口，因此一致性检查直接读取导出文件顶层键。
        {QStringLiteral("exports/scheme-export.json"),
         QStringLiteral("scheme_export_json"),
         {QStringLiteral("export_id"),
          QStringLiteral("scheme_id"),
          QStringLiteral("export_format"),
          QStringLiteral("generated_at"),
          QStringLiteral("output_file_path"),
          QStringLiteral("success"),
          QStringLiteral("message"),
          QStringLiteral("available_module_count"),
          QStringLiteral("executive_summary"),
          QStringLiteral("module_records")}}};
}

} // namespace

SchemaDtoConsistencyChecker::SchemaDtoConsistencyChecker(RoboSDP::Repository::IJsonRepository& repository)
    : m_repository(repository)
{
}

ValidationResult SchemaDtoConsistencyChecker::CheckProject(const QString& projectRootPath) const
{
    ValidationResult result;
    if (projectRootPath.trimmed().isEmpty())
    {
        result.Add(
            QStringLiteral("project_root"),
            QStringLiteral("SCHEMA_PROJECT_ROOT_REQUIRED"),
            QStringLiteral("项目目录不能为空，无法执行 schema / DTO 一致性检查。"));
        return result;
    }

    const auto openError = m_repository.OpenProject(projectRootPath);
    if (openError != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.Add(
            QStringLiteral("project_root"),
            QStringLiteral("SCHEMA_PROJECT_OPEN_FAILED"),
            QStringLiteral("打开项目目录失败：%1").arg(RoboSDP::Errors::ToChineseMessage(openError)));
        return result;
    }

    for (const auto& spec : BuildSpecs())
    {
        result.Append(CheckDocument(m_repository, spec));
    }
    return result;
}

} // namespace RoboSDP::Schema
