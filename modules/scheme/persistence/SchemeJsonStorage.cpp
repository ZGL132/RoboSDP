#include "modules/scheme/persistence/SchemeJsonStorage.h"

#include <QDir>
#include <QJsonArray>

namespace RoboSDP::Scheme::Persistence
{

namespace
{

QString ReadString(const QJsonObject& object, const QString& key, const QString& defaultValue = {})
{
    return object.contains(key) ? object.value(key).toString(defaultValue) : defaultValue;
}

int ReadInt(const QJsonObject& object, const QString& key, int defaultValue = 0)
{
    return object.contains(key) ? object.value(key).toInt(defaultValue) : defaultValue;
}

double ReadDouble(const QJsonObject& object, const QString& key, double defaultValue = 0.0)
{
    return object.contains(key) ? object.value(key).toDouble(defaultValue) : defaultValue;
}

bool ReadBool(const QJsonObject& object, const QString& key, bool defaultValue = false)
{
    return object.contains(key) ? object.value(key).toBool(defaultValue) : defaultValue;
}

QJsonObject ToRequirementSummaryObject(const RoboSDP::Scheme::Dto::RequirementSummaryDto& summary)
{
    QJsonObject object;
    object.insert(QStringLiteral("available"), summary.available);
    object.insert(QStringLiteral("project_name"), summary.project_name);
    object.insert(QStringLiteral("scenario_type"), summary.scenario_type);
    object.insert(QStringLiteral("rated_payload_kg"), summary.rated_payload_kg);
    object.insert(QStringLiteral("max_payload_kg"), summary.max_payload_kg);
    object.insert(QStringLiteral("key_pose_count"), summary.key_pose_count);
    object.insert(QStringLiteral("takt_time_s"), summary.takt_time_s);
    object.insert(QStringLiteral("summary_text"), summary.summary_text);
    return object;
}

RoboSDP::Scheme::Dto::RequirementSummaryDto FromRequirementSummaryObject(const QJsonObject& object)
{
    RoboSDP::Scheme::Dto::RequirementSummaryDto summary;
    summary.available = ReadBool(object, QStringLiteral("available"));
    summary.project_name = ReadString(object, QStringLiteral("project_name"));
    summary.scenario_type = ReadString(object, QStringLiteral("scenario_type"));
    summary.rated_payload_kg = ReadDouble(object, QStringLiteral("rated_payload_kg"));
    summary.max_payload_kg = ReadDouble(object, QStringLiteral("max_payload_kg"));
    summary.key_pose_count = ReadInt(object, QStringLiteral("key_pose_count"));
    summary.takt_time_s = ReadDouble(object, QStringLiteral("takt_time_s"));
    summary.summary_text = ReadString(object, QStringLiteral("summary_text"));
    return summary;
}

QJsonObject ToTopologySummaryObject(const RoboSDP::Scheme::Dto::TopologySummaryDto& summary)
{
    QJsonObject object;
    object.insert(QStringLiteral("available"), summary.available);
    object.insert(QStringLiteral("topology_id"), summary.topology_id);
    object.insert(QStringLiteral("topology_name"), summary.topology_name);
    object.insert(QStringLiteral("template_id"), summary.template_id);
    object.insert(QStringLiteral("robot_type"), summary.robot_type);
    object.insert(QStringLiteral("joint_count"), summary.joint_count);
    object.insert(QStringLiteral("candidate_count"), summary.candidate_count);
    object.insert(QStringLiteral("recommended_candidate_id"), summary.recommended_candidate_id);
    object.insert(QStringLiteral("summary_text"), summary.summary_text);
    return object;
}

RoboSDP::Scheme::Dto::TopologySummaryDto FromTopologySummaryObject(const QJsonObject& object)
{
    RoboSDP::Scheme::Dto::TopologySummaryDto summary;
    summary.available = ReadBool(object, QStringLiteral("available"));
    summary.topology_id = ReadString(object, QStringLiteral("topology_id"));
    summary.topology_name = ReadString(object, QStringLiteral("topology_name"));
    summary.template_id = ReadString(object, QStringLiteral("template_id"));
    summary.robot_type = ReadString(object, QStringLiteral("robot_type"));
    summary.joint_count = ReadInt(object, QStringLiteral("joint_count"));
    summary.candidate_count = ReadInt(object, QStringLiteral("candidate_count"));
    summary.recommended_candidate_id = ReadString(object, QStringLiteral("recommended_candidate_id"));
    summary.summary_text = ReadString(object, QStringLiteral("summary_text"));
    return summary;
}

QJsonObject ToKinematicsSummaryObject(const RoboSDP::Scheme::Dto::KinematicsSummaryDto& summary)
{
    QJsonObject object;
    object.insert(QStringLiteral("available"), summary.available);
    object.insert(QStringLiteral("kinematic_id"), summary.kinematic_id);
    object.insert(QStringLiteral("parameter_convention"), summary.parameter_convention);
    object.insert(QStringLiteral("joint_count"), summary.joint_count);
    object.insert(QStringLiteral("last_fk_success"), summary.last_fk_success);
    object.insert(QStringLiteral("last_ik_success"), summary.last_ik_success);
    object.insert(
        QStringLiteral("workspace_reachable_sample_count"),
        summary.workspace_reachable_sample_count);
    object.insert(QStringLiteral("workspace_max_radius_m"), summary.workspace_max_radius_m);
    object.insert(QStringLiteral("summary_text"), summary.summary_text);
    return object;
}

RoboSDP::Scheme::Dto::KinematicsSummaryDto FromKinematicsSummaryObject(const QJsonObject& object)
{
    RoboSDP::Scheme::Dto::KinematicsSummaryDto summary;
    summary.available = ReadBool(object, QStringLiteral("available"));
    summary.kinematic_id = ReadString(object, QStringLiteral("kinematic_id"));
    summary.parameter_convention = ReadString(object, QStringLiteral("parameter_convention"));
    summary.joint_count = ReadInt(object, QStringLiteral("joint_count"));
    summary.last_fk_success = ReadBool(object, QStringLiteral("last_fk_success"));
    summary.last_ik_success = ReadBool(object, QStringLiteral("last_ik_success"));
    summary.workspace_reachable_sample_count =
        ReadInt(object, QStringLiteral("workspace_reachable_sample_count"));
    summary.workspace_max_radius_m = ReadDouble(object, QStringLiteral("workspace_max_radius_m"));
    summary.summary_text = ReadString(object, QStringLiteral("summary_text"));
    return summary;
}

QJsonObject ToDynamicsSummaryObject(const RoboSDP::Scheme::Dto::DynamicsSummaryDto& summary)
{
    QJsonObject object;
    object.insert(QStringLiteral("available"), summary.available);
    object.insert(QStringLiteral("dynamic_id"), summary.dynamic_id);
    object.insert(QStringLiteral("trajectory_count"), summary.trajectory_count);
    object.insert(QStringLiteral("peak_stat_count"), summary.peak_stat_count);
    object.insert(QStringLiteral("rms_stat_count"), summary.rms_stat_count);
    object.insert(QStringLiteral("envelope_joint_count"), summary.envelope_joint_count);
    object.insert(QStringLiteral("max_peak_torque_nm"), summary.max_peak_torque_nm);
    object.insert(QStringLiteral("summary_text"), summary.summary_text);
    return object;
}

RoboSDP::Scheme::Dto::DynamicsSummaryDto FromDynamicsSummaryObject(const QJsonObject& object)
{
    RoboSDP::Scheme::Dto::DynamicsSummaryDto summary;
    summary.available = ReadBool(object, QStringLiteral("available"));
    summary.dynamic_id = ReadString(object, QStringLiteral("dynamic_id"));
    summary.trajectory_count = ReadInt(object, QStringLiteral("trajectory_count"));
    summary.peak_stat_count = ReadInt(object, QStringLiteral("peak_stat_count"));
    summary.rms_stat_count = ReadInt(object, QStringLiteral("rms_stat_count"));
    summary.envelope_joint_count = ReadInt(object, QStringLiteral("envelope_joint_count"));
    summary.max_peak_torque_nm = ReadDouble(object, QStringLiteral("max_peak_torque_nm"));
    summary.summary_text = ReadString(object, QStringLiteral("summary_text"));
    return summary;
}

QJsonObject ToSelectionSummaryObject(const RoboSDP::Scheme::Dto::SelectionSummaryDto& summary)
{
    QJsonObject object;
    object.insert(QStringLiteral("available"), summary.available);
    object.insert(QStringLiteral("success"), summary.success);
    object.insert(QStringLiteral("joint_selection_count"), summary.joint_selection_count);
    object.insert(QStringLiteral("recommended_joint_count"), summary.recommended_joint_count);
    object.insert(QStringLiteral("summary_text"), summary.summary_text);
    return object;
}

RoboSDP::Scheme::Dto::SelectionSummaryDto FromSelectionSummaryObject(const QJsonObject& object)
{
    RoboSDP::Scheme::Dto::SelectionSummaryDto summary;
    summary.available = ReadBool(object, QStringLiteral("available"));
    summary.success = ReadBool(object, QStringLiteral("success"));
    summary.joint_selection_count = ReadInt(object, QStringLiteral("joint_selection_count"));
    summary.recommended_joint_count = ReadInt(object, QStringLiteral("recommended_joint_count"));
    summary.summary_text = ReadString(object, QStringLiteral("summary_text"));
    return summary;
}

QJsonObject ToPlanningSummaryObject(const RoboSDP::Scheme::Dto::PlanningSummaryDto& summary)
{
    QJsonObject object;
    object.insert(QStringLiteral("available"), summary.available);
    object.insert(QStringLiteral("success"), summary.success);
    object.insert(QStringLiteral("request_count"), summary.request_count);
    object.insert(QStringLiteral("verification_count"), summary.verification_count);
    object.insert(QStringLiteral("collision_issue_count"), summary.collision_issue_count);
    object.insert(QStringLiteral("self_collision_issue_count"), summary.self_collision_issue_count);
    object.insert(QStringLiteral("cycle_time_within_target"), summary.cycle_time_within_target);
    object.insert(QStringLiteral("summary_text"), summary.summary_text);
    return object;
}

RoboSDP::Scheme::Dto::PlanningSummaryDto FromPlanningSummaryObject(const QJsonObject& object)
{
    RoboSDP::Scheme::Dto::PlanningSummaryDto summary;
    summary.available = ReadBool(object, QStringLiteral("available"));
    summary.success = ReadBool(object, QStringLiteral("success"));
    summary.request_count = ReadInt(object, QStringLiteral("request_count"));
    summary.verification_count = ReadInt(object, QStringLiteral("verification_count"));
    summary.collision_issue_count = ReadInt(object, QStringLiteral("collision_issue_count"));
    summary.self_collision_issue_count = ReadInt(object, QStringLiteral("self_collision_issue_count"));
    summary.cycle_time_within_target = ReadBool(object, QStringLiteral("cycle_time_within_target"));
    summary.summary_text = ReadString(object, QStringLiteral("summary_text"));
    return summary;
}

QJsonObject ToAggregateObject(const RoboSDP::Scheme::Dto::SchemeAggregateDto& aggregate)
{
    QJsonObject refsObject;
    refsObject.insert(QStringLiteral("requirement_ref"), aggregate.refs.requirement_ref);
    refsObject.insert(QStringLiteral("topology_ref"), aggregate.refs.topology_ref);
    refsObject.insert(QStringLiteral("kinematic_ref"), aggregate.refs.kinematic_ref);
    refsObject.insert(QStringLiteral("dynamic_ref"), aggregate.refs.dynamic_ref);
    refsObject.insert(QStringLiteral("selection_ref"), aggregate.refs.selection_ref);
    refsObject.insert(QStringLiteral("planning_scene_ref"), aggregate.refs.planning_scene_ref);
    refsObject.insert(QStringLiteral("planning_result_ref"), aggregate.refs.planning_result_ref);

    QJsonObject exportMetaObject;
    exportMetaObject.insert(
        QStringLiteral("default_export_format"),
        aggregate.export_meta.default_export_format);
    exportMetaObject.insert(
        QStringLiteral("default_export_relative_path"),
        aggregate.export_meta.default_export_relative_path);
    exportMetaObject.insert(
        QStringLiteral("last_export_relative_path"),
        aggregate.export_meta.last_export_relative_path);
    exportMetaObject.insert(QStringLiteral("last_exported_at"), aggregate.export_meta.last_exported_at);

    QJsonObject object;
    object.insert(QStringLiteral("refs"), refsObject);
    object.insert(
        QStringLiteral("requirement_summary"),
        ToRequirementSummaryObject(aggregate.requirement_summary));
    object.insert(
        QStringLiteral("topology_summary"),
        ToTopologySummaryObject(aggregate.topology_summary));
    object.insert(
        QStringLiteral("kinematics_summary"),
        ToKinematicsSummaryObject(aggregate.kinematics_summary));
    object.insert(
        QStringLiteral("dynamics_summary"),
        ToDynamicsSummaryObject(aggregate.dynamics_summary));
    object.insert(
        QStringLiteral("selection_summary"),
        ToSelectionSummaryObject(aggregate.selection_summary));
    object.insert(
        QStringLiteral("planning_summary"),
        ToPlanningSummaryObject(aggregate.planning_summary));
    object.insert(QStringLiteral("export_meta"), exportMetaObject);
    object.insert(QStringLiteral("available_module_count"), aggregate.available_module_count);
    object.insert(QStringLiteral("completeness_summary"), aggregate.completeness_summary);
    return object;
}

RoboSDP::Scheme::Dto::SchemeAggregateDto FromAggregateObject(const QJsonObject& object)
{
    RoboSDP::Scheme::Dto::SchemeAggregateDto aggregate;
    const QJsonObject refsObject = object.value(QStringLiteral("refs")).toObject();
    aggregate.refs.requirement_ref = ReadString(
        refsObject,
        QStringLiteral("requirement_ref"),
        QStringLiteral("requirements/requirement-model.json"));
    aggregate.refs.topology_ref = ReadString(refsObject, QStringLiteral("topology_ref"));
    aggregate.refs.kinematic_ref = ReadString(refsObject, QStringLiteral("kinematic_ref"));
    aggregate.refs.dynamic_ref = ReadString(refsObject, QStringLiteral("dynamic_ref"));
    aggregate.refs.selection_ref = ReadString(
        refsObject,
        QStringLiteral("selection_ref"),
        QStringLiteral("selection/drivetrain-selection.json"));
    aggregate.refs.planning_scene_ref = ReadString(refsObject, QStringLiteral("planning_scene_ref"));
    aggregate.refs.planning_result_ref = ReadString(refsObject, QStringLiteral("planning_result_ref"));

    aggregate.requirement_summary =
        FromRequirementSummaryObject(object.value(QStringLiteral("requirement_summary")).toObject());
    aggregate.topology_summary =
        FromTopologySummaryObject(object.value(QStringLiteral("topology_summary")).toObject());
    aggregate.kinematics_summary =
        FromKinematicsSummaryObject(object.value(QStringLiteral("kinematics_summary")).toObject());
    aggregate.dynamics_summary =
        FromDynamicsSummaryObject(object.value(QStringLiteral("dynamics_summary")).toObject());
    aggregate.selection_summary =
        FromSelectionSummaryObject(object.value(QStringLiteral("selection_summary")).toObject());
    aggregate.planning_summary =
        FromPlanningSummaryObject(object.value(QStringLiteral("planning_summary")).toObject());

    const QJsonObject exportMetaObject = object.value(QStringLiteral("export_meta")).toObject();
    aggregate.export_meta.default_export_format = ReadString(
        exportMetaObject,
        QStringLiteral("default_export_format"),
        QStringLiteral("json"));
    aggregate.export_meta.default_export_relative_path = ReadString(
        exportMetaObject,
        QStringLiteral("default_export_relative_path"),
        QStringLiteral("exports/scheme-export.json"));
    aggregate.export_meta.last_export_relative_path =
        ReadString(exportMetaObject, QStringLiteral("last_export_relative_path"));
    aggregate.export_meta.last_exported_at =
        ReadString(exportMetaObject, QStringLiteral("last_exported_at"));

    aggregate.available_module_count = ReadInt(object, QStringLiteral("available_module_count"));
    aggregate.completeness_summary = ReadString(object, QStringLiteral("completeness_summary"));
    return aggregate;
}

} // namespace

SchemeJsonStorage::SchemeJsonStorage(RoboSDP::Repository::IJsonRepository& repository)
    : m_repository(repository)
{
}

RoboSDP::Errors::ErrorCode SchemeJsonStorage::SaveSnapshot(
    const QString& projectRootPath,
    const RoboSDP::Scheme::Dto::SchemeSnapshotDto& snapshot) const
{
    const RoboSDP::Errors::ErrorCode openError = m_repository.OpenProject(projectRootPath);
    if (openError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return openError;
    }

    return m_repository.WriteDocument(RelativeSnapshotFilePath(), ToSnapshotJsonObject(snapshot));
}

RoboSDP::Errors::ErrorCode SchemeJsonStorage::LoadSnapshot(
    const QString& projectRootPath,
    RoboSDP::Scheme::Dto::SchemeSnapshotDto& snapshot) const
{
    const RoboSDP::Errors::ErrorCode openError = m_repository.OpenProject(projectRootPath);
    if (openError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return openError;
    }

    QJsonObject jsonObject;
    const RoboSDP::Errors::ErrorCode readError =
        m_repository.ReadDocument(RelativeSnapshotFilePath(), jsonObject);
    if (readError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return readError;
    }

    snapshot = FromSnapshotJsonObject(jsonObject);
    return RoboSDP::Errors::ErrorCode::Ok;
}

RoboSDP::Errors::ErrorCode SchemeJsonStorage::SaveExportJson(
    const QString& projectRootPath,
    const RoboSDP::Scheme::Dto::SchemeExportDto& exportDto) const
{
    const RoboSDP::Errors::ErrorCode openError = m_repository.OpenProject(projectRootPath);
    if (openError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return openError;
    }

    return m_repository.WriteDocument(RelativeExportFilePath(), ToExportJsonObject(exportDto));
}

QString SchemeJsonStorage::RelativeSnapshotFilePath() const
{
    return QStringLiteral("snapshots/scheme-snapshot.json");
}

QString SchemeJsonStorage::RelativeExportFilePath() const
{
    return QStringLiteral("exports/scheme-export.json");
}

QString SchemeJsonStorage::BuildAbsoluteSnapshotFilePath(const QString& projectRootPath) const
{
    return QDir(projectRootPath).absoluteFilePath(RelativeSnapshotFilePath());
}

QString SchemeJsonStorage::BuildAbsoluteExportFilePath(const QString& projectRootPath) const
{
    return QDir(projectRootPath).absoluteFilePath(RelativeExportFilePath());
}

QJsonObject SchemeJsonStorage::ToSnapshotJsonObject(
    const RoboSDP::Scheme::Dto::SchemeSnapshotDto& snapshot) const
{
    QJsonObject metaObject;
    metaObject.insert(QStringLiteral("scheme_id"), snapshot.meta.scheme_id);
    metaObject.insert(QStringLiteral("name"), snapshot.meta.name);
    metaObject.insert(QStringLiteral("version"), snapshot.meta.version);
    metaObject.insert(QStringLiteral("status"), snapshot.meta.status);
    metaObject.insert(QStringLiteral("source_project_root"), snapshot.meta.source_project_root);
    metaObject.insert(QStringLiteral("created_at"), snapshot.meta.created_at);
    metaObject.insert(QStringLiteral("updated_at"), snapshot.meta.updated_at);

    QJsonObject rootObject;
    rootObject.insert(QStringLiteral("meta"), metaObject);
    rootObject.insert(QStringLiteral("aggregate"), ToAggregateObject(snapshot.aggregate));
    return rootObject;
}

RoboSDP::Scheme::Dto::SchemeSnapshotDto SchemeJsonStorage::FromSnapshotJsonObject(
    const QJsonObject& jsonObject) const
{
    RoboSDP::Scheme::Dto::SchemeSnapshotDto snapshot =
        RoboSDP::Scheme::Dto::SchemeSnapshotDto::CreateDefault();

    const QJsonObject metaObject = jsonObject.value(QStringLiteral("meta")).toObject();
    snapshot.meta.scheme_id = ReadString(metaObject, QStringLiteral("scheme_id"));
    snapshot.meta.name = ReadString(metaObject, QStringLiteral("name"));
    snapshot.meta.version = ReadInt(metaObject, QStringLiteral("version"), 1);
    snapshot.meta.status = ReadString(metaObject, QStringLiteral("status"), QStringLiteral("draft"));
    snapshot.meta.source_project_root = ReadString(metaObject, QStringLiteral("source_project_root"));
    snapshot.meta.created_at = ReadString(metaObject, QStringLiteral("created_at"));
    snapshot.meta.updated_at = ReadString(metaObject, QStringLiteral("updated_at"));
    snapshot.aggregate = FromAggregateObject(jsonObject.value(QStringLiteral("aggregate")).toObject());
    return snapshot;
}

QJsonObject SchemeJsonStorage::ToExportJsonObject(
    const RoboSDP::Scheme::Dto::SchemeExportDto& exportDto) const
{
    QJsonArray recordArray;
    for (const auto& record : exportDto.module_records)
    {
        QJsonObject recordObject;
        recordObject.insert(QStringLiteral("module_name"), record.module_name);
        recordObject.insert(QStringLiteral("object_ref"), record.object_ref);
        recordObject.insert(QStringLiteral("available"), record.available);
        recordObject.insert(QStringLiteral("summary_text"), record.summary_text);
        recordArray.push_back(recordObject);
    }

    QJsonObject object;
    object.insert(QStringLiteral("export_id"), exportDto.export_id);
    object.insert(QStringLiteral("scheme_id"), exportDto.scheme_id);
    object.insert(QStringLiteral("export_format"), exportDto.export_format);
    object.insert(QStringLiteral("generated_at"), exportDto.generated_at);
    object.insert(QStringLiteral("output_file_path"), exportDto.output_file_path);
    object.insert(QStringLiteral("success"), exportDto.success);
    object.insert(QStringLiteral("message"), exportDto.message);
    object.insert(QStringLiteral("available_module_count"), exportDto.available_module_count);
    object.insert(QStringLiteral("executive_summary"), exportDto.executive_summary);
    object.insert(QStringLiteral("module_records"), recordArray);
    return object;
}

} // namespace RoboSDP::Scheme::Persistence
