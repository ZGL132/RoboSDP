#include "modules/requirement/persistence/RequirementJsonStorage.h"

#include <QDir>
#include <QJsonArray>
#include <QJsonObject>

namespace RoboSDP::Requirement::Persistence
{

namespace
{

QJsonArray ToJsonArray3(const std::array<double, 3>& values)
{
    QJsonArray array;
    for (double value : values)
    {
        array.append(value);
    }

    return array;
}

QJsonArray ToJsonArray6(const std::array<double, 6>& values)
{
    QJsonArray array;
    for (double value : values)
    {
        array.append(value);
    }

    return array;
}

std::array<double, 3> ReadArray3(const QJsonArray& array)
{
    std::array<double, 3> values {0.0, 0.0, 0.0};
    for (int index = 0; index < array.size() && index < 3; ++index)
    {
        values[static_cast<std::size_t>(index)] = array.at(index).toDouble();
    }

    return values;
}

std::array<double, 6> ReadArray6(const QJsonArray& array)
{
    std::array<double, 6> values {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (int index = 0; index < array.size() && index < 6; ++index)
    {
        values[static_cast<std::size_t>(index)] = array.at(index).toDouble();
    }

    return values;
}

double ReadDouble(const QJsonObject& object, const QString& key, double defaultValue = 0.0)
{
    return object.contains(key) ? object.value(key).toDouble(defaultValue) : defaultValue;
}

int ReadInt(const QJsonObject& object, const QString& key, int defaultValue = 0)
{
    return object.contains(key) ? object.value(key).toInt(defaultValue) : defaultValue;
}

bool ReadBool(const QJsonObject& object, const QString& key, bool defaultValue = false)
{
    return object.contains(key) ? object.value(key).toBool(defaultValue) : defaultValue;
}

QString ReadString(const QJsonObject& object, const QString& key, const QString& defaultValue = {})
{
    return object.contains(key) ? object.value(key).toString(defaultValue) : defaultValue;
}

} // namespace

RequirementJsonStorage::RequirementJsonStorage(RoboSDP::Repository::IJsonRepository& repository)
    : m_repository(repository)
{
}

RoboSDP::Errors::ErrorCode RequirementJsonStorage::Save(
    const QString& projectRootPath,
    const RoboSDP::Requirement::Dto::RequirementModelDto& model) const
{
    const RoboSDP::Errors::ErrorCode openProjectError = m_repository.OpenProject(projectRootPath);
    if (openProjectError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return openProjectError;
    }

    return m_repository.WriteDocument(RelativeFilePath(), ToJsonObject(model));
}

RoboSDP::Errors::ErrorCode RequirementJsonStorage::Load(
    const QString& projectRootPath,
    RoboSDP::Requirement::Dto::RequirementModelDto& model) const
{
    const RoboSDP::Errors::ErrorCode openProjectError = m_repository.OpenProject(projectRootPath);
    if (openProjectError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return openProjectError;
    }

    QJsonObject jsonObject;
    const RoboSDP::Errors::ErrorCode readError = m_repository.ReadDocument(RelativeFilePath(), jsonObject);
    if (readError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return readError;
    }

    model = FromJsonObject(jsonObject);
    return RoboSDP::Errors::ErrorCode::Ok;
}

QString RequirementJsonStorage::RelativeFilePath() const
{
    return QStringLiteral("requirements/requirement-model.json");
}

QString RequirementJsonStorage::BuildAbsoluteFilePath(const QString& projectRootPath) const
{
    return QDir(projectRootPath).filePath(RelativeFilePath());
}

QJsonObject RequirementJsonStorage::ToJsonObject(
    const RoboSDP::Requirement::Dto::RequirementModelDto& model) const
{
    QJsonObject rootObject;

    QJsonObject projectMetaObject;
    projectMetaObject.insert(QStringLiteral("project_name"), model.project_meta.project_name);
    projectMetaObject.insert(QStringLiteral("scenario_type"), model.project_meta.scenario_type);
    projectMetaObject.insert(QStringLiteral("description"), model.project_meta.description);
    rootObject.insert(QStringLiteral("project_meta"), projectMetaObject);

    QJsonObject loadRequirementsObject;
    loadRequirementsObject.insert(QStringLiteral("rated_payload"), model.load_requirements.rated_payload);
    loadRequirementsObject.insert(QStringLiteral("max_payload"), model.load_requirements.max_payload);
    loadRequirementsObject.insert(QStringLiteral("tool_mass"), model.load_requirements.tool_mass);
    loadRequirementsObject.insert(QStringLiteral("fixture_mass"), model.load_requirements.fixture_mass);
    loadRequirementsObject.insert(QStringLiteral("payload_cog"), ToJsonArray3(model.load_requirements.payload_cog));
    loadRequirementsObject.insert(
        QStringLiteral("payload_inertia"),
        ToJsonArray6(model.load_requirements.payload_inertia));
    loadRequirementsObject.insert(QStringLiteral("off_center_load"), model.load_requirements.off_center_load);
    loadRequirementsObject.insert(QStringLiteral("cable_drag_load"), model.load_requirements.cable_drag_load);
    loadRequirementsObject.insert(QStringLiteral("load_variants"), model.load_requirements.load_variants);
    rootObject.insert(QStringLiteral("load_requirements"), loadRequirementsObject);

    QJsonObject workspaceRequirementsObject;
    workspaceRequirementsObject.insert(QStringLiteral("max_radius"), model.workspace_requirements.max_radius);
    workspaceRequirementsObject.insert(QStringLiteral("min_radius"), model.workspace_requirements.min_radius);
    workspaceRequirementsObject.insert(QStringLiteral("max_height"), model.workspace_requirements.max_height);
    workspaceRequirementsObject.insert(QStringLiteral("min_height"), model.workspace_requirements.min_height);

    QJsonArray keyPosesArray;
    for (const auto& keyPose : model.workspace_requirements.key_poses)
    {
        QJsonObject keyPoseObject;
        keyPoseObject.insert(QStringLiteral("pose_id"), keyPose.pose_id);
        keyPoseObject.insert(QStringLiteral("name"), keyPose.name);
        keyPoseObject.insert(QStringLiteral("pose"), ToJsonArray6(keyPose.pose));
        keyPoseObject.insert(QStringLiteral("position_tol"), keyPose.position_tol);
        keyPoseObject.insert(QStringLiteral("orientation_tol"), keyPose.orientation_tol);
        if (keyPose.has_required_direction)
        {
            keyPoseObject.insert(
                QStringLiteral("required_direction"),
                ToJsonArray3(keyPose.required_direction));
        }

        keyPosesArray.append(keyPoseObject);
    }

    workspaceRequirementsObject.insert(QStringLiteral("key_poses"), keyPosesArray);
    workspaceRequirementsObject.insert(
        QStringLiteral("forbidden_regions"),
        model.workspace_requirements.forbidden_regions);
    workspaceRequirementsObject.insert(
        QStringLiteral("obstacle_regions"),
        model.workspace_requirements.obstacle_regions);
    workspaceRequirementsObject.insert(
        QStringLiteral("base_constraints"),
        model.workspace_requirements.base_constraints);
    rootObject.insert(QStringLiteral("workspace_requirements"), workspaceRequirementsObject);

    QJsonObject motionRequirementsObject;
    motionRequirementsObject.insert(
        QStringLiteral("max_linear_speed"),
        model.motion_requirements.max_linear_speed);
    motionRequirementsObject.insert(
        QStringLiteral("max_angular_speed"),
        model.motion_requirements.max_angular_speed);
    motionRequirementsObject.insert(
        QStringLiteral("max_acceleration"),
        model.motion_requirements.max_acceleration);
    motionRequirementsObject.insert(
        QStringLiteral("max_angular_acceleration"),
        model.motion_requirements.max_angular_acceleration);
    motionRequirementsObject.insert(QStringLiteral("jerk_limit"), model.motion_requirements.jerk_limit);
    motionRequirementsObject.insert(QStringLiteral("takt_time"), model.motion_requirements.takt_time);
    rootObject.insert(QStringLiteral("motion_requirements"), motionRequirementsObject);

    QJsonObject accuracyRequirementsObject;
    accuracyRequirementsObject.insert(
        QStringLiteral("absolute_accuracy"),
        model.accuracy_requirements.absolute_accuracy);
    accuracyRequirementsObject.insert(
        QStringLiteral("repeatability"),
        model.accuracy_requirements.repeatability);
    accuracyRequirementsObject.insert(
        QStringLiteral("tracking_accuracy"),
        model.accuracy_requirements.tracking_accuracy);
    accuracyRequirementsObject.insert(
        QStringLiteral("orientation_accuracy"),
        model.accuracy_requirements.orientation_accuracy);
    accuracyRequirementsObject.insert(
        QStringLiteral("tcp_position_tol"),
        model.accuracy_requirements.tcp_position_tol);
    accuracyRequirementsObject.insert(
        QStringLiteral("tcp_orientation_tol"),
        model.accuracy_requirements.tcp_orientation_tol);
    rootObject.insert(QStringLiteral("accuracy_requirements"), accuracyRequirementsObject);

    QJsonObject reliabilityRequirementsObject;
    reliabilityRequirementsObject.insert(
        QStringLiteral("design_life"),
        model.reliability_requirements.design_life);
    reliabilityRequirementsObject.insert(
        QStringLiteral("cycle_count"),
        model.reliability_requirements.cycle_count);
    reliabilityRequirementsObject.insert(
        QStringLiteral("duty_cycle"),
        model.reliability_requirements.duty_cycle);
    reliabilityRequirementsObject.insert(
        QStringLiteral("operating_hours_per_day"),
        model.reliability_requirements.operating_hours_per_day);
    reliabilityRequirementsObject.insert(
        QStringLiteral("mtbf_target"),
        model.reliability_requirements.mtbf_target);
    rootObject.insert(QStringLiteral("reliability_requirements"), reliabilityRequirementsObject);

    QJsonObject derivedConditionsObject;
    derivedConditionsObject.insert(QStringLiteral("rated_case"), model.derived_conditions.rated_case);
    derivedConditionsObject.insert(QStringLiteral("peak_case"), model.derived_conditions.peak_case);
    rootObject.insert(QStringLiteral("derived_conditions"), derivedConditionsObject);

    return rootObject;
}

RoboSDP::Requirement::Dto::RequirementModelDto RequirementJsonStorage::FromJsonObject(
    const QJsonObject& jsonObject) const
{
    using namespace RoboSDP::Requirement::Dto;

    RequirementModelDto model = RequirementModelDto::CreateDefault();

    const QJsonObject projectMetaObject = jsonObject.value(QStringLiteral("project_meta")).toObject();
    model.project_meta.project_name = ReadString(projectMetaObject, QStringLiteral("project_name"));
    model.project_meta.scenario_type =
        ReadString(projectMetaObject, QStringLiteral("scenario_type"), QStringLiteral("custom"));
    model.project_meta.description = ReadString(projectMetaObject, QStringLiteral("description"));

    const QJsonObject loadRequirementsObject = jsonObject.value(QStringLiteral("load_requirements")).toObject();
    model.load_requirements.rated_payload =
        ReadDouble(loadRequirementsObject, QStringLiteral("rated_payload"));
    model.load_requirements.max_payload =
        ReadDouble(loadRequirementsObject, QStringLiteral("max_payload"));
    model.load_requirements.tool_mass =
        ReadDouble(loadRequirementsObject, QStringLiteral("tool_mass"));
    model.load_requirements.fixture_mass =
        ReadDouble(loadRequirementsObject, QStringLiteral("fixture_mass"));
    model.load_requirements.payload_cog =
        ReadArray3(loadRequirementsObject.value(QStringLiteral("payload_cog")).toArray());
    model.load_requirements.payload_inertia =
        ReadArray6(loadRequirementsObject.value(QStringLiteral("payload_inertia")).toArray());
    model.load_requirements.off_center_load =
        ReadBool(loadRequirementsObject, QStringLiteral("off_center_load"));
    model.load_requirements.cable_drag_load =
        ReadDouble(loadRequirementsObject, QStringLiteral("cable_drag_load"));
    model.load_requirements.load_variants =
        loadRequirementsObject.value(QStringLiteral("load_variants")).toArray();

    const QJsonObject workspaceRequirementsObject =
        jsonObject.value(QStringLiteral("workspace_requirements")).toObject();
    model.workspace_requirements.max_radius =
        ReadDouble(workspaceRequirementsObject, QStringLiteral("max_radius"));
    model.workspace_requirements.min_radius =
        ReadDouble(workspaceRequirementsObject, QStringLiteral("min_radius"));
    model.workspace_requirements.max_height =
        ReadDouble(workspaceRequirementsObject, QStringLiteral("max_height"));
    model.workspace_requirements.min_height =
        ReadDouble(workspaceRequirementsObject, QStringLiteral("min_height"));
    model.workspace_requirements.forbidden_regions =
        workspaceRequirementsObject.value(QStringLiteral("forbidden_regions")).toArray();
    model.workspace_requirements.obstacle_regions =
        workspaceRequirementsObject.value(QStringLiteral("obstacle_regions")).toArray();
    model.workspace_requirements.base_constraints =
        workspaceRequirementsObject.value(QStringLiteral("base_constraints")).toObject();

    model.workspace_requirements.key_poses.clear();
    const QJsonArray keyPosesArray = workspaceRequirementsObject.value(QStringLiteral("key_poses")).toArray();
    for (const QJsonValue& keyPoseValue : keyPosesArray)
    {
        const QJsonObject keyPoseObject = keyPoseValue.toObject();
        RequirementKeyPoseDto keyPose;
        keyPose.pose_id = ReadString(keyPoseObject, QStringLiteral("pose_id"));
        keyPose.name = ReadString(keyPoseObject, QStringLiteral("name"));
        keyPose.pose = ReadArray6(keyPoseObject.value(QStringLiteral("pose")).toArray());
        keyPose.position_tol = ReadDouble(keyPoseObject, QStringLiteral("position_tol"));
        keyPose.orientation_tol = ReadDouble(keyPoseObject, QStringLiteral("orientation_tol"));

        const QJsonArray requiredDirectionArray =
            keyPoseObject.value(QStringLiteral("required_direction")).toArray();
        if (!requiredDirectionArray.isEmpty())
        {
            keyPose.has_required_direction = true;
            keyPose.required_direction = ReadArray3(requiredDirectionArray);
        }

        model.workspace_requirements.key_poses.push_back(keyPose);
    }

    if (model.workspace_requirements.key_poses.empty())
    {
        model.workspace_requirements.key_poses.push_back(RequirementKeyPoseDto {});
    }

    const QJsonObject motionRequirementsObject =
        jsonObject.value(QStringLiteral("motion_requirements")).toObject();
    model.motion_requirements.max_linear_speed =
        ReadDouble(motionRequirementsObject, QStringLiteral("max_linear_speed"));
    model.motion_requirements.max_angular_speed =
        ReadDouble(motionRequirementsObject, QStringLiteral("max_angular_speed"));
    model.motion_requirements.max_acceleration =
        ReadDouble(motionRequirementsObject, QStringLiteral("max_acceleration"));
    model.motion_requirements.max_angular_acceleration =
        ReadDouble(motionRequirementsObject, QStringLiteral("max_angular_acceleration"));
    model.motion_requirements.jerk_limit =
        ReadDouble(motionRequirementsObject, QStringLiteral("jerk_limit"));
    model.motion_requirements.takt_time =
        ReadDouble(motionRequirementsObject, QStringLiteral("takt_time"));

    const QJsonObject accuracyRequirementsObject =
        jsonObject.value(QStringLiteral("accuracy_requirements")).toObject();
    model.accuracy_requirements.absolute_accuracy =
        ReadDouble(accuracyRequirementsObject, QStringLiteral("absolute_accuracy"));
    model.accuracy_requirements.repeatability =
        ReadDouble(accuracyRequirementsObject, QStringLiteral("repeatability"));
    model.accuracy_requirements.tracking_accuracy =
        ReadDouble(accuracyRequirementsObject, QStringLiteral("tracking_accuracy"));
    model.accuracy_requirements.orientation_accuracy =
        ReadDouble(accuracyRequirementsObject, QStringLiteral("orientation_accuracy"));
    model.accuracy_requirements.tcp_position_tol =
        ReadDouble(accuracyRequirementsObject, QStringLiteral("tcp_position_tol"));
    model.accuracy_requirements.tcp_orientation_tol =
        ReadDouble(accuracyRequirementsObject, QStringLiteral("tcp_orientation_tol"));

    const QJsonObject reliabilityRequirementsObject =
        jsonObject.value(QStringLiteral("reliability_requirements")).toObject();
    model.reliability_requirements.design_life =
        ReadDouble(reliabilityRequirementsObject, QStringLiteral("design_life"));
    model.reliability_requirements.cycle_count =
        ReadInt(reliabilityRequirementsObject, QStringLiteral("cycle_count"));
    model.reliability_requirements.duty_cycle =
        ReadDouble(reliabilityRequirementsObject, QStringLiteral("duty_cycle"));
    model.reliability_requirements.operating_hours_per_day =
        ReadDouble(reliabilityRequirementsObject, QStringLiteral("operating_hours_per_day"));
    model.reliability_requirements.mtbf_target =
        ReadDouble(reliabilityRequirementsObject, QStringLiteral("mtbf_target"));

    const QJsonObject derivedConditionsObject =
        jsonObject.value(QStringLiteral("derived_conditions")).toObject();
    model.derived_conditions.rated_case =
        derivedConditionsObject.value(QStringLiteral("rated_case")).toObject();
    model.derived_conditions.peak_case =
        derivedConditionsObject.value(QStringLiteral("peak_case")).toObject();

    return model;
}

} // namespace RoboSDP::Requirement::Persistence
