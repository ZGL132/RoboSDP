#include "modules/dynamics/persistence/DynamicJsonStorage.h"

#include <QDir>
#include <QJsonArray>
#include <QJsonObject>

namespace RoboSDP::Dynamics::Persistence
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

QJsonArray ToVectorArray(const std::vector<double>& values)
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

std::vector<double> ReadVectorArray(const QJsonArray& array)
{
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(array.size()));
    for (const QJsonValue& value : array)
    {
        values.push_back(value.toDouble());
    }
    return values;
}

QString ReadString(const QJsonObject& object, const QString& key, const QString& defaultValue = {})
{
    return object.contains(key) ? object.value(key).toString(defaultValue) : defaultValue;
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

QJsonObject ToPoseObject(const RoboSDP::Dynamics::Dto::DynamicPoseDto& pose)
{
    QJsonObject object;
    object.insert(QStringLiteral("position_m"), ToJsonArray3(pose.position_m));
    object.insert(QStringLiteral("rpy_deg"), ToJsonArray3(pose.rpy_deg));
    return object;
}

RoboSDP::Dynamics::Dto::DynamicPoseDto FromPoseObject(const QJsonObject& object)
{
    RoboSDP::Dynamics::Dto::DynamicPoseDto pose;
    pose.position_m = ReadArray3(object.value(QStringLiteral("position_m")).toArray());
    pose.rpy_deg = ReadArray3(object.value(QStringLiteral("rpy_deg")).toArray());
    return pose;
}

} // namespace

DynamicJsonStorage::DynamicJsonStorage(RoboSDP::Repository::IJsonRepository& repository)
    : m_repository(repository)
{
}

RoboSDP::Errors::ErrorCode DynamicJsonStorage::Save(
    const QString& projectRootPath,
    const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& state) const
{
    const RoboSDP::Errors::ErrorCode openProjectError = m_repository.OpenProject(projectRootPath);
    if (openProjectError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return openProjectError;
    }

    RoboSDP::Errors::ErrorCode error = m_repository.WriteDocument(
        RelativeModelFilePath(),
        ToModelJsonObject(state.current_model));
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    error = m_repository.WriteDocument(RelativeMassFilePath(), ToMassJsonObject(state.current_model));
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    return m_repository.WriteDocument(RelativeEnvelopeFilePath(), ToEnvelopeJsonObject(state));
}

RoboSDP::Errors::ErrorCode DynamicJsonStorage::Load(
    const QString& projectRootPath,
    RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& state) const
{
    const RoboSDP::Errors::ErrorCode openProjectError = m_repository.OpenProject(projectRootPath);
    if (openProjectError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return openProjectError;
    }

    QJsonObject modelObject;
    RoboSDP::Errors::ErrorCode error = m_repository.ReadDocument(RelativeModelFilePath(), modelObject);
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    state = RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto::CreateDefault();
    FillModelFromJsonObject(modelObject, state.current_model);

    QJsonObject massObject;
    error = m_repository.ReadDocument(RelativeMassFilePath(), massObject);
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }
    FillMassFromJsonObject(massObject, state.current_model);

    QJsonObject envelopeObject;
    error = m_repository.ReadDocument(RelativeEnvelopeFilePath(), envelopeObject);
    if (error == RoboSDP::Errors::ErrorCode::Ok)
    {
        FillEnvelopeFromJsonObject(envelopeObject, state);
    }

    return RoboSDP::Errors::ErrorCode::Ok;
}

QString DynamicJsonStorage::RelativeModelFilePath() const
{
    return QStringLiteral("dynamics/dynamic-model.json");
}

QString DynamicJsonStorage::RelativeMassFilePath() const
{
    return QStringLiteral("dynamics/mass-properties.json");
}

QString DynamicJsonStorage::RelativeEnvelopeFilePath() const
{
    return QStringLiteral("dynamics/load-envelopes.json");
}

QString DynamicJsonStorage::BuildAbsoluteModelFilePath(const QString& projectRootPath) const
{
    return QDir(projectRootPath).filePath(RelativeModelFilePath());
}

QString DynamicJsonStorage::BuildAbsoluteMassFilePath(const QString& projectRootPath) const
{
    return QDir(projectRootPath).filePath(RelativeMassFilePath());
}

QString DynamicJsonStorage::BuildAbsoluteEnvelopeFilePath(const QString& projectRootPath) const
{
    return QDir(projectRootPath).filePath(RelativeEnvelopeFilePath());
}

QJsonObject DynamicJsonStorage::ToModelJsonObject(const RoboSDP::Dynamics::Dto::DynamicModelDto& model) const
{
    QJsonObject rootObject;
    QJsonObject metaObject;
    metaObject.insert(QStringLiteral("dynamic_id"), model.meta.dynamic_id);
    metaObject.insert(QStringLiteral("name"), model.meta.name);
    metaObject.insert(QStringLiteral("version"), model.meta.version);
    metaObject.insert(QStringLiteral("source"), model.meta.source);
    metaObject.insert(QStringLiteral("status"), model.meta.status);
    metaObject.insert(QStringLiteral("kinematic_ref"), model.meta.kinematic_ref);
    metaObject.insert(QStringLiteral("topology_ref"), model.meta.topology_ref);
    metaObject.insert(QStringLiteral("requirement_ref"), model.meta.requirement_ref);
    rootObject.insert(QStringLiteral("meta"), metaObject);
    rootObject.insert(QStringLiteral("gravity"), ToJsonArray3(model.gravity));
    rootObject.insert(QStringLiteral("installation_pose"), ToPoseObject(model.installation_pose));

    QJsonArray trajectoriesArray;
    for (const auto& trajectory : model.trajectories)
    {
        QJsonObject trajectoryObject;
        trajectoryObject.insert(QStringLiteral("trajectory_id"), trajectory.trajectory_id);
        trajectoryObject.insert(QStringLiteral("name"), trajectory.name);
        trajectoryObject.insert(QStringLiteral("profile_type"), trajectory.profile_type);
        trajectoryObject.insert(QStringLiteral("active_joint_id"), trajectory.active_joint_id);
        trajectoryObject.insert(QStringLiteral("joint_start_deg"), ToVectorArray(trajectory.joint_start_deg));
        trajectoryObject.insert(QStringLiteral("joint_target_deg"), ToVectorArray(trajectory.joint_target_deg));
        trajectoryObject.insert(QStringLiteral("duration_s"), trajectory.duration_s);
        trajectoryObject.insert(QStringLiteral("sample_count"), trajectory.sample_count);
        trajectoriesArray.append(trajectoryObject);
    }
    rootObject.insert(QStringLiteral("trajectories"), trajectoriesArray);
    return rootObject;
}

void DynamicJsonStorage::FillModelFromJsonObject(
    const QJsonObject& jsonObject,
    RoboSDP::Dynamics::Dto::DynamicModelDto& model) const
{
    const QJsonObject metaObject = jsonObject.value(QStringLiteral("meta")).toObject();
    model.meta.dynamic_id = ReadString(metaObject, QStringLiteral("dynamic_id"), model.meta.dynamic_id);
    model.meta.name = ReadString(metaObject, QStringLiteral("name"), model.meta.name);
    model.meta.version = ReadInt(metaObject, QStringLiteral("version"), model.meta.version);
    model.meta.source = ReadString(metaObject, QStringLiteral("source"), model.meta.source);
    model.meta.status = ReadString(metaObject, QStringLiteral("status"), model.meta.status);
    model.meta.kinematic_ref = ReadString(metaObject, QStringLiteral("kinematic_ref"));
    model.meta.topology_ref = ReadString(metaObject, QStringLiteral("topology_ref"));
    model.meta.requirement_ref = ReadString(metaObject, QStringLiteral("requirement_ref"));
    model.gravity = ReadArray3(jsonObject.value(QStringLiteral("gravity")).toArray());
    model.installation_pose = FromPoseObject(jsonObject.value(QStringLiteral("installation_pose")).toObject());

    model.trajectories.clear();
    for (const QJsonValue& value : jsonObject.value(QStringLiteral("trajectories")).toArray())
    {
        const QJsonObject trajectoryObject = value.toObject();
        model.trajectories.push_back({
            ReadString(trajectoryObject, QStringLiteral("trajectory_id")),
            ReadString(trajectoryObject, QStringLiteral("name")),
            ReadString(trajectoryObject, QStringLiteral("profile_type")),
            ReadString(trajectoryObject, QStringLiteral("active_joint_id")),
            ReadVectorArray(trajectoryObject.value(QStringLiteral("joint_start_deg")).toArray()),
            ReadVectorArray(trajectoryObject.value(QStringLiteral("joint_target_deg")).toArray()),
            ReadDouble(trajectoryObject, QStringLiteral("duration_s"), 1.0),
            ReadInt(trajectoryObject, QStringLiteral("sample_count"), 64)});
    }
}

QJsonObject DynamicJsonStorage::ToMassJsonObject(const RoboSDP::Dynamics::Dto::DynamicModelDto& model) const
{
    QJsonObject rootObject;
    QJsonArray linksArray;
    for (const auto& link : model.links)
    {
        QJsonObject linkObject;
        linkObject.insert(QStringLiteral("link_id"), link.link_id);
        linkObject.insert(QStringLiteral("mass"), link.mass);
        linkObject.insert(QStringLiteral("cog"), ToJsonArray3(link.cog));
        linkObject.insert(QStringLiteral("inertia_tensor"), ToJsonArray6(link.inertia_tensor));
        linkObject.insert(QStringLiteral("source_detail"), link.source_detail);
        linksArray.append(linkObject);
    }

    QJsonArray jointsArray;
    for (const auto& joint : model.joints)
    {
        QJsonObject jointObject;
        jointObject.insert(QStringLiteral("joint_id"), joint.joint_id);
        jointObject.insert(QStringLiteral("transmission_ratio"), joint.transmission_ratio);
        jointObject.insert(QStringLiteral("efficiency"), joint.efficiency);
        QJsonObject frictionObject;
        frictionObject.insert(QStringLiteral("viscous"), joint.friction.viscous);
        frictionObject.insert(QStringLiteral("coulomb"), joint.friction.coulomb);
        frictionObject.insert(QStringLiteral("static"), joint.friction.statik);
        jointObject.insert(QStringLiteral("friction"), frictionObject);
        jointsArray.append(jointObject);
    }

    QJsonObject endEffectorObject;
    endEffectorObject.insert(QStringLiteral("mass"), model.end_effector.mass);
    endEffectorObject.insert(QStringLiteral("cog"), ToJsonArray3(model.end_effector.cog));
    endEffectorObject.insert(QStringLiteral("inertia_tensor"), ToJsonArray6(model.end_effector.inertia_tensor));

    rootObject.insert(QStringLiteral("links"), linksArray);
    rootObject.insert(QStringLiteral("joints"), jointsArray);
    rootObject.insert(QStringLiteral("end_effector"), endEffectorObject);
    return rootObject;
}

void DynamicJsonStorage::FillMassFromJsonObject(
    const QJsonObject& jsonObject,
    RoboSDP::Dynamics::Dto::DynamicModelDto& model) const
{
    model.links.clear();
    for (const QJsonValue& value : jsonObject.value(QStringLiteral("links")).toArray())
    {
        const QJsonObject linkObject = value.toObject();
        model.links.push_back({
            ReadString(linkObject, QStringLiteral("link_id")),
            ReadDouble(linkObject, QStringLiteral("mass")),
            ReadArray3(linkObject.value(QStringLiteral("cog")).toArray()),
            ReadArray6(linkObject.value(QStringLiteral("inertia_tensor")).toArray()),
            ReadString(linkObject, QStringLiteral("source_detail"))});
    }

    model.joints.clear();
    for (const QJsonValue& value : jsonObject.value(QStringLiteral("joints")).toArray())
    {
        const QJsonObject jointObject = value.toObject();
        const QJsonObject frictionObject = jointObject.value(QStringLiteral("friction")).toObject();
        model.joints.push_back({
            ReadString(jointObject, QStringLiteral("joint_id")),
            ReadDouble(jointObject, QStringLiteral("transmission_ratio"), 100.0),
            ReadDouble(jointObject, QStringLiteral("efficiency"), 0.92),
            {ReadDouble(frictionObject, QStringLiteral("viscous")),
             ReadDouble(frictionObject, QStringLiteral("coulomb")),
             ReadDouble(frictionObject, QStringLiteral("static"))}});
    }

    const QJsonObject endEffectorObject = jsonObject.value(QStringLiteral("end_effector")).toObject();
    model.end_effector.mass = ReadDouble(endEffectorObject, QStringLiteral("mass"));
    model.end_effector.cog = ReadArray3(endEffectorObject.value(QStringLiteral("cog")).toArray());
    model.end_effector.inertia_tensor =
        ReadArray6(endEffectorObject.value(QStringLiteral("inertia_tensor")).toArray());
}

QJsonObject DynamicJsonStorage::ToEnvelopeJsonObject(
    const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& state) const
{
    QJsonObject rootObject;

    QJsonArray parameterizedArray;
    for (const auto& trajectory : state.parameterized_trajectories)
    {
        QJsonObject trajectoryObject;
        trajectoryObject.insert(QStringLiteral("trajectory_id"), trajectory.trajectory_id);
        trajectoryObject.insert(QStringLiteral("name"), trajectory.name);
        trajectoryObject.insert(QStringLiteral("duration_s"), trajectory.duration_s);

        QJsonArray sampleArray;
        for (const auto& sample : trajectory.samples)
        {
            QJsonObject sampleObject;
            sampleObject.insert(QStringLiteral("time_s"), sample.time_s);
            sampleObject.insert(QStringLiteral("positions_deg"), ToVectorArray(sample.positions_deg));
            sampleObject.insert(QStringLiteral("velocities_deg_s"), ToVectorArray(sample.velocities_deg_s));
            sampleObject.insert(QStringLiteral("accelerations_deg_s2"), ToVectorArray(sample.accelerations_deg_s2));
            sampleArray.append(sampleObject);
        }
        trajectoryObject.insert(QStringLiteral("samples"), sampleArray);
        parameterizedArray.append(trajectoryObject);
    }
    rootObject.insert(QStringLiteral("parameterized_trajectories"), parameterizedArray);

    QJsonArray resultArray;
    for (const auto& trajectoryResult : state.trajectory_results)
    {
        QJsonObject resultObject;
        resultObject.insert(QStringLiteral("trajectory_id"), trajectoryResult.trajectory_id);
        resultObject.insert(QStringLiteral("name"), trajectoryResult.name);
        resultObject.insert(QStringLiteral("success"), trajectoryResult.success);
        resultObject.insert(QStringLiteral("solver_backend"), trajectoryResult.solver_backend);
        resultObject.insert(QStringLiteral("used_fallback"), trajectoryResult.used_fallback);
        resultObject.insert(QStringLiteral("message"), trajectoryResult.message);

        QJsonArray curveArray;
        for (const auto& curve : trajectoryResult.joint_curves)
        {
            QJsonObject curveObject;
            curveObject.insert(QStringLiteral("joint_id"), curve.joint_id);
            QJsonArray sampleArray;
            for (const auto& sample : curve.samples)
            {
                QJsonObject sampleObject;
                sampleObject.insert(QStringLiteral("time_s"), sample.time_s);
                sampleObject.insert(QStringLiteral("torque_nm"), sample.torque_nm);
                sampleObject.insert(QStringLiteral("speed_deg_s"), sample.speed_deg_s);
                sampleObject.insert(QStringLiteral("power_w"), sample.power_w);
                sampleArray.append(sampleObject);
            }
            curveObject.insert(QStringLiteral("samples"), sampleArray);
            curveArray.append(curveObject);
        }
        resultObject.insert(QStringLiteral("joint_curves"), curveArray);
        resultArray.append(resultObject);
    }
    rootObject.insert(QStringLiteral("trajectory_results"), resultArray);

    QJsonArray peakArray;
    for (const auto& peak : state.peak_stats)
    {
        QJsonObject peakObject;
        peakObject.insert(QStringLiteral("joint_id"), peak.joint_id);
        peakObject.insert(QStringLiteral("trajectory_id"), peak.trajectory_id);
        peakObject.insert(QStringLiteral("peak_torque_nm"), peak.peak_torque_nm);
        peakObject.insert(QStringLiteral("peak_speed_deg_s"), peak.peak_speed_deg_s);
        peakObject.insert(QStringLiteral("peak_power_w"), peak.peak_power_w);
        peakArray.append(peakObject);
    }
    rootObject.insert(QStringLiteral("peak_stats"), peakArray);

    QJsonArray rmsArray;
    for (const auto& rms : state.rms_stats)
    {
        QJsonObject rmsObject;
        rmsObject.insert(QStringLiteral("joint_id"), rms.joint_id);
        rmsObject.insert(QStringLiteral("trajectory_id"), rms.trajectory_id);
        rmsObject.insert(QStringLiteral("rms_torque_nm"), rms.rms_torque_nm);
        rmsArray.append(rmsObject);
    }
    rootObject.insert(QStringLiteral("rms_stats"), rmsArray);

    QJsonArray envelopeArray;
    for (const auto& joint : state.load_envelope.joints)
    {
        QJsonObject jointObject;
        jointObject.insert(QStringLiteral("joint_id"), joint.joint_id);
        jointObject.insert(QStringLiteral("source_trajectory_id"), joint.source_trajectory_id);
        jointObject.insert(QStringLiteral("peak_torque_nm"), joint.peak_torque_nm);
        jointObject.insert(QStringLiteral("rms_torque_nm"), joint.rms_torque_nm);
        jointObject.insert(QStringLiteral("peak_power_w"), joint.peak_power_w);
        envelopeArray.append(jointObject);
    }
    rootObject.insert(QStringLiteral("load_envelope"), envelopeArray);
    return rootObject;
}

void DynamicJsonStorage::FillEnvelopeFromJsonObject(
    const QJsonObject& jsonObject,
    RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& state) const
{
    state.parameterized_trajectories.clear();
    for (const QJsonValue& value : jsonObject.value(QStringLiteral("parameterized_trajectories")).toArray())
    {
        const QJsonObject trajectoryObject = value.toObject();
        RoboSDP::Dynamics::Dto::ParameterizedTrajectoryDto parameterized;
        parameterized.trajectory_id = ReadString(trajectoryObject, QStringLiteral("trajectory_id"));
        parameterized.name = ReadString(trajectoryObject, QStringLiteral("name"));
        parameterized.duration_s = ReadDouble(trajectoryObject, QStringLiteral("duration_s"));

        for (const QJsonValue& sampleValue : trajectoryObject.value(QStringLiteral("samples")).toArray())
        {
            const QJsonObject sampleObject = sampleValue.toObject();
            parameterized.samples.push_back({
                ReadDouble(sampleObject, QStringLiteral("time_s")),
                ReadVectorArray(sampleObject.value(QStringLiteral("positions_deg")).toArray()),
                ReadVectorArray(sampleObject.value(QStringLiteral("velocities_deg_s")).toArray()),
                ReadVectorArray(sampleObject.value(QStringLiteral("accelerations_deg_s2")).toArray())});
        }
        state.parameterized_trajectories.push_back(parameterized);
    }

    state.trajectory_results.clear();
    for (const QJsonValue& value : jsonObject.value(QStringLiteral("trajectory_results")).toArray())
    {
        const QJsonObject resultObject = value.toObject();
        RoboSDP::Dynamics::Dto::InverseDynamicsTrajectoryResultDto trajectoryResult;
        trajectoryResult.trajectory_id = ReadString(resultObject, QStringLiteral("trajectory_id"));
        trajectoryResult.name = ReadString(resultObject, QStringLiteral("name"));
        trajectoryResult.success = ReadBool(resultObject, QStringLiteral("success"));
        trajectoryResult.solver_backend =
            ReadString(resultObject, QStringLiteral("solver_backend"), trajectoryResult.solver_backend);
        trajectoryResult.used_fallback =
            ReadBool(resultObject, QStringLiteral("used_fallback"), trajectoryResult.used_fallback);
        trajectoryResult.message = ReadString(resultObject, QStringLiteral("message"));

        for (const QJsonValue& curveValue : resultObject.value(QStringLiteral("joint_curves")).toArray())
        {
            const QJsonObject curveObject = curveValue.toObject();
            RoboSDP::Dynamics::Dto::JointLoadCurveDto curve;
            curve.joint_id = ReadString(curveObject, QStringLiteral("joint_id"));
            for (const QJsonValue& sampleValue : curveObject.value(QStringLiteral("samples")).toArray())
            {
                const QJsonObject sampleObject = sampleValue.toObject();
                curve.samples.push_back({
                    ReadDouble(sampleObject, QStringLiteral("time_s")),
                    ReadDouble(sampleObject, QStringLiteral("torque_nm")),
                    ReadDouble(sampleObject, QStringLiteral("speed_deg_s")),
                    ReadDouble(sampleObject, QStringLiteral("power_w"))});
            }
            trajectoryResult.joint_curves.push_back(curve);
        }

        state.trajectory_results.push_back(trajectoryResult);
    }

    state.peak_stats.clear();
    for (const QJsonValue& value : jsonObject.value(QStringLiteral("peak_stats")).toArray())
    {
        const QJsonObject peakObject = value.toObject();
        state.peak_stats.push_back({
            ReadString(peakObject, QStringLiteral("joint_id")),
            ReadString(peakObject, QStringLiteral("trajectory_id")),
            ReadDouble(peakObject, QStringLiteral("peak_torque_nm")),
            ReadDouble(peakObject, QStringLiteral("peak_speed_deg_s")),
            ReadDouble(peakObject, QStringLiteral("peak_power_w"))});
    }

    state.rms_stats.clear();
    for (const QJsonValue& value : jsonObject.value(QStringLiteral("rms_stats")).toArray())
    {
        const QJsonObject rmsObject = value.toObject();
        state.rms_stats.push_back({
            ReadString(rmsObject, QStringLiteral("joint_id")),
            ReadString(rmsObject, QStringLiteral("trajectory_id")),
            ReadDouble(rmsObject, QStringLiteral("rms_torque_nm"))});
    }

    state.load_envelope.joints.clear();
    for (const QJsonValue& value : jsonObject.value(QStringLiteral("load_envelope")).toArray())
    {
        const QJsonObject jointObject = value.toObject();
        state.load_envelope.joints.push_back({
            ReadString(jointObject, QStringLiteral("joint_id")),
            ReadString(jointObject, QStringLiteral("source_trajectory_id")),
            ReadDouble(jointObject, QStringLiteral("peak_torque_nm")),
            ReadDouble(jointObject, QStringLiteral("rms_torque_nm")),
            ReadDouble(jointObject, QStringLiteral("peak_power_w"))});
    }
}

} // namespace RoboSDP::Dynamics::Persistence
