#include "modules/planning/persistence/PlanningJsonStorage.h"

#include <QDir>
#include <QJsonArray>

namespace RoboSDP::Planning::Persistence
{

namespace
{

QJsonArray ToStringArray(const std::vector<QString>& values)
{
    QJsonArray array;
    for (const auto& value : values)
    {
        array.push_back(value);
    }
    return array;
}

std::vector<QString> FromStringArray(const QJsonArray& array)
{
    std::vector<QString> values;
    values.reserve(static_cast<std::size_t>(array.size()));
    for (const auto& value : array)
    {
        values.push_back(value.toString());
    }
    return values;
}

QJsonArray ToDoubleArray(const std::vector<double>& values)
{
    QJsonArray array;
    for (double value : values)
    {
        array.push_back(value);
    }
    return array;
}

std::vector<double> FromDoubleArray(const QJsonArray& array)
{
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(array.size()));
    for (const auto& value : array)
    {
        values.push_back(value.toDouble());
    }
    return values;
}

QJsonArray ToArray3(const std::array<double, 3>& values)
{
    return QJsonArray {values[0], values[1], values[2]};
}

std::array<double, 3> FromArray3(const QJsonArray& array)
{
    std::array<double, 3> values {0.0, 0.0, 0.0};
    for (int index = 0; index < 3 && index < array.size(); ++index)
    {
        values[static_cast<std::size_t>(index)] = array.at(index).toDouble();
    }
    return values;
}

QJsonObject ToJointStateObject(const RoboSDP::Planning::Dto::PlanningJointStateDto& state)
{
    QJsonObject object;
    object.insert(QStringLiteral("joint_ids"), ToStringArray(state.joint_ids));
    object.insert(QStringLiteral("joint_values_deg"), ToDoubleArray(state.joint_values_deg));
    return object;
}

RoboSDP::Planning::Dto::PlanningJointStateDto FromJointStateObject(const QJsonObject& object)
{
    RoboSDP::Planning::Dto::PlanningJointStateDto state;
    state.joint_ids = FromStringArray(object.value(QStringLiteral("joint_ids")).toArray());
    state.joint_values_deg = FromDoubleArray(object.value(QStringLiteral("joint_values_deg")).toArray());
    return state;
}

QJsonObject ToTrajectoryResultObject(const RoboSDP::Planning::Dto::TrajectoryResultDto& result)
{
    QJsonObject object;
    object.insert(QStringLiteral("request_id"), result.request_id);
    object.insert(QStringLiteral("planner_id"), result.planner_id);
    object.insert(QStringLiteral("success"), result.success);
    object.insert(QStringLiteral("message"), result.message);
    object.insert(QStringLiteral("planning_time_s"), result.planning_time_s);
    object.insert(QStringLiteral("trajectory_duration_s"), result.trajectory_duration_s);
    object.insert(QStringLiteral("waypoint_count"), result.waypoint_count);

    QJsonArray waypointArray;
    for (const auto& waypoint : result.waypoints)
    {
        QJsonObject waypointObject;
        waypointObject.insert(QStringLiteral("time_from_start_s"), waypoint.time_from_start_s);
        waypointObject.insert(QStringLiteral("joint_values_deg"), ToDoubleArray(waypoint.joint_values_deg));
        waypointArray.push_back(waypointObject);
    }
    object.insert(QStringLiteral("waypoints"), waypointArray);
    return object;
}

RoboSDP::Planning::Dto::TrajectoryResultDto FromTrajectoryResultObject(const QJsonObject& object)
{
    RoboSDP::Planning::Dto::TrajectoryResultDto result;
    result.request_id = object.value(QStringLiteral("request_id")).toString();
    result.planner_id = object.value(QStringLiteral("planner_id")).toString();
    result.success = object.value(QStringLiteral("success")).toBool();
    result.message = object.value(QStringLiteral("message")).toString();
    result.planning_time_s = object.value(QStringLiteral("planning_time_s")).toDouble();
    result.trajectory_duration_s = object.value(QStringLiteral("trajectory_duration_s")).toDouble();
    result.waypoint_count = object.value(QStringLiteral("waypoint_count")).toInt();

    const QJsonArray waypointArray = object.value(QStringLiteral("waypoints")).toArray();
    result.waypoints.reserve(static_cast<std::size_t>(waypointArray.size()));
    for (const auto& value : waypointArray)
    {
        const QJsonObject waypointObject = value.toObject();
        result.waypoints.push_back({
            waypointObject.value(QStringLiteral("time_from_start_s")).toDouble(),
            FromDoubleArray(waypointObject.value(QStringLiteral("joint_values_deg")).toArray())});
    }
    return result;
}

QJsonObject ToCollisionResultObject(const RoboSDP::Planning::Dto::CollisionResultDto& result)
{
    QJsonObject object;
    object.insert(QStringLiteral("request_id"), result.request_id);
    object.insert(QStringLiteral("object_id"), result.object_id);
    object.insert(QStringLiteral("in_collision"), result.in_collision);
    object.insert(QStringLiteral("message"), result.message);
    return object;
}

RoboSDP::Planning::Dto::CollisionResultDto FromCollisionResultObject(const QJsonObject& object)
{
    RoboSDP::Planning::Dto::CollisionResultDto result;
    result.request_id = object.value(QStringLiteral("request_id")).toString();
    result.object_id = object.value(QStringLiteral("object_id")).toString();
    result.in_collision = object.value(QStringLiteral("in_collision")).toBool();
    result.message = object.value(QStringLiteral("message")).toString();
    return result;
}

QJsonObject ToSelfCollisionResultObject(const RoboSDP::Planning::Dto::SelfCollisionResultDto& result)
{
    QJsonObject object;
    object.insert(QStringLiteral("request_id"), result.request_id);
    object.insert(QStringLiteral("link_a"), result.link_a);
    object.insert(QStringLiteral("link_b"), result.link_b);
    object.insert(QStringLiteral("in_self_collision"), result.in_self_collision);
    object.insert(QStringLiteral("message"), result.message);
    return object;
}

RoboSDP::Planning::Dto::SelfCollisionResultDto FromSelfCollisionResultObject(const QJsonObject& object)
{
    RoboSDP::Planning::Dto::SelfCollisionResultDto result;
    result.request_id = object.value(QStringLiteral("request_id")).toString();
    result.link_a = object.value(QStringLiteral("link_a")).toString();
    result.link_b = object.value(QStringLiteral("link_b")).toString();
    result.in_self_collision = object.value(QStringLiteral("in_self_collision")).toBool();
    result.message = object.value(QStringLiteral("message")).toString();
    return result;
}

QJsonObject ToCycleTimeResultObject(const RoboSDP::Planning::Dto::CycleTimeResultDto& result)
{
    QJsonObject object;
    object.insert(QStringLiteral("request_id"), result.request_id);
    object.insert(QStringLiteral("trajectory_duration_s"), result.trajectory_duration_s);
    object.insert(QStringLiteral("target_cycle_time_s"), result.target_cycle_time_s);
    object.insert(QStringLiteral("within_target"), result.within_target);
    object.insert(QStringLiteral("margin_s"), result.margin_s);
    object.insert(QStringLiteral("message"), result.message);
    return object;
}

RoboSDP::Planning::Dto::CycleTimeResultDto FromCycleTimeResultObject(const QJsonObject& object)
{
    RoboSDP::Planning::Dto::CycleTimeResultDto result;
    result.request_id = object.value(QStringLiteral("request_id")).toString();
    result.trajectory_duration_s = object.value(QStringLiteral("trajectory_duration_s")).toDouble();
    result.target_cycle_time_s = object.value(QStringLiteral("target_cycle_time_s")).toDouble();
    result.within_target = object.value(QStringLiteral("within_target")).toBool();
    result.margin_s = object.value(QStringLiteral("margin_s")).toDouble();
    result.message = object.value(QStringLiteral("message")).toString();
    return result;
}

} // namespace

PlanningJsonStorage::PlanningJsonStorage(RoboSDP::Repository::IJsonRepository& repository)
    : m_repository(repository)
{
}

RoboSDP::Errors::ErrorCode PlanningJsonStorage::Save(
    const QString& projectRootPath,
    const RoboSDP::Planning::Dto::PlanningWorkspaceStateDto& state) const
{
    RoboSDP::Errors::ErrorCode error = m_repository.OpenProject(projectRootPath);
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    error = m_repository.WriteDocument(RelativeSceneFilePath(), ToSceneJsonObject(state.current_scene));
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    error = m_repository.WriteDocument(RelativeRequestFilePath(), ToRequestsJsonObject(state.requests));
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    return m_repository.WriteDocument(RelativeResultFilePath(), ToResultsJsonObject(state.results));
}

RoboSDP::Errors::ErrorCode PlanningJsonStorage::Load(
    const QString& projectRootPath,
    RoboSDP::Planning::Dto::PlanningWorkspaceStateDto& state) const
{
    RoboSDP::Errors::ErrorCode error = m_repository.OpenProject(projectRootPath);
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    QJsonObject sceneObject;
    error = m_repository.ReadDocument(RelativeSceneFilePath(), sceneObject);
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    QJsonObject requestObject;
    error = m_repository.ReadDocument(RelativeRequestFilePath(), requestObject);
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    QJsonObject resultObject;
    error = m_repository.ReadDocument(RelativeResultFilePath(), resultObject);
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    state.current_scene = FromSceneJsonObject(sceneObject);
    state.requests = FromRequestsJsonObject(requestObject);
    state.results = FromResultsJsonObject(resultObject);
    return RoboSDP::Errors::ErrorCode::Ok;
}

QString PlanningJsonStorage::RelativeSceneFilePath() const
{
    return QStringLiteral("planning/planning-scene.json");
}

QString PlanningJsonStorage::RelativeRequestFilePath() const
{
    return QStringLiteral("planning/planning-requests.json");
}

QString PlanningJsonStorage::RelativeResultFilePath() const
{
    return QStringLiteral("planning/planning-results.json");
}

QString PlanningJsonStorage::BuildAbsoluteSceneFilePath(const QString& projectRootPath) const
{
    return QDir(projectRootPath).absoluteFilePath(RelativeSceneFilePath());
}

QString PlanningJsonStorage::BuildAbsoluteRequestFilePath(const QString& projectRootPath) const
{
    return QDir(projectRootPath).absoluteFilePath(RelativeRequestFilePath());
}

QString PlanningJsonStorage::BuildAbsoluteResultFilePath(const QString& projectRootPath) const
{
    return QDir(projectRootPath).absoluteFilePath(RelativeResultFilePath());
}

QJsonObject PlanningJsonStorage::ToSceneJsonObject(const RoboSDP::Planning::Dto::PlanningSceneDto& scene) const
{
    QJsonObject metaObject;
    metaObject.insert(QStringLiteral("planning_scene_id"), scene.meta.planning_scene_id);
    metaObject.insert(QStringLiteral("name"), scene.meta.name);
    metaObject.insert(QStringLiteral("version"), scene.meta.version);
    metaObject.insert(QStringLiteral("source"), scene.meta.source);
    metaObject.insert(QStringLiteral("status"), scene.meta.status);
    metaObject.insert(QStringLiteral("kinematic_ref"), scene.meta.kinematic_ref);
    metaObject.insert(QStringLiteral("dynamic_ref"), scene.meta.dynamic_ref);
    metaObject.insert(QStringLiteral("selection_ref"), scene.meta.selection_ref);
    metaObject.insert(QStringLiteral("requirement_ref"), scene.meta.requirement_ref);

    QJsonArray limitArray;
    for (const auto& limit : scene.joint_limits)
    {
        QJsonObject limitObject;
        limitObject.insert(QStringLiteral("joint_id"), limit.joint_id);
        limitObject.insert(QStringLiteral("lower_deg"), limit.lower_deg);
        limitObject.insert(QStringLiteral("upper_deg"), limit.upper_deg);
        limitArray.push_back(limitObject);
    }

    QJsonArray objectArray;
    for (const auto& object : scene.environment_objects)
    {
        QJsonObject objectJson;
        objectJson.insert(QStringLiteral("object_id"), object.object_id);
        objectJson.insert(QStringLiteral("name"), object.name);
        objectJson.insert(QStringLiteral("shape_type"), object.shape_type);
        objectJson.insert(QStringLiteral("size_m"), ToArray3(object.size_m));
        objectJson.insert(QStringLiteral("position_m"), ToArray3(object.position_m));
        objectJson.insert(QStringLiteral("rpy_deg"), ToArray3(object.rpy_deg));
        objectJson.insert(QStringLiteral("enabled"), object.enabled);
        objectArray.push_back(objectJson);
    }

    QJsonObject configObject;
    configObject.insert(QStringLiteral("motion_group"), scene.planning_config.motion_group);
    configObject.insert(QStringLiteral("planner_id"), scene.planning_config.planner_id);
    configObject.insert(QStringLiteral("service_endpoint"), scene.planning_config.service_endpoint);
    configObject.insert(QStringLiteral("allowed_planning_time_s"), scene.planning_config.allowed_planning_time_s);
    configObject.insert(QStringLiteral("max_planning_attempts"), scene.planning_config.max_planning_attempts);
    configObject.insert(QStringLiteral("target_cycle_time_s"), scene.planning_config.target_cycle_time_s);
    configObject.insert(QStringLiteral("check_collision"), scene.planning_config.check_collision);
    configObject.insert(QStringLiteral("check_self_collision"), scene.planning_config.check_self_collision);

    QJsonObject sceneObject;
    sceneObject.insert(QStringLiteral("meta"), metaObject);
    sceneObject.insert(QStringLiteral("robot_collision_model_ref"), scene.robot_collision_model_ref);
    sceneObject.insert(QStringLiteral("joint_limits"), limitArray);
    sceneObject.insert(QStringLiteral("start_state"), ToJointStateObject(scene.start_state));
    sceneObject.insert(QStringLiteral("goal_state"), ToJointStateObject(scene.goal_state));
    sceneObject.insert(QStringLiteral("environment_objects"), objectArray);
    sceneObject.insert(QStringLiteral("planning_config"), configObject);
    return sceneObject;
}

RoboSDP::Planning::Dto::PlanningSceneDto PlanningJsonStorage::FromSceneJsonObject(const QJsonObject& jsonObject) const
{
    RoboSDP::Planning::Dto::PlanningSceneDto scene = RoboSDP::Planning::Dto::PlanningSceneDto::CreateDefault();
    const QJsonObject metaObject = jsonObject.value(QStringLiteral("meta")).toObject();
    scene.meta.planning_scene_id = metaObject.value(QStringLiteral("planning_scene_id")).toString();
    scene.meta.name = metaObject.value(QStringLiteral("name")).toString();
    scene.meta.version = metaObject.value(QStringLiteral("version")).toInt(1);
    scene.meta.source = metaObject.value(QStringLiteral("source")).toString();
    scene.meta.status = metaObject.value(QStringLiteral("status")).toString();
    scene.meta.kinematic_ref = metaObject.value(QStringLiteral("kinematic_ref")).toString();
    scene.meta.dynamic_ref = metaObject.value(QStringLiteral("dynamic_ref")).toString();
    scene.meta.selection_ref = metaObject.value(QStringLiteral("selection_ref")).toString();
    scene.meta.requirement_ref = metaObject.value(QStringLiteral("requirement_ref")).toString();
    scene.robot_collision_model_ref = jsonObject.value(QStringLiteral("robot_collision_model_ref")).toString();

    scene.joint_limits.clear();
    const QJsonArray limitArray = jsonObject.value(QStringLiteral("joint_limits")).toArray();
    for (const auto& value : limitArray)
    {
        const QJsonObject limitObject = value.toObject();
        scene.joint_limits.push_back({
            limitObject.value(QStringLiteral("joint_id")).toString(),
            limitObject.value(QStringLiteral("lower_deg")).toDouble(),
            limitObject.value(QStringLiteral("upper_deg")).toDouble()});
    }

    scene.start_state = FromJointStateObject(jsonObject.value(QStringLiteral("start_state")).toObject());
    scene.goal_state = FromJointStateObject(jsonObject.value(QStringLiteral("goal_state")).toObject());

    scene.environment_objects.clear();
    const QJsonArray objectArray = jsonObject.value(QStringLiteral("environment_objects")).toArray();
    for (const auto& value : objectArray)
    {
        const QJsonObject objectJson = value.toObject();
        scene.environment_objects.push_back({
            objectJson.value(QStringLiteral("object_id")).toString(),
            objectJson.value(QStringLiteral("name")).toString(),
            objectJson.value(QStringLiteral("shape_type")).toString(),
            FromArray3(objectJson.value(QStringLiteral("size_m")).toArray()),
            FromArray3(objectJson.value(QStringLiteral("position_m")).toArray()),
            FromArray3(objectJson.value(QStringLiteral("rpy_deg")).toArray()),
            objectJson.value(QStringLiteral("enabled")).toBool(true)});
    }

    const QJsonObject configObject = jsonObject.value(QStringLiteral("planning_config")).toObject();
    scene.planning_config.motion_group = configObject.value(QStringLiteral("motion_group")).toString();
    scene.planning_config.planner_id = configObject.value(QStringLiteral("planner_id")).toString();
    scene.planning_config.service_endpoint = configObject.value(QStringLiteral("service_endpoint")).toString();
    scene.planning_config.allowed_planning_time_s = configObject.value(QStringLiteral("allowed_planning_time_s")).toDouble();
    scene.planning_config.max_planning_attempts = configObject.value(QStringLiteral("max_planning_attempts")).toInt(1);
    scene.planning_config.target_cycle_time_s = configObject.value(QStringLiteral("target_cycle_time_s")).toDouble();
    scene.planning_config.check_collision = configObject.value(QStringLiteral("check_collision")).toBool(true);
    scene.planning_config.check_self_collision = configObject.value(QStringLiteral("check_self_collision")).toBool(true);
    return scene;
}

QJsonObject PlanningJsonStorage::ToRequestsJsonObject(
    const std::vector<RoboSDP::Planning::Dto::PlanningRequestDto>& requests) const
{
    QJsonArray requestArray;
    for (const auto& request : requests)
    {
        QJsonObject requestObject;
        requestObject.insert(QStringLiteral("request_id"), request.request_id);
        requestObject.insert(QStringLiteral("planning_scene_ref"), request.planning_scene_ref);
        requestObject.insert(QStringLiteral("request_type"), request.request_type);
        // 保存最小 gRPC 方法路径，便于后续按稳定接口重新加载联调请求。
        requestObject.insert(QStringLiteral("rpc_method"), request.rpc_method);
        requestObject.insert(QStringLiteral("motion_group"), request.motion_group);
        requestObject.insert(QStringLiteral("planner_id"), request.planner_id);
        requestObject.insert(QStringLiteral("service_endpoint"), request.service_endpoint);
        requestObject.insert(QStringLiteral("check_collision"), request.check_collision);
        requestObject.insert(QStringLiteral("check_self_collision"), request.check_self_collision);
        requestObject.insert(QStringLiteral("allowed_planning_time_s"), request.allowed_planning_time_s);
        requestObject.insert(QStringLiteral("target_cycle_time_s"), request.target_cycle_time_s);
        requestObject.insert(QStringLiteral("joint_ids"), ToStringArray(request.joint_ids));
        requestObject.insert(QStringLiteral("start_joint_values_deg"), ToDoubleArray(request.start_joint_values_deg));
        requestObject.insert(QStringLiteral("goal_joint_values_deg"), ToDoubleArray(request.goal_joint_values_deg));
        requestArray.push_back(requestObject);
    }

    QJsonObject object;
    object.insert(QStringLiteral("requests"), requestArray);
    return object;
}

std::vector<RoboSDP::Planning::Dto::PlanningRequestDto> PlanningJsonStorage::FromRequestsJsonObject(
    const QJsonObject& jsonObject) const
{
    std::vector<RoboSDP::Planning::Dto::PlanningRequestDto> requests;
    const QJsonArray requestArray = jsonObject.value(QStringLiteral("requests")).toArray();
    requests.reserve(static_cast<std::size_t>(requestArray.size()));
    for (const auto& value : requestArray)
    {
        const QJsonObject requestObject = value.toObject();
        RoboSDP::Planning::Dto::PlanningRequestDto request = RoboSDP::Planning::Dto::PlanningRequestDto::CreateDefault();
        request.request_id = requestObject.value(QStringLiteral("request_id")).toString();
        request.planning_scene_ref = requestObject.value(QStringLiteral("planning_scene_ref")).toString();
        request.request_type = requestObject.value(QStringLiteral("request_type")).toString();
        // 若旧草稿没有该字段，则继续沿用 DTO 默认方法路径，兼容前一轮 skeleton 数据。
        request.rpc_method = requestObject.value(QStringLiteral("rpc_method")).toString(request.rpc_method);
        request.motion_group = requestObject.value(QStringLiteral("motion_group")).toString();
        request.planner_id = requestObject.value(QStringLiteral("planner_id")).toString();
        request.service_endpoint = requestObject.value(QStringLiteral("service_endpoint")).toString();
        request.check_collision = requestObject.value(QStringLiteral("check_collision")).toBool(true);
        request.check_self_collision = requestObject.value(QStringLiteral("check_self_collision")).toBool(true);
        request.allowed_planning_time_s = requestObject.value(QStringLiteral("allowed_planning_time_s")).toDouble();
        request.target_cycle_time_s = requestObject.value(QStringLiteral("target_cycle_time_s")).toDouble();
        request.joint_ids = FromStringArray(requestObject.value(QStringLiteral("joint_ids")).toArray());
        request.start_joint_values_deg = FromDoubleArray(requestObject.value(QStringLiteral("start_joint_values_deg")).toArray());
        request.goal_joint_values_deg = FromDoubleArray(requestObject.value(QStringLiteral("goal_joint_values_deg")).toArray());
        requests.push_back(request);
    }
    return requests;
}

QJsonObject PlanningJsonStorage::ToResultsJsonObject(
    const std::vector<RoboSDP::Planning::Dto::PlanningVerificationResultDto>& results) const
{
    QJsonArray resultArray;
    for (const auto& result : results)
    {
        QJsonObject resultObject;
        resultObject.insert(QStringLiteral("verification_id"), result.verification_id);
        resultObject.insert(QStringLiteral("planning_scene_ref"), result.planning_scene_ref);
        resultObject.insert(QStringLiteral("request_id"), result.request_id);
        resultObject.insert(QStringLiteral("success"), result.success);
        resultObject.insert(QStringLiteral("message"), result.message);
        resultObject.insert(QStringLiteral("feasibility_summary"), result.feasibility_summary);
        resultObject.insert(QStringLiteral("cycle_time_result"), ToCycleTimeResultObject(result.cycle_time_result));

        QJsonArray trajectoryArray;
        for (const auto& trajectory : result.trajectory_results)
        {
            trajectoryArray.push_back(ToTrajectoryResultObject(trajectory));
        }
        resultObject.insert(QStringLiteral("trajectory_results"), trajectoryArray);

        QJsonArray collisionArray;
        for (const auto& collision : result.collision_results)
        {
            collisionArray.push_back(ToCollisionResultObject(collision));
        }
        resultObject.insert(QStringLiteral("collision_results"), collisionArray);

        QJsonArray selfCollisionArray;
        for (const auto& selfCollision : result.self_collision_results)
        {
            selfCollisionArray.push_back(ToSelfCollisionResultObject(selfCollision));
        }
        resultObject.insert(QStringLiteral("self_collision_results"), selfCollisionArray);
        resultArray.push_back(resultObject);
    }

    QJsonObject object;
    object.insert(QStringLiteral("results"), resultArray);
    return object;
}

std::vector<RoboSDP::Planning::Dto::PlanningVerificationResultDto> PlanningJsonStorage::FromResultsJsonObject(
    const QJsonObject& jsonObject) const
{
    std::vector<RoboSDP::Planning::Dto::PlanningVerificationResultDto> results;
    const QJsonArray resultArray = jsonObject.value(QStringLiteral("results")).toArray();
    results.reserve(static_cast<std::size_t>(resultArray.size()));
    for (const auto& value : resultArray)
    {
        const QJsonObject resultObject = value.toObject();
        RoboSDP::Planning::Dto::PlanningVerificationResultDto result =
            RoboSDP::Planning::Dto::PlanningVerificationResultDto::CreateDefault();
        result.verification_id = resultObject.value(QStringLiteral("verification_id")).toString();
        result.planning_scene_ref = resultObject.value(QStringLiteral("planning_scene_ref")).toString();
        result.request_id = resultObject.value(QStringLiteral("request_id")).toString();
        result.success = resultObject.value(QStringLiteral("success")).toBool();
        result.message = resultObject.value(QStringLiteral("message")).toString();
        result.feasibility_summary = resultObject.value(QStringLiteral("feasibility_summary")).toString();
        result.cycle_time_result = FromCycleTimeResultObject(resultObject.value(QStringLiteral("cycle_time_result")).toObject());

        const QJsonArray trajectoryArray = resultObject.value(QStringLiteral("trajectory_results")).toArray();
        result.trajectory_results.reserve(static_cast<std::size_t>(trajectoryArray.size()));
        for (const auto& trajectoryValue : trajectoryArray)
        {
            result.trajectory_results.push_back(FromTrajectoryResultObject(trajectoryValue.toObject()));
        }

        const QJsonArray collisionArray = resultObject.value(QStringLiteral("collision_results")).toArray();
        result.collision_results.reserve(static_cast<std::size_t>(collisionArray.size()));
        for (const auto& collisionValue : collisionArray)
        {
            result.collision_results.push_back(FromCollisionResultObject(collisionValue.toObject()));
        }

        const QJsonArray selfCollisionArray = resultObject.value(QStringLiteral("self_collision_results")).toArray();
        result.self_collision_results.reserve(static_cast<std::size_t>(selfCollisionArray.size()));
        for (const auto& selfCollisionValue : selfCollisionArray)
        {
            result.self_collision_results.push_back(FromSelfCollisionResultObject(selfCollisionValue.toObject()));
        }

        results.push_back(result);
    }
    return results;
}

} // namespace RoboSDP::Planning::Persistence
