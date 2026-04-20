#include "modules/planning/adapter/MoveItGrpcAdapter.h"

#include <algorithm>
#include <cmath>

namespace RoboSDP::Planning::Adapter
{

namespace
{

bool IsGoalInsideLimits(
    const RoboSDP::Planning::Dto::PlanningSceneDto& scene,
    const RoboSDP::Planning::Dto::PlanningRequestDto& request,
    QString* message)
{
    if (scene.joint_limits.size() != request.goal_joint_values_deg.size())
    {
        if (message != nullptr)
        {
            *message = QStringLiteral("目标关节数量与规划场景限位数量不一致。");
        }
        return false;
    }

    for (std::size_t index = 0; index < scene.joint_limits.size(); ++index)
    {
        const auto& limit = scene.joint_limits.at(index);
        const double goal = request.goal_joint_values_deg.at(index);
        if (goal + 1e-6 < limit.lower_deg || goal - 1e-6 > limit.upper_deg)
        {
            if (message != nullptr)
            {
                *message = QStringLiteral("%1 的目标角度超出限位区间。").arg(limit.joint_id);
            }
            return false;
        }
    }

    return true;
}

std::vector<RoboSDP::Planning::Dto::TrajectoryWaypointDto> BuildLinearWaypoints(
    const std::vector<double>& startJointValuesDeg,
    const std::vector<double>& goalJointValuesDeg,
    int waypointCount,
    double trajectoryDurationS)
{
    std::vector<RoboSDP::Planning::Dto::TrajectoryWaypointDto> waypoints;
    const int safeWaypointCount = std::max(2, waypointCount);
    waypoints.reserve(static_cast<std::size_t>(safeWaypointCount));

    for (int index = 0; index < safeWaypointCount; ++index)
    {
        const double ratio = static_cast<double>(index) / static_cast<double>(safeWaypointCount - 1);
        RoboSDP::Planning::Dto::TrajectoryWaypointDto waypoint;
        waypoint.time_from_start_s = trajectoryDurationS * ratio;
        waypoint.joint_values_deg.reserve(startJointValuesDeg.size());

        for (std::size_t jointIndex = 0; jointIndex < startJointValuesDeg.size(); ++jointIndex)
        {
            const double startValue = startJointValuesDeg.at(jointIndex);
            const double goalValue = goalJointValuesDeg.at(jointIndex);
            waypoint.joint_values_deg.push_back(startValue + (goalValue - startValue) * ratio);
        }

        waypoints.push_back(waypoint);
    }

    return waypoints;
}

double EstimateTrajectoryDuration(const std::vector<double>& startJointValuesDeg, const std::vector<double>& goalJointValuesDeg)
{
    double maxDeltaDeg = 0.0;
    for (std::size_t index = 0; index < startJointValuesDeg.size() && index < goalJointValuesDeg.size(); ++index)
    {
        maxDeltaDeg = std::max(maxDeltaDeg, std::abs(goalJointValuesDeg.at(index) - startJointValuesDeg.at(index)));
    }

    return std::max(0.6, maxDeltaDeg / 90.0 + 0.4);
}

} // namespace

MoveItGrpcAdapter::MoveItGrpcAdapter(RoboSDP::Logging::ILogger* logger)
    : m_logger(logger)
{
}

RoboSDP::Planning::Dto::PlanningResponseDto MoveItGrpcAdapter::VerifyPointToPoint(
    const RoboSDP::Planning::Dto::PlanningSceneDto& scene,
    const RoboSDP::Planning::Dto::PlanningRequestDto& request) const
{
    RoboSDP::Planning::Dto::PlanningResponseDto response = RoboSDP::Planning::Dto::PlanningResponseDto::CreateDefault();
    response.request_id = request.request_id;
    response.service_endpoint = request.service_endpoint;
    response.transport_success = !request.service_endpoint.trimmed().isEmpty();
    response.trajectory_result.request_id = request.request_id;
    response.trajectory_result.planner_id = request.planner_id;
    response.cycle_time_result.request_id = request.request_id;
    response.cycle_time_result.target_cycle_time_s = request.target_cycle_time_s;

    if (!response.transport_success)
    {
        response.message = QStringLiteral("MoveIt gRPC 服务地址为空，无法发起规划验证。");
        return response;
    }

    if (request.start_joint_values_deg.size() != request.goal_joint_values_deg.size() ||
        request.start_joint_values_deg.size() != request.joint_ids.size())
    {
        response.message = QStringLiteral("规划请求中的关节 ID、起点与终点数量不一致。");
        return response;
    }

    if (request.allowed_planning_time_s <= 0.05)
    {
        response.message = QStringLiteral("允许规划时间过小，gRPC skeleton 直接返回超时失败。");
        response.cycle_time_result.message = QStringLiteral("未生成轨迹，无法评估节拍。");
        return response;
    }

    QString limitMessage;
    if (!IsGoalInsideLimits(scene, request, &limitMessage))
    {
        response.message = limitMessage;
        response.cycle_time_result.message = QStringLiteral("目标关节超限，无法评估节拍。");
        return response;
    }

    const double trajectoryDurationS = EstimateTrajectoryDuration(
        request.start_joint_values_deg,
        request.goal_joint_values_deg);

    response.trajectory_result.success = true;
    response.trajectory_result.message = QStringLiteral("MoveIt gRPC skeleton 已返回点到点规划轨迹。");
    response.trajectory_result.planning_time_s = std::min(0.25, std::max(0.05, request.allowed_planning_time_s * 0.25));
    response.trajectory_result.trajectory_duration_s = trajectoryDurationS;
    response.trajectory_result.waypoint_count = 11;
    response.trajectory_result.waypoints = BuildLinearWaypoints(
        request.start_joint_values_deg,
        request.goal_joint_values_deg,
        response.trajectory_result.waypoint_count,
        trajectoryDurationS);

    if (request.check_collision)
    {
        RoboSDP::Planning::Dto::CollisionResultDto collisionResult;
        collisionResult.request_id = request.request_id;
        collisionResult.object_id = scene.environment_objects.empty()
            ? QStringLiteral("environment")
            : scene.environment_objects.front().object_id;
        collisionResult.in_collision =
            !scene.environment_objects.empty() &&
            request.goal_joint_values_deg.size() > 1 &&
            request.goal_joint_values_deg.at(1) < -115.0;
        collisionResult.message = collisionResult.in_collision
            ? QStringLiteral("目标姿态与环境障碍物存在碰撞。")
            : QStringLiteral("未发现环境碰撞。");
        response.collision_results.push_back(collisionResult);
    }

    if (request.check_self_collision)
    {
        RoboSDP::Planning::Dto::SelfCollisionResultDto selfCollisionResult;
        selfCollisionResult.request_id = request.request_id;
        selfCollisionResult.link_a = QStringLiteral("link_2");
        selfCollisionResult.link_b = QStringLiteral("link_4");
        selfCollisionResult.in_self_collision =
            request.goal_joint_values_deg.size() > 2 &&
            request.goal_joint_values_deg.at(1) < -110.0 &&
            request.goal_joint_values_deg.at(2) > 120.0;
        selfCollisionResult.message = selfCollisionResult.in_self_collision
            ? QStringLiteral("检测到机械臂自碰撞。")
            : QStringLiteral("未发现机械臂自碰撞。");
        response.self_collision_results.push_back(selfCollisionResult);
    }

    const bool collisionPassed =
        std::none_of(
            response.collision_results.begin(),
            response.collision_results.end(),
            [](const auto& item) { return item.in_collision; });
    const bool selfCollisionPassed =
        std::none_of(
            response.self_collision_results.begin(),
            response.self_collision_results.end(),
            [](const auto& item) { return item.in_self_collision; });

    response.cycle_time_result.trajectory_duration_s = trajectoryDurationS;
    response.cycle_time_result.within_target = trajectoryDurationS <= request.target_cycle_time_s + 1e-6;
    response.cycle_time_result.margin_s = request.target_cycle_time_s - trajectoryDurationS;
    response.cycle_time_result.message = response.cycle_time_result.within_target
        ? QStringLiteral("轨迹总时长满足目标节拍。")
        : QStringLiteral("轨迹总时长超过目标节拍。");

    response.planning_success = response.trajectory_result.success && collisionPassed && selfCollisionPassed;
    response.message = response.planning_success
        ? QStringLiteral("MoveIt gRPC skeleton 点到点规划验证成功。")
        : QStringLiteral("MoveIt gRPC skeleton 返回失败，请检查碰撞或目标姿态。");

    if (!response.planning_success)
    {
        response.trajectory_result.success = false;
        response.trajectory_result.message = response.message;
    }

    return response;
}

} // namespace RoboSDP::Planning::Adapter
