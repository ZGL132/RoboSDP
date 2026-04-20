#pragma once

#include <QString>

#include <vector>

namespace RoboSDP::Planning::Dto
{

/// 单个轨迹路点 DTO。
struct TrajectoryWaypointDto
{
    double time_from_start_s = 0.0;
    std::vector<double> joint_values_deg;
};

/// 单次规划轨迹结果 DTO。
struct TrajectoryResultDto
{
    QString request_id;
    QString planner_id;
    bool success = false;
    QString message;
    double planning_time_s = 0.0;
    double trajectory_duration_s = 0.0;
    int waypoint_count = 0;
    std::vector<TrajectoryWaypointDto> waypoints;
};

/// 外部碰撞结果 DTO。
struct CollisionResultDto
{
    QString request_id;
    QString object_id;
    bool in_collision = false;
    QString message;
};

/// 自碰撞结果 DTO。
struct SelfCollisionResultDto
{
    QString request_id;
    QString link_a;
    QString link_b;
    bool in_self_collision = false;
    QString message;
};

/// 节拍评估结果 DTO。
struct CycleTimeResultDto
{
    QString request_id;
    double trajectory_duration_s = 0.0;
    double target_cycle_time_s = 0.0;
    bool within_target = false;
    double margin_s = 0.0;
    QString message;
};

/**
 * @brief MoveIt gRPC Adapter 响应 DTO。
 *
 * 当前响应对象只承载最小规划验证链路需要的数据，
 * 既可以对应未来真实 gRPC 响应，也可以承载本轮 skeleton 返回结果。
 */
struct PlanningResponseDto
{
    QString request_id;
    QString adapter_name = QStringLiteral("moveit_grpc_skeleton");
    QString service_endpoint;
    /// gRPC 状态码，0 表示服务端正常返回。
    int grpc_status_code = -1;
    bool transport_success = false;
    bool planning_success = false;
    QString message;
    TrajectoryResultDto trajectory_result;
    std::vector<CollisionResultDto> collision_results;
    std::vector<SelfCollisionResultDto> self_collision_results;
    CycleTimeResultDto cycle_time_result;

    static PlanningResponseDto CreateDefault()
    {
        return {};
    }
};

} // namespace RoboSDP::Planning::Dto
