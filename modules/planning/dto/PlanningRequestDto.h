#pragma once

#include <QString>

#include <vector>

namespace RoboSDP::Planning::Dto
{

/**
 * @brief 点到点规划请求 DTO。
 *
 * 该请求对象承载 Qt 主程序发送给 MoveIt gRPC Adapter 的最小字段，
 * 当前只覆盖单次点到点规划验证，不扩展到复杂路径约束和批量任务。
 */
struct PlanningRequestDto
{
    QString request_id = QStringLiteral("planning_request_001");
    QString planning_scene_ref;
    QString request_type = QStringLiteral("point_to_point");
    /// gRPC 方法路径，本轮固定为最小点到点规划验证接口。
    QString rpc_method =
        QStringLiteral("/robosdp.planning.v1.PlanningVerificationService/VerifyPointToPoint");
    QString motion_group = QStringLiteral("manipulator");
    QString planner_id = QStringLiteral("RRTConnect");
    QString service_endpoint = QStringLiteral("127.0.0.1:50051");
    bool check_collision = true;
    bool check_self_collision = true;
    double allowed_planning_time_s = 1.0;
    double target_cycle_time_s = 4.0;
    std::vector<QString> joint_ids;
    std::vector<double> start_joint_values_deg;
    std::vector<double> goal_joint_values_deg;

    static PlanningRequestDto CreateDefault()
    {
        return {};
    }
};

} // namespace RoboSDP::Planning::Dto
