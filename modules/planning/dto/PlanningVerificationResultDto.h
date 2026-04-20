#pragma once

#include "modules/planning/dto/PlanningRequestDto.h"
#include "modules/planning/dto/PlanningResponseDto.h"
#include "modules/planning/dto/PlanningSceneDto.h"

#include <QString>

#include <vector>

namespace RoboSDP::Planning::Dto
{

/**
 * @brief 规划验证结果 DTO。
 *
 * 该对象是阶段 6 的核心结果对象，
 * 用于汇总轨迹结果、碰撞结果、自碰撞结果与节拍评估摘要。
 */
struct PlanningVerificationResultDto
{
    QString verification_id = QStringLiteral("planning_verification_001");
    QString planning_scene_ref;
    QString request_id;
    bool success = false;
    QString message;
    std::vector<TrajectoryResultDto> trajectory_results;
    std::vector<CollisionResultDto> collision_results;
    std::vector<SelfCollisionResultDto> self_collision_results;
    CycleTimeResultDto cycle_time_result;
    QString feasibility_summary;

    static PlanningVerificationResultDto CreateDefault()
    {
        return {};
    }
};

/// Planning 模块持久化工作态 DTO。
struct PlanningWorkspaceStateDto
{
    PlanningSceneDto current_scene;
    std::vector<PlanningRequestDto> requests;
    std::vector<PlanningVerificationResultDto> results;

    static PlanningWorkspaceStateDto CreateDefault()
    {
        PlanningWorkspaceStateDto state;
        state.current_scene = PlanningSceneDto::CreateDefault();
        return state;
    }
};

} // namespace RoboSDP::Planning::Dto
