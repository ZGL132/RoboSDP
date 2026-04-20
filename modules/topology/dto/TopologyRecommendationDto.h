#pragma once

#include "modules/topology/dto/RobotTopologyModelDto.h"

#include <QString>

#include <vector>

namespace RoboSDP::Topology::Dto
{

/**
 * @brief 单个候选构型 DTO。
 *
 * 候选对象保存模板生成后的拓扑骨架、最小评分和推荐理由，
 * 供 UI 列表展示和后续推荐逻辑消费。
 */
struct TopologyCandidateDto
{
    QString candidate_id;
    QString title;
    QString template_id;
    double score = 0.0;
    bool matches_requirement = false;
    std::vector<QString> recommendation_reason;
    RobotTopologyModelDto model;
};

/**
 * @brief Topology 推荐结果 DTO。
 *
 * 当前阶段仅提供规则型最小推荐结果，不引入高级评分器和自动优化。
 */
struct TopologyRecommendationDto
{
    QString recommended_candidate_id;
    QString recommended_topology_id;
    QString recommended_template_id;
    double combined_score = 0.0;
    bool requirement_loaded = false;
    bool requirement_constraints_applied = false;
    std::vector<QString> recommendation_reason;
};

/**
 * @brief Topology 页面工作态 DTO。
 *
 * 为了满足保存/加载链路，这里把当前构型、候选列表和推荐结果放在一起，
 * 序列化时仍以 `RobotTopologyModel` 根结构为主体，额外挂接候选与推荐信息。
 */
struct TopologyWorkspaceStateDto
{
    RobotTopologyModelDto current_model;
    std::vector<TopologyCandidateDto> candidates;
    TopologyRecommendationDto recommendation;

    static TopologyWorkspaceStateDto CreateDefault()
    {
        TopologyWorkspaceStateDto dto;
        dto.current_model = RobotTopologyModelDto::CreateDefault();
        return dto;
    }
};

} // namespace RoboSDP::Topology::Dto
