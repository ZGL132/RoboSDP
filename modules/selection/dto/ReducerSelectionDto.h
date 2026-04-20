#pragma once

#include <QString>

#include <vector>

namespace RoboSDP::Selection::Dto
{

/// 样例减速器目录项 DTO。
struct ReducerCatalogItemDto
{
    QString reducer_id;
    QString name;
    QString vendor;
    double ratio = 1.0;
    double rated_output_torque_nm = 0.0;
    double peak_output_torque_nm = 0.0;
    double max_input_speed_rpm = 0.0;
    double efficiency = 0.9;
    double backlash_arcmin = 0.0;
};

/**
 * @brief 减速器选型请求 DTO。
 *
 * 请求中保留关节输出侧负载、目标减速比区间和输入转速上限，
 * 供第一阶段基础筛选规则使用。
 */
struct ReducerSelectionRequestDto
{
    QString dynamic_ref;
    QString kinematic_ref;
    QString topology_ref;
    QString requirement_ref;
    QString joint_id;
    double peak_output_torque_nm = 0.0;
    double rms_output_torque_nm = 0.0;
    double peak_output_speed_deg_s = 0.0;
    double target_ratio = 1.0;
    double target_ratio_min = 1.0;
    double target_ratio_max = 1.0;
    double motor_speed_limit_rpm = 0.0;
};

/// 减速器候选 DTO。
struct ReducerSelectionCandidateDto
{
    ReducerCatalogItemDto reducer;
    double ratio_deviation = 0.0;
    double rated_torque_margin_nm = 0.0;
    double peak_torque_margin_nm = 0.0;
    bool ratio_rule_passed = false;
    QString recommendation_reason;
};

/// 减速器选型结果 DTO。
struct ReducerSelectionResultDto
{
    ReducerSelectionRequestDto request;
    std::vector<ReducerSelectionCandidateDto> candidates;
    bool has_recommendation = false;
    ReducerSelectionCandidateDto recommended;
    QString message;
};

} // namespace RoboSDP::Selection::Dto
