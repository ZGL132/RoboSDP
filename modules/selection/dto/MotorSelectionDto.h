#pragma once

#include <QString>

#include <vector>

namespace RoboSDP::Selection::Dto
{

/// 样例电机目录项 DTO。
struct MotorCatalogItemDto
{
    QString motor_id;
    QString name;
    QString vendor;
    double rated_torque_nm = 0.0;
    double peak_torque_nm = 0.0;
    double rated_speed_rpm = 0.0;
    double max_speed_rpm = 0.0;
    double rated_power_w = 0.0;
    double rotor_inertia_kg_m2 = 0.0;
    double efficiency = 0.9;
    bool has_brake = false;
    double brake_holding_torque_nm = 0.0;
};

/**
 * @brief 电机选型请求 DTO。
 *
 * 该请求只保留第一阶段最小规则所需字段：
 * 1. 上游引用链；
 * 2. 关节负载需求；
 * 3. 预估减速比区间；
 * 4. 抱闸基础校核输入。
 */
struct MotorSelectionRequestDto
{
    QString dynamic_ref;
    QString kinematic_ref;
    QString topology_ref;
    QString requirement_ref;
    QString joint_id;
    double peak_output_torque_nm = 0.0;
    double rms_output_torque_nm = 0.0;
    double peak_output_speed_deg_s = 0.0;
    double peak_output_power_w = 0.0;
    double preferred_ratio_min = 1.0;
    double preferred_ratio_max = 1.0;
    bool brake_required = false;
    double required_brake_output_torque_nm = 0.0;
};

/// 电机候选结果 DTO。
struct MotorSelectionCandidateDto
{
    MotorCatalogItemDto motor;
    double estimated_peak_output_torque_nm = 0.0;
    double estimated_rated_output_torque_nm = 0.0;
    double estimated_peak_output_speed_deg_s = 0.0;
    double torque_margin_nm = 0.0;
    double speed_margin_deg_s = 0.0;
    double power_margin_w = 0.0;
    bool brake_rule_passed = false;
    QString recommendation_reason;
};

/// 电机选型结果 DTO。
struct MotorSelectionResultDto
{
    MotorSelectionRequestDto request;
    std::vector<MotorSelectionCandidateDto> candidates;
    bool has_recommendation = false;
    MotorSelectionCandidateDto recommended;
    QString message;
};

} // namespace RoboSDP::Selection::Dto
