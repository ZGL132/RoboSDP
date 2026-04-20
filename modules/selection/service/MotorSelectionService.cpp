#include "modules/selection/service/MotorSelectionService.h"

#include <algorithm>
#include <cmath>

namespace RoboSDP::Selection::Service
{

namespace
{

double DegPerSecondToRpm(double valueDegPerSecond)
{
    return valueDegPerSecond / 6.0;
}

} // namespace

MotorSelectionService::MotorSelectionService(const RoboSDP::Selection::Catalog::JsonComponentCatalog& catalog)
    : m_catalog(catalog)
{
}

RoboSDP::Selection::Dto::MotorSelectionResultDto MotorSelectionService::Select(
    const RoboSDP::Selection::Dto::MotorSelectionRequestDto& request) const
{
    RoboSDP::Selection::Dto::MotorSelectionResultDto result;
    result.request = request;

    const double requiredMotorSpeedMinRpm = DegPerSecondToRpm(request.peak_output_speed_deg_s) * request.preferred_ratio_min;
    const double requiredMotorPowerW = std::max(0.0, request.peak_output_power_w * 0.80);

    for (const auto& motor : m_catalog.Motors())
    {
        if (request.brake_required && !motor.has_brake)
        {
            continue;
        }

        if (motor.max_speed_rpm + 1e-6 < requiredMotorSpeedMinRpm)
        {
            continue;
        }

        if (motor.rated_power_w + 1e-6 < requiredMotorPowerW)
        {
            continue;
        }

        RoboSDP::Selection::Dto::MotorSelectionCandidateDto candidate;
        candidate.motor = motor;
        candidate.estimated_rated_output_torque_nm = motor.rated_torque_nm * request.preferred_ratio_min * motor.efficiency;
        candidate.estimated_peak_output_torque_nm = motor.peak_torque_nm * request.preferred_ratio_max * motor.efficiency;
        candidate.estimated_peak_output_speed_deg_s =
            motor.max_speed_rpm * 6.0 / std::max(1.0, request.preferred_ratio_min);
        candidate.torque_margin_nm = candidate.estimated_peak_output_torque_nm - request.peak_output_torque_nm;
        candidate.speed_margin_deg_s = candidate.estimated_peak_output_speed_deg_s - request.peak_output_speed_deg_s;
        candidate.power_margin_w = motor.rated_power_w - requiredMotorPowerW;
        candidate.brake_rule_passed = !request.brake_required ||
            motor.brake_holding_torque_nm * request.preferred_ratio_min * motor.efficiency >= request.required_brake_output_torque_nm;
        candidate.recommendation_reason = QStringLiteral("满足功率、转速与基础抱闸约束。");
        result.candidates.push_back(candidate);
    }

    std::sort(
        result.candidates.begin(),
        result.candidates.end(),
        [](const auto& left, const auto& right)
        {
            const double leftScore = (left.brake_rule_passed ? 1000.0 : 0.0) +
                left.motor.efficiency * 100.0 -
                std::abs(left.torque_margin_nm) * 0.05 -
                std::abs(left.power_margin_w) * 0.001;
            const double rightScore = (right.brake_rule_passed ? 1000.0 : 0.0) +
                right.motor.efficiency * 100.0 -
                std::abs(right.torque_margin_nm) * 0.05 -
                std::abs(right.power_margin_w) * 0.001;
            return leftScore > rightScore;
        });

    if (!result.candidates.empty())
    {
        result.has_recommendation = true;
        result.recommended = result.candidates.front();
        result.message = QStringLiteral("已生成电机候选并给出最小推荐结果。");
    }
    else
    {
        result.message = QStringLiteral("未找到满足基础规则的电机候选。");
    }

    return result;
}

} // namespace RoboSDP::Selection::Service
