#include "modules/selection/service/ReducerSelectionService.h"

#include <algorithm>
#include <cmath>

namespace RoboSDP::Selection::Service
{

ReducerSelectionService::ReducerSelectionService(const RoboSDP::Selection::Catalog::JsonComponentCatalog& catalog)
    : m_catalog(catalog)
{
}

RoboSDP::Selection::Dto::ReducerSelectionResultDto ReducerSelectionService::Select(
    const RoboSDP::Selection::Dto::ReducerSelectionRequestDto& request) const
{
    RoboSDP::Selection::Dto::ReducerSelectionResultDto result;
    result.request = request;

    for (const auto& reducer : m_catalog.Reducers())
    {
        if (reducer.ratio + 1e-6 < request.target_ratio_min || reducer.ratio - 1e-6 > request.target_ratio_max)
        {
            continue;
        }

        if (reducer.rated_output_torque_nm + 1e-6 < request.rms_output_torque_nm)
        {
            continue;
        }

        if (reducer.peak_output_torque_nm + 1e-6 < request.peak_output_torque_nm)
        {
            continue;
        }

        if (reducer.max_input_speed_rpm + 1e-6 < request.motor_speed_limit_rpm)
        {
            continue;
        }

        RoboSDP::Selection::Dto::ReducerSelectionCandidateDto candidate;
        candidate.reducer = reducer;
        candidate.ratio_deviation = std::abs(reducer.ratio - request.target_ratio);
        candidate.rated_torque_margin_nm = reducer.rated_output_torque_nm - request.rms_output_torque_nm;
        candidate.peak_torque_margin_nm = reducer.peak_output_torque_nm - request.peak_output_torque_nm;
        candidate.ratio_rule_passed = true;
        candidate.recommendation_reason = QStringLiteral("满足输出扭矩、输入转速与目标减速比区间。");
        result.candidates.push_back(candidate);
    }

    std::sort(
        result.candidates.begin(),
        result.candidates.end(),
        [](const auto& left, const auto& right)
        {
            const double leftScore = left.reducer.efficiency * 100.0 -
                left.ratio_deviation * 0.8 -
                left.rated_torque_margin_nm * 0.01;
            const double rightScore = right.reducer.efficiency * 100.0 -
                right.ratio_deviation * 0.8 -
                right.rated_torque_margin_nm * 0.01;
            return leftScore > rightScore;
        });

    if (!result.candidates.empty())
    {
        result.has_recommendation = true;
        result.recommended = result.candidates.front();
        result.message = QStringLiteral("已生成减速器候选并给出最小推荐结果。");
    }
    else
    {
        result.message = QStringLiteral("未找到满足基础规则的减速器候选。");
    }

    return result;
}

} // namespace RoboSDP::Selection::Service
