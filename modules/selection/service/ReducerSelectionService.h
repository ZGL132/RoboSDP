#pragma once

#include "modules/selection/catalog/JsonComponentCatalog.h"
#include "modules/selection/dto/ReducerSelectionDto.h"

namespace RoboSDP::Selection::Service
{

/**
 * @brief 减速器选型服务。
 *
 * 第一阶段只做输出扭矩、输入转速和减速比区间匹配，
 * 推荐时优先选择比值更接近目标且效率更高的候选。
 */
class ReducerSelectionService
{
public:
    explicit ReducerSelectionService(const RoboSDP::Selection::Catalog::JsonComponentCatalog& catalog);

    RoboSDP::Selection::Dto::ReducerSelectionResultDto Select(
        const RoboSDP::Selection::Dto::ReducerSelectionRequestDto& request) const;

private:
    const RoboSDP::Selection::Catalog::JsonComponentCatalog& m_catalog;
};

} // namespace RoboSDP::Selection::Service
