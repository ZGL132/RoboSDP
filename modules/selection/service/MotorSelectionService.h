#pragma once

#include "modules/selection/catalog/JsonComponentCatalog.h"
#include "modules/selection/dto/MotorSelectionDto.h"

namespace RoboSDP::Selection::Service
{

/**
 * @brief 电机选型服务。
 *
 * 第一阶段只做基础规则筛选：
 * 1. 抱闸要求命中；
 * 2. 功率满足输出需求；
 * 3. 转速落在预估减速比可覆盖区间；
 * 4. 推荐优先高效率且不过度超配。
 */
class MotorSelectionService
{
public:
    explicit MotorSelectionService(const RoboSDP::Selection::Catalog::JsonComponentCatalog& catalog);

    RoboSDP::Selection::Dto::MotorSelectionResultDto Select(
        const RoboSDP::Selection::Dto::MotorSelectionRequestDto& request) const;

private:
    const RoboSDP::Selection::Catalog::JsonComponentCatalog& m_catalog;
};

} // namespace RoboSDP::Selection::Service
