#pragma once

#include "modules/selection/dto/MotorSelectionDto.h"
#include "modules/selection/dto/ReducerSelectionDto.h"

#include <vector>

namespace RoboSDP::Selection::Catalog
{

/**
 * @brief 第一阶段样例元件目录。
 *
 * 本目录只提供少量样例电机和减速器数据，
 * 目的不是覆盖真实市场，而是支撑最小推荐闭环验证。
 */
class Stage1ComponentCatalog
{
public:
    const std::vector<RoboSDP::Selection::Dto::MotorCatalogItemDto>& Motors() const;
    const std::vector<RoboSDP::Selection::Dto::ReducerCatalogItemDto>& Reducers() const;

private:
    std::vector<RoboSDP::Selection::Dto::MotorCatalogItemDto> m_motors {
        {QStringLiteral("motor_040b"), QStringLiteral("伺服电机 040B"), QStringLiteral("Stage1Vendor"), 0.40, 1.20, 3000.0, 5000.0, 200.0, 0.00008, 0.90, true, 0.8},
        {QStringLiteral("motor_075b"), QStringLiteral("伺服电机 075B"), QStringLiteral("Stage1Vendor"), 0.75, 2.30, 3000.0, 5000.0, 400.0, 0.00012, 0.91, true, 1.5},
        {QStringLiteral("motor_150b"), QStringLiteral("伺服电机 150B"), QStringLiteral("Stage1Vendor"), 1.50, 4.50, 2500.0, 4500.0, 750.0, 0.00028, 0.92, true, 3.0},
        {QStringLiteral("motor_300"), QStringLiteral("伺服电机 300"), QStringLiteral("Stage1Vendor"), 3.00, 9.00, 2000.0, 4000.0, 1500.0, 0.00055, 0.93, false, 0.0},
        {QStringLiteral("motor_450b"), QStringLiteral("伺服电机 450B"), QStringLiteral("Stage1Vendor"), 4.50, 13.0, 2000.0, 3500.0, 2000.0, 0.00110, 0.93, true, 6.0}
    };

    std::vector<RoboSDP::Selection::Dto::ReducerCatalogItemDto> m_reducers {
        {QStringLiteral("reducer_050"), QStringLiteral("精密减速器 050"), QStringLiteral("Stage1Vendor"), 50.0, 120.0, 240.0, 5000.0, 0.94, 8.0},
        {QStringLiteral("reducer_080"), QStringLiteral("精密减速器 080"), QStringLiteral("Stage1Vendor"), 80.0, 180.0, 360.0, 4500.0, 0.93, 8.0},
        {QStringLiteral("reducer_100"), QStringLiteral("精密减速器 100"), QStringLiteral("Stage1Vendor"), 100.0, 240.0, 480.0, 4000.0, 0.92, 7.0},
        {QStringLiteral("reducer_120"), QStringLiteral("精密减速器 120"), QStringLiteral("Stage1Vendor"), 120.0, 320.0, 640.0, 4000.0, 0.91, 7.0},
        {QStringLiteral("reducer_160"), QStringLiteral("精密减速器 160"), QStringLiteral("Stage1Vendor"), 160.0, 450.0, 900.0, 3500.0, 0.90, 6.0}
    };
};

} // namespace RoboSDP::Selection::Catalog
