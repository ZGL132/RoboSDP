#pragma once

#include "modules/dynamics/dto/DynamicModelDto.h"
#include "modules/kinematics/dto/KinematicModelDto.h"

namespace RoboSDP::Dynamics::Trajectory
{

/**
 * @brief 基准轨迹工厂。
 *
 * 本轮只生成一阶段逆动力学主链所需的最小基准轨迹集合，
 * 用于后续时间参数化与负载统计。
 */
class BenchmarkTrajectoryFactory
{
public:
    std::vector<RoboSDP::Dynamics::Dto::BenchmarkTrajectoryDto> CreateMinimumSet(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& kinematicModel) const;
};

} // namespace RoboSDP::Dynamics::Trajectory
