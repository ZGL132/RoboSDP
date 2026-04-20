#pragma once

#include "modules/dynamics/dto/DynamicModelDto.h"
#include "modules/dynamics/dto/DynamicsResultDto.h"

namespace RoboSDP::Dynamics::Trajectory
{

/**
 * @brief 时间参数化骨架。
 *
 * 当前实现使用统一平滑插值函数生成位置、速度和加速度序列，
 * 只服务于一阶段逆动力学主链，不追求高级 jerk 优化。
 */
class TimeParameterizationSkeleton
{
public:
    RoboSDP::Dynamics::Dto::ParameterizedTrajectoryDto Parameterize(
        const RoboSDP::Dynamics::Dto::BenchmarkTrajectoryDto& trajectory) const;
};

} // namespace RoboSDP::Dynamics::Trajectory
