#pragma once

#include "modules/dynamics/dto/DynamicModelDto.h"
#include "modules/dynamics/dto/DynamicsResultDto.h"
#include "modules/kinematics/dto/KinematicModelDto.h"

namespace RoboSDP::Dynamics::Adapter
{

/**
 * @brief 动力学后端适配器统一接口。
 *
 * @details
 * 该接口只定义动力学模块对“后端计算引擎”的最小依赖边界，
 * 便于后续在不修改 Service/UI 编排的情况下切换 Pinocchio 或其他实现。
 */
class IDynamicsBackendAdapter
{
public:
    virtual ~IDynamicsBackendAdapter() = default;

    /// @brief 返回后端稳定标识。
    virtual QString BackendId() const = 0;

    /// @brief 返回后端中文描述。
    virtual QString BackendDescription() const = 0;

    /// @brief 返回当前后端是否已真实接入共享 Pinocchio 物理内核。
    virtual bool UsesSharedRobotKernel() const = 0;

    /// @brief 计算单条时间参数化轨迹的逆动力学结果。
    virtual RoboSDP::Dynamics::Dto::InverseDynamicsTrajectoryResultDto Analyze(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& kinematicModel,
        const RoboSDP::Dynamics::Dto::DynamicModelDto& dynamicModel,
        const RoboSDP::Dynamics::Dto::ParameterizedTrajectoryDto& trajectory) const = 0;
};

} // namespace RoboSDP::Dynamics::Adapter
