#pragma once

#include "modules/dynamics/dto/DynamicModelDto.h"
#include "modules/dynamics/dto/DynamicsResultDto.h"
#include "modules/kinematics/dto/KinematicModelDto.h"

namespace RoboSDP::Dynamics::Adapter
{

/**
 * @brief 动力学后端诊断接口。
 *
 * @details
 * 本接口只暴露轻量的状态查询和 Dry-Run 验证能力，
 * 用于单测和后续灰度观测，不改变现有 Dynamics 业务主链。
 */
class IDynamicsBackendDiagnosticsAdapter
{
public:
    virtual ~IDynamicsBackendDiagnosticsAdapter() = default;

    /// @brief 返回最近一次共享内核构建的中文状态说明。
    virtual QString ExplainWhySharedKernelNotReady() const = 0;

    /// @brief 返回最近一次原生模型真实构建次数，用于验证缓存是否生效。
    virtual int GetNativeModelBuildCount() const = 0;

    /// @brief 执行 Pinocchio 原生 RNEA 干跑，仅供诊断验证。
    virtual RoboSDP::Dynamics::Dto::NativeRneaDryRunResultDto EvaluateNativeRneaDryRun(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& kinematicModel,
        const RoboSDP::Dynamics::Dto::DynamicModelDto& dynamicModel,
        const std::vector<double>& joint_positions_deg,
        const std::vector<double>& joint_velocities_deg_s,
        const std::vector<double>& joint_accelerations_deg_s2) const = 0;
};

} // namespace RoboSDP::Dynamics::Adapter
