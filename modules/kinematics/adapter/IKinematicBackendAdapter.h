#pragma once

#include "modules/kinematics/dto/KinematicModelDto.h"
#include "modules/kinematics/dto/KinematicSolverResultDto.h"

#include <QString>

namespace RoboSDP::Kinematics::Adapter
{

/**
 * @brief 运动学核心后端的统一适配器接口。
 * @details
 * Stage 16 之后，Service 层只依赖这组稳定边界，不再感知任何旧时代回退实现：
 * 1. FK 与工作空间采样都通过该接口进入共享 Pinocchio 内核。
 * 2. 上层只交换 DTO，不暴露任何 Pinocchio / Eigen 原生类型。
 * 3. 若未来继续扩展诊断或渲染接线，也必须沿用这条分层边界。
 */
class IKinematicBackendAdapter
{
public:
    virtual ~IKinematicBackendAdapter() = default;

    /// @brief 获取后端唯一标识，便于日志与遥测定位当前使用的运动学引擎。
    virtual QString BackendId() const = 0;

    /// @brief 获取后端中文描述，供状态面板和调试输出展示。
    virtual QString BackendDescription() const = 0;

    /// @brief 查询当前后端是否已经接入统一共享机器人内核。
    virtual bool UsesSharedRobotKernel() const = 0;

    /// @brief 执行正运动学（FK）求解。
    virtual RoboSDP::Kinematics::Dto::FkResultDto SolveFk(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const RoboSDP::Kinematics::Dto::FkRequestDto& request) const = 0;

    /// @brief 执行工作空间采样。
    virtual RoboSDP::Kinematics::Dto::WorkspaceResultDto SampleWorkspace(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const RoboSDP::Kinematics::Dto::WorkspaceRequestDto& request) const = 0;
};

} // namespace RoboSDP::Kinematics::Adapter
