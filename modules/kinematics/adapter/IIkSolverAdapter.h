#pragma once

#include "modules/kinematics/dto/KinematicModelDto.h"
#include "modules/kinematics/dto/KinematicSolverResultDto.h"

#include <QString>

namespace RoboSDP::Kinematics::Adapter
{

/**
 * @brief IK（逆运动学）求解器的统一适配器接口。
 * @details
 * Stage 16 之后，系统只保留 Pinocchio 数值 IK 主链；但 Service 层依然只依赖这条稳定接口，
 * 从而保证 DTO / Service / Adapter 的清晰分层不被破坏。
 */
class IIkSolverAdapter
{
public:
    virtual ~IIkSolverAdapter() = default;

    /// @brief 获取求解器唯一标识，便于日志与遥测跟踪。
    virtual QString SolverId() const = 0;

    /// @brief 获取求解器中文描述，供界面和调试输出显示。
    virtual QString SolverDescription() const = 0;

    /// @brief 执行逆运动学（IK）求解。
    virtual RoboSDP::Kinematics::Dto::IkResultDto SolveIk(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const RoboSDP::Kinematics::Dto::IkRequestDto& request) const = 0;
};

} // namespace RoboSDP::Kinematics::Adapter
