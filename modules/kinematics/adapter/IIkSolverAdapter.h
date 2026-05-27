#pragma once

#include "modules/kinematics/dto/KinematicModelDto.h"
#include "modules/kinematics/dto/KinematicSolverResultDto.h"

#include <QString>

namespace RoboSDP::Kinematics::Adapter
{

/**
 * @brief IK（逆运动学）求解器的统一适配器接口。
 * @details
 * 声明了获取求解器 ID、中文描述以及执行 IK 求解的纯虚函数 SolveIk。Service 层仅依赖该接口，屏蔽了底层是采用“数值迭代法”还是“闭式解析法”的具体实现细节。
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
