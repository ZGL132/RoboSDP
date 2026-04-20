#pragma once

#include "core/logging/ILogger.h"
#include "modules/kinematics/adapter/IIkSolverAdapter.h"

#include <memory>

namespace RoboSDP::Kinematics::Adapter
{

/**
 * @brief 基于 Pinocchio 干跑能力构建的数值 IK 适配器。
 * @details
 * 本类在 Stage 16 之后承担纯 Pinocchio 的 IK 主链职责：
 * 1. 对外继续只暴露 IIkSolverAdapter，避免上层感知三方细节。
 * 2. 对内维护最小缓存状态，避免重复做 build-context 准备。
 * 3. 失败时直接返回明确错误，不再保留任何 Legacy 回退逻辑。
 */
class PinocchioIkSolverAdapter final : public IIkSolverAdapter
{
public:
    explicit PinocchioIkSolverAdapter(RoboSDP::Logging::ILogger* logger = nullptr);
    ~PinocchioIkSolverAdapter() override;

    QString SolverId() const override;
    QString SolverDescription() const override;

    /// @brief 执行基于 Jacobian 的数值 IK 迭代。
    RoboSDP::Kinematics::Dto::IkResultDto SolveIk(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const RoboSDP::Kinematics::Dto::IkRequestDto& request) const override;

private:
    /// @brief 私有缓存状态，仅在 cpp 中定义，避免头文件暴露额外实现细节。
    struct NativeKernelState;

    /// @brief 同步 IK 所需的最小原生上下文准备状态。
    bool SyncNativeModel(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        QString& failureReason) const;

    /// @brief 若外部未提供完整 seed，则退回到软限位中点作为数值迭代起点。
    std::vector<double> BuildSeedJointPositionsDeg(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const RoboSDP::Kinematics::Dto::IkRequestDto& request) const;

    /// @brief 统一记录 IK 求解过程中的信息与告警日志。
    void LogMessage(
        RoboSDP::Logging::LogLevel level,
        const QString& actionName,
        const QString& message) const;

private:
    RoboSDP::Logging::ILogger* m_logger = nullptr;
    mutable std::unique_ptr<NativeKernelState> m_native_kernel_state;
};

} // namespace RoboSDP::Kinematics::Adapter
