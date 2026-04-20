#pragma once

#include "core/logging/ILogger.h"
#include "modules/dynamics/adapter/IDynamicsBackendAdapter.h"
#include "modules/dynamics/adapter/IDynamicsBackendDiagnosticsAdapter.h"
#include "modules/kinematics/adapter/PinocchioKinematicBackendAdapter.h"

#include <cstdint>
#include <memory>

namespace RoboSDP::Dynamics::Adapter
{

/**
 * @brief Pinocchio 动力学共享内核适配器。
 *
 * @details
 * 本类承担两类职责：
 * 1. 复用 Kinematics 模块已经冻结的统一机器人模型语义，确保动力学与运动学建模口径一致。
 * 2. 在 Dynamics 模块内部实例化原生 Pinocchio Model/Data，并提供 RNEA Dry-Run 诊断入口。
 *
 * 当前阶段仅接入诊断链路，不替换现有 Dynamics 主业务链。
 */
class PinocchioDynamicsBackendAdapter final
    : public IDynamicsBackendAdapter
    , public IDynamicsBackendDiagnosticsAdapter
{
public:
    explicit PinocchioDynamicsBackendAdapter(RoboSDP::Logging::ILogger* logger = nullptr);
    ~PinocchioDynamicsBackendAdapter() override;

    QString BackendId() const override;
    QString BackendDescription() const override;
    bool UsesSharedRobotKernel() const override;

    RoboSDP::Dynamics::Dto::InverseDynamicsTrajectoryResultDto Analyze(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& kinematicModel,
        const RoboSDP::Dynamics::Dto::DynamicModelDto& dynamicModel,
        const RoboSDP::Dynamics::Dto::ParameterizedTrajectoryDto& trajectory) const override;

    QString ExplainWhySharedKernelNotReady() const override;
    int GetNativeModelBuildCount() const override;

    /// @brief 仅供测试验证共享注册表复用是否生效，返回原生共享 Model 的数值地址。
    std::uintptr_t NativeModelAddressForDiagnostics() const;

    /// @brief 仅供测试验证 Data 是否按模块隔离，返回当前 adapter 私有 Data 的数值地址。
    std::uintptr_t NativeDataAddressForDiagnostics() const;

    RoboSDP::Dynamics::Dto::NativeRneaDryRunResultDto EvaluateNativeRneaDryRun(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& kinematicModel,
        const RoboSDP::Dynamics::Dto::DynamicModelDto& dynamicModel,
        const std::vector<double>& joint_positions_deg,
        const std::vector<double>& joint_velocities_deg_s,
        const std::vector<double>& joint_accelerations_deg_s2) const override;

private:
    /// @brief 原生 Pinocchio Model/Data 的私有状态，仅在 cpp 内定义，禁止暴露三方对象。
    struct NativeKernelState;

    /// @brief 同步 Dynamics 所需的原生模型缓存；失败时返回明确中文原因。
    bool SyncNativeModel(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& kinematicModel,
        const RoboSDP::Dynamics::Dto::DynamicModelDto& dynamicModel,
        QString& message) const;

private:
    RoboSDP::Logging::ILogger* m_logger = nullptr;
    /// @brief 复用 Kinematics 的 build-context 体检逻辑，确保两侧统一模型语义完全一致。
    mutable RoboSDP::Kinematics::Adapter::PinocchioKinematicBackendAdapter m_kinematics_backend_adapter;
    /// @brief Dynamics 内部独立持有的原生内核状态；第 10 阶段暂未与 Kinematics 共享同一指针。
    mutable std::unique_ptr<NativeKernelState> m_native_kernel_state;
};

} // namespace RoboSDP::Dynamics::Adapter
