#pragma once

#include "core/errors/ErrorCode.h"
#include "core/kinematics/SharedRobotKernelRegistry.h"
#include "core/logging/ILogger.h"
#include "modules/kinematics/adapter/IKinematicBackendAdapter.h"
#include "modules/kinematics/adapter/IKinematicBackendDiagnosticsAdapter.h"

#include <cstdint>
#include <memory>

namespace RoboSDP::Kinematics::Adapter
{

/**
 * @brief Pinocchio 运动学后端适配器。
 *
 * @details
 * 本类当前承担“双通道”职责：
 * 1. 归一化：把外部松散的 KinematicModelDto 整理成标准统一的 build-context（构建上下文）。
 * 2. 规则确立：把关节顺序、坐标系意义、模型来源等解释规则固定下来，防呆防错。
 * 3. 主路径：当 Pinocchio 原生 Model/Data 可用时，承担 FK 主链计算。
 * 4. 失败透明：当 Pinocchio 构建失败、输入不兼容或底层抛出异常时，立即返回明确错误，避免隐藏问题。
 */
class PinocchioKinematicBackendAdapter final
    : public IKinematicBackendAdapter
    , public IKinematicBackendDiagnosticsAdapter
{
public:
    /**
     * @brief 构造函数
     * @param logger 日志组件指针，用于记录调用痕迹
     */
    explicit PinocchioKinematicBackendAdapter(RoboSDP::Logging::ILogger* logger = nullptr);
    ~PinocchioKinematicBackendAdapter() override;

    /// @brief 获取当前构建阶段的状态简码 (如 "not_built_yet", "building_context")
    QString CurrentBuildStatusCode(const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const;

    /// @brief 获取当前构建阶段的详细中文说明
    QString CurrentBuildStatusMessage(const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const;

    /// @brief 实现接口：返回后端身份标识
    QString BackendId() const override;
    
    /// @brief 实现接口：返回后端的人类可读描述
    QString BackendDescription() const override;
    
    /// @brief 实现接口：是否已接入真正的共享机器人内核
    bool UsesSharedRobotKernel() const override;

    /**
     * @brief 执行正向运动学 (FK) 求解
     * @note 当前已是纯 Pinocchio 主链；若主路径失败，则直接返回明确错误。
     */
    RoboSDP::Kinematics::Dto::FkResultDto SolveFk(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const RoboSDP::Kinematics::Dto::FkRequestDto& request) const override;

    /**
     * @brief 执行工作空间采样
     * @note 当前已是纯 Pinocchio 主链；若主路径失败，则直接返回明确错误。
     */
    RoboSDP::Kinematics::Dto::WorkspaceResultDto SampleWorkspace(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const RoboSDP::Kinematics::Dto::WorkspaceRequestDto& request) const override;

    /// @brief 执行体检，看看传入的模型参数能不能满足构建 Pinocchio 引擎的要求
    RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto ValidateBuildContext(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const override;

    /// @brief 深度体检，并将松散的模型 DTO 翻译、整理成底层的标准归一化上下文
    RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto BuildNormalizedContext(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const override;

    /// @brief 执行 Pinocchio 原生 FK 干跑，仅用于诊断验证，不接入 SolveFk 主链
    RoboSDP::Kinematics::Dto::NativeFkDryRunResultDto EvaluateNativeFkDryRun(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const std::vector<double>& joint_positions_deg) const override;

    /// @brief 执行 Pinocchio 原生 Jacobian 干跑，仅用于诊断验证，不接入默认 FK/IK 主链
    RoboSDP::Kinematics::Dto::NativeJacobianDryRunResultDto EvaluateNativeJacobianDryRun(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const std::vector<double>& joint_positions_deg) const override;

    /// @brief 获取上一次体检（诊断）的最终结论状态
    RoboSDP::Kinematics::Dto::KinematicBackendBuildStatusDto GetLastBuildStatus() const override;

    /// @brief 用中文向外界解释：为什么当前的 Pinocchio 物理内核还没准备好？
    QString ExplainWhySharedKernelNotReady() const override;

    /// @brief 仅供测试验证共享注册表复用是否生效，返回原生共享 Model 的数值地址。
    std::uintptr_t NativeModelAddressForDiagnostics() const;

    /// @brief 仅供测试验证 Data 是否按适配器隔离，返回当前 adapter 私有 Data 的数值地址。
    std::uintptr_t NativeDataAddressForDiagnostics() const;

private:
    /**
     * @brief 统一机器人模型构建上下文 (内部数据结构)
     * @details 这个结构体就像是一张“清洗后的干净表单”。
     * 把上层五花八门的建模模式（DH/MDH/URDF）、后端类型、文件路径等，全部标准化记录在这里。
     * 未来真正调用 Pinocchio API 时，只看这张表单即可。
     */
    struct UnifiedRobotModelBuildContext
    {
        QString modeling_mode;                // 建模模式 (DH/MDH/URDF)
        QString parameter_convention;         // 参数约定
        QString declared_backend_type;        // 声明的后端类型
        QString normalized_backend_type;      // 实际生效的后端类型
        QString model_source_mode;            // 模型来源模式 (手工种子/拓扑衍生/URDF导入)
        QString unified_robot_model_ref;      // 统一模型参考ID
        QString urdf_source_path;             // URDF 文件路径 (如果有)
        int joint_count = 0;                  // 关节总数
        int frame_semantics_version = 1;      // 坐标系语义版本
        QString joint_order_signature;        // 关节顺序签名 (用于防篡改)
        bool is_urdf_semantic_placeholder = false; // 是否只是个URDF的占位符
    };

    /**
     * @brief 坐标系语义映射摘要
     * @details 明确机器人的 Base（基座）、Flange（法兰）、TCP（工具端点）在未来物理引擎中扮演的固定角色，
     * 防止物理引擎中的 Frame ID 与业务层的理解发生错位。
     */
    struct FrameSemanticMapping
    {
        QString base_frame_role;
        QString flange_frame_role;
        QString tcp_frame_role;
        bool semantics_ready = false;
    };

    /**
     * @brief 关节顺序映射摘要
     * @details 锁定传入的各个关节角度数组（joint_positions_deg）到底对应机器人的哪一个物理轴。
     * 如果顺序乱了，机器人的动作就会完全错乱（比如想动大臂，结果动了手腕）。
     */
    struct JointOrderMapping
    {
        QString signature;                   // 顺序防篡改签名
        int expected_joint_count = 0;        // 期望的关节数量
        bool order_ready = false;            // 映射是否准备就绪
        std::vector<QString> ordered_joint_ids; // 排好序的关节 ID 列表
    };

    /**
     * @brief 后端构建状态摘要
     * @details 记录“安检”过程中各个环节的通过情况。
     * 明确区分“上下文准备好了(build_context_ready)” 和 “物理内核真准备好了(shared_kernel_ready)”。
     */
    struct BackendBuildStatus
    {
        RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::UnknownError;
        QString status_code;
        QString status_message;
        QString warning_message;                  // 记录非阻断性预警，例如 URDF mesh 暂不支持
        bool normalized_semantics_ready = false; // 基本语义归一化是否完成
        bool frame_semantics_ready = false;      // 坐标系映射是否完成
        bool joint_order_ready = false;          // 关节顺序映射是否完成
        bool build_context_ready = false;        // 所有上下文是否都准备完毕
        bool shared_kernel_ready = false;        // 真实的 Pinocchio 内核是否可用
        bool native_model_cache_hit = false;     // 是否复用了上一次原生模型实例
        int native_model_build_count = 0;        // 原生模型累计真实构建次数
        int native_joint_count = 0;              // 原生 Pinocchio 模型关节数量
        int native_frame_count = 0;              // 原生 Pinocchio 模型 frame 数量
        QString native_model_cache_key;          // 原生模型缓存键，仅用于诊断
    };

    /**
     * @brief 原生内核构建结果摘要。
     * @details 该结构只记录构建是否成功、是否命中缓存以及公开可诊断的轻量数字，
     * 不向 Service/UI 暴露 pinocchio::Model 或 pinocchio::Data 指针。
     */
    struct NativeBuildSummary
    {
        bool success = false;
        bool cache_hit = false;
        int build_count = 0;
        int native_joint_count = 0;
        int native_frame_count = 0;
        QString cache_key;
        QString status_code;
        QString status_message;
        QString warning_message;
    };

    /// @brief 原生 Pinocchio Model/Data 的私有封装，定义在 cpp 中以隔离三方库头文件。
    struct NativeKernelState;

    /// @brief 执行归一化与校验的核心主流程
    RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto BuildContextInternal(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const;

    /// @brief 将 DTO 归一化为 UnifiedRobotModelBuildContext 结构
    UnifiedRobotModelBuildContext BuildUnifiedRobotModelContext(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        BackendBuildStatus& status) const;

    /// @brief 提取并校验坐标系（Frame）的安全性和合理性
    FrameSemanticMapping BuildFrameSemanticMapping(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        BackendBuildStatus& status) const;

    /// @brief 校验关节数组的数量和防篡改签名是否完全匹配
    JointOrderMapping BuildJointOrderMapping(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const UnifiedRobotModelBuildContext& buildContext,
        BackendBuildStatus& status) const;

    /// @brief 综合所有检查环节，给出最终的“体检报告”状态
    BackendBuildStatus EvaluateBuildStatus(
        const UnifiedRobotModelBuildContext& buildContext,
        const FrameSemanticMapping& frameMapping,
        const JointOrderMapping& jointOrderMapping,
        const NativeBuildSummary& nativeBuildSummary,
        const BackendBuildStatus& currentStatus) const;

    /// @brief 将清洗后的 build-context 同步成原生 Pinocchio Model/Data，失败时返回中文原因。
    NativeBuildSummary SyncNativeModel(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const UnifiedRobotModelBuildContext& buildContext,
        const FrameSemanticMapping& frameMapping,
        const JointOrderMapping& jointOrderMapping) const;

    /// @brief 内部结构转外部安全结构：向外部展示经过清洗和整理后的干净数据
    RoboSDP::Kinematics::Dto::KinematicBackendBuildContextDto ToPublicBuildContext(
        const UnifiedRobotModelBuildContext& buildContext,
        const FrameSemanticMapping& frameMapping,
        const JointOrderMapping& jointOrderMapping) const;

    /// @brief 内部结构转外部安全结构：向外部展示状态报告
    RoboSDP::Kinematics::Dto::KinematicBackendBuildStatusDto ToPublicBuildStatus(
        const BackendBuildStatus& status) const;

    /// @brief 在日志中记录“占位符方法”被空调用的情况，方便后续追踪
    void LogStubInvocation(const QString& actionName, const QString& message) const;

private:
    RoboSDP::Logging::ILogger* m_logger = nullptr;
    // 使用 mutable 关键字，允许在 const 方法（只读方法）中更新这个“最后一次的诊断结果”缓存
    mutable RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto m_last_build_context_result;
    /// @brief 原生 Pinocchio 内核状态，隐藏真实 Model/Data，确保外层无法误用三方对象。
    mutable std::unique_ptr<NativeKernelState> m_native_kernel_state;
};

} // namespace RoboSDP::Kinematics::Adapter
