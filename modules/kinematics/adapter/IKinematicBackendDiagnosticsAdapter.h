#pragma once

#include "modules/kinematics/dto/KinematicBackendBuildContextDto.h"
#include "modules/kinematics/dto/KinematicModelDto.h"

namespace RoboSDP::Kinematics::Adapter
{

/**
 * @brief 运动学底层引擎(Backend)的专门诊断与体检接口。
 *
 * @details
 * 这是一个非常聪明的设计，属于“高级调试探针”。它不参与常规的 FK/IK 计算，只用于“检查身体”：
 * 1. ValidateBuildContext：校验上层参数是否能正常被底层引擎解析。
 * 2. EvaluateNativeFkDryRun 和 EvaluateNativeJacobianDryRun：不通过业务主链，直接在底层执行干跑（Dry-Run）测试，输出 base/flange/tcp 位姿及空间雅可比矩阵。
 * 3. SampleWorkspaceWithSingularity 和 ComputeJacobianAnalysis：执行带奇异区识别的工作空间分析。
 * 4. 这种设计在不暴露 Eigen/Pinocchio 原生指针的前提下，为界面和排错提供了直观的“底层透视”能力。
 */
class IKinematicBackendDiagnosticsAdapter
{
public:
    /**
     * @brief 虚析构函数，保证多态安全。
     */
    virtual ~IKinematicBackendDiagnosticsAdapter() = default;

    /**
     * @brief 模拟构建底层的上下文环境，执行干跑 (Dry-Run) 校验
     * @param model 待校验的运动学模型
     * @return 返回轻量级的状态结果，包含成功与否以及中文的报错原因
     * @details 就像编译器的“语法检查”功能，不真正运行，只看看你给的参数能不能建得起模型。
     */
    virtual RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto ValidateBuildContext(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const = 0;

    /**
     * @brief 提取底层引擎深度解析后的归一化上下文结果
     * @param model 原始运动学模型
     * @return 经过引擎翻译、消化后生成的标准树状拓扑结构数据
     * @details 当发现界面显示的模型和底层算的对不上时，调用这个函数能“透视”底层究竟长什么样，是绝佳的调试入口。
     */
    virtual RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto BuildNormalizedContext(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const = 0;

    /**
     * @brief 执行 Pinocchio 原生 FK 干跑验证，但不接入业务主链
     * @param model 待验证的运动学模型
     * @param joint_positions_deg 按 joint_order_signature 顺序传入的关节角度，单位为度
     * @return 轻量 FK 干跑结果，包含 base/flange/tcp 三个语义 frame 的世界位姿
     * @details 该接口只用于诊断和测试：它会真实调用底层 forwardKinematics，
     * 但不会替代 KinematicsService::SolveFk，也不会向外暴露 Eigen 或 Pinocchio 原生对象。
     */
    virtual RoboSDP::Kinematics::Dto::NativeFkDryRunResultDto EvaluateNativeFkDryRun(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const std::vector<double>& joint_positions_deg) const = 0;

    /**
     * @brief 执行 Pinocchio 原生 Jacobian 干跑验证，但不接入业务主链
     * @param model 待验证的运动学模型
     * @param joint_positions_deg 按 joint_order_signature 顺序传入的关节角度，单位为度
     * @return 安全展平后的 6xN 空间雅可比矩阵，矩阵数据使用行主序 std::vector<double> 承载
     * @details 当前固定使用 Pinocchio 的 LOCAL_WORLD_ALIGNED 参考系：
     * Jacobian 的作用点位于 TCP frame 原点，线速度/角速度分量投影在世界坐标轴方向。
     * 该接口只用于诊断与测试，不暴露 Eigen::Matrix 或 pinocchio 原生对象。
     */
    virtual RoboSDP::Kinematics::Dto::NativeJacobianDryRunResultDto EvaluateNativeJacobianDryRun(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const std::vector<double>& joint_positions_deg) const = 0;

    /**
     * @brief 执行带奇异区识别的工作空间采样
     * @param model 运动学模型
     * @param request 采样配置（包含采样数量和条件数阈值）
     * @return 每个采样点附带条件数和可操作度的工作空间结果
     */
    virtual RoboSDP::Kinematics::Dto::WorkspaceResultDto SampleWorkspaceWithSingularity(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const RoboSDP::Kinematics::Dto::SingularityAnalysisRequestDto& request) const = 0;

    /**
     * @brief 执行 Jacobian 分析：计算奇异值、条件数、Yoshikawa 可操作度
     * @param model 运动学模型
     * @param joint_positions_deg 关节角度（度）
     * @return Jacobian 分析结果
     */
    virtual RoboSDP::Kinematics::Dto::JacobianAnalysisDto ComputeJacobianAnalysis(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const std::vector<double>& joint_positions_deg) const = 0;

    /**
     * @brief 获取底层引擎最后一次尝试构建模型时的”遗言”或状态摘要
     * @return 最近一次操作的状态 DTO
     * @details 如果系统崩溃或者初始化失败，这个接口能调出最近一次的”黑匣子”记录，方便后续排查。
     */
    virtual RoboSDP::Kinematics::Dto::KinematicBackendBuildStatusDto GetLastBuildStatus() const = 0;

    /**
     * @brief 获取底层共享内核无法就绪的具体原因
     * @return 明确的中文报错或说明字符串（例如："未安装 Pinocchio 库"，或 "连杆参数矩阵奇异"）
     * @details 这是一个非常人性化的设计，当 `UsesSharedRobotKernel()` 返回 false 时，用这个函数告诉开发者“为什么不行”。
     */
    virtual QString ExplainWhySharedKernelNotReady() const = 0;
};

} // namespace RoboSDP::Kinematics::Adapter
