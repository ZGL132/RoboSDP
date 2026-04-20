#pragma once

#include "core/errors/ErrorCode.h"
#include "core/logging/ILogger.h"
#include "modules/kinematics/adapter/IIkSolverAdapter.h"
#include "modules/kinematics/adapter/IKinematicBackendDiagnosticsAdapter.h"
#include "modules/kinematics/adapter/IKinematicBackendAdapter.h"
#include "modules/kinematics/dto/KinematicBackendBuildContextDto.h"
#include "modules/kinematics/dto/KinematicSolverResultDto.h"
#include "modules/kinematics/dto/UrdfPreviewSceneDto.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

#include <map>
#include <memory>

namespace RoboSDP::Kinematics::Service
{

/// @brief 从 Topology (拓扑) 构建 KinematicModel (运动学模型) 的结果包装体。
struct KinematicBuildResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok; // 错误码，默认成功
    QString message;                                                        // 附带的提示或错误信息
    QString topology_file_path;                                             // 依赖的拓扑文件路径
    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto state;            // 构建出的运动学工作区状态

    /// @brief 判断操作是否成功
    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// @brief Kinematics 数据持久化（保存）的结果包装体。
struct KinematicSaveResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString model_file_path;     // 模型保存的具体文件路径
    QString workspace_file_path; // 工作空间缓存保存的文件路径

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// @brief Kinematics 数据加载的结果包装体。
struct KinematicLoadResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString model_file_path;
    QString workspace_file_path;
    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto state; // 加载成功后恢复的状态数据

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// @brief URDF 骨架导入结果包装体。
struct UrdfImportResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString urdf_file_path;                                        // 导入的 URDF 原文件路径
    RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto preview_scene;   // 解析后生成的用于 3D 预览的场景数据
    /// @brief 预览链路对应的统一机器人模型 DTO，供 UI 高频姿态刷新时复用共享内核，不重新解析 URDF。
    RoboSDP::Kinematics::Dto::KinematicModelDto preview_model;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// @brief URDF 预览姿态高频刷新结果包装体。
struct PreviewPoseUpdateResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    /// @brief 以 link_name 为键保存共享内核 FK 输出的最新全局位姿，供 VTK Actor 快速更新矩阵。
    std::map<QString, RoboSDP::Kinematics::Dto::CartesianPoseDto> link_world_poses;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/**
 * @brief Kinematics 核心服务层类。
 *
 * @details 
 * 当前服务层主要负责以下职责：
 * 1. 业务流转：从 Topology 草稿构建最小 KinematicModel。
 * 2. 算法编排：提供正运动学 (FK)、逆运动学 (IK) 和工作空间采样的统一入口。
 * 3. 持久化：处理 Kinematic JSON 数据的保存与读取。
 * 4. 可视化辅助：生成 URDF 骨架导入后的中央三维预览数据。
 *
 * 架构设计特点：使用适配器模式 (Adapter Pattern) 隔离了底层的具体算法库（如自研 Legacy 或 Pinocchio），
 * 保证了服务层业务逻辑的稳定。
 */
class KinematicsService
{
public:
    /**
     * @brief 默认构造函数
     * @details 内部会自动装配当前默认的 Pinocchio 共享内核后端，
     * 适合在不需要复杂依赖注入的普通场景下直接实例化使用。
     * @param storage 运动学 JSON 存储组件引用
     * @param topologyStorage 拓扑结构 JSON 存储组件引用
     * @param logger 日志组件指针（可选）
     */
    KinematicsService(
        RoboSDP::Kinematics::Persistence::KinematicJsonStorage& storage,
        RoboSDP::Topology::Persistence::TopologyJsonStorage& topologyStorage,
        RoboSDP::Logging::ILogger* logger = nullptr);

    /**
     * @brief 依赖注入构造函数
     * @details 允许外部显式传入特定的运动学后端和逆解求解器适配器。
     * 这对于单元测试（Mock）或在不同 Pinocchio 配置下验证共享内核行为非常有用。
     * @param storage 运动学 JSON 存储组件
     * @param topologyStorage 拓扑结构 JSON 存储组件
     * @param backendAdapter 运动学核心后端适配器（负责 FK / Workspace）
     * @param ikSolverAdapter 逆运动学求解器适配器
     * @param logger 日志组件
     */
    KinematicsService(
        RoboSDP::Kinematics::Persistence::KinematicJsonStorage& storage,
        RoboSDP::Topology::Persistence::TopologyJsonStorage& topologyStorage,
        std::unique_ptr<RoboSDP::Kinematics::Adapter::IKinematicBackendAdapter> backendAdapter,
        std::unique_ptr<RoboSDP::Kinematics::Adapter::IIkSolverAdapter> ikSolverAdapter,
        RoboSDP::Logging::ILogger* logger);

    /// @brief 创建一个全默认属性的运动学模型 DTO
    RoboSDP::Kinematics::Dto::KinematicModelDto CreateDefaultModel() const;
    
    /// @brief 创建一个全默认属性的运动学工作区状态 DTO
    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto CreateDefaultState() const;

    /**
     * @brief 基于项目根目录中的拓扑 (Topology) 草稿构建初始运动学模型
     * @param projectRootPath 项目根目录路径
     * @return 构建结果，包含生成的模型数据
     */
    KinematicBuildResult BuildFromTopology(const QString& projectRootPath) const;

    /**
     * @brief 执行正运动学 (FK) 求解
     * @param model 当前的运动学模型参数
     * @param request 包含关节角度的请求参数
     * @return 包含末端位姿的 FK 结果
     */
    RoboSDP::Kinematics::Dto::FkResultDto SolveFk(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const RoboSDP::Kinematics::Dto::FkRequestDto& request) const;

    /**
     * @brief 执行逆运动学 (IK) 求解
     * @param model 当前的运动学模型参数
     * @param request 包含目标笛卡尔位姿的请求参数
     * @return 包含各关节所需角度的 IK 结果
     */
    RoboSDP::Kinematics::Dto::IkResultDto SolveIk(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const RoboSDP::Kinematics::Dto::IkRequestDto& request) const;

    /**
     * @brief 对机器人的工作空间进行点云采样
     * @param model 当前的运动学模型参数
     * @param request 包含采样密度和配置的请求参数
     * @return 包含空间点集的工作空间结果
     */
    RoboSDP::Kinematics::Dto::WorkspaceResultDto SampleWorkspace(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const RoboSDP::Kinematics::Dto::WorkspaceRequestDto& request) const;

    /**
     * @brief 检查后端构建上下文，主要用于诊断和调试
     * @details 探测底层算法库是否正确理解了传入的模型参数。
     * 如果后端实现了诊断接口，将返回底层构建出的实际数学模型语义。
     * @param model 待诊断的运动学模型
     * @return 诊断结果 DTO
     */
    RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto InspectBackendBuildContext(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const;

    /**
     * @brief 执行原生 Pinocchio FK 干跑诊断
     * @details 该方法只用于测试与后续调试，不改变 SolveFk 默认主链。
     * @param model 待诊断的运动学模型
     * @param joint_positions_deg 按 joint_order_signature 顺序传入的关节角，单位为度
     * @return base / flange / tcp 语义 frame 的轻量 FK 干跑结果
     */
    RoboSDP::Kinematics::Dto::NativeFkDryRunResultDto InspectNativeFkDryRun(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const std::vector<double>& joint_positions_deg) const;

    /**
     * @brief 执行原生 Pinocchio Jacobian 干跑诊断
     * @details 该方法只用于测试与后续调试，不改变 SolveFk / SolveIk / SampleWorkspace 默认主链。
     * @param model 待诊断的运动学模型
     * @param joint_positions_deg 按 joint_order_signature 顺序传入的关节角，单位为度
     * @return 6xN 空间雅可比矩阵的安全展平结果，不暴露 Eigen 或 Pinocchio 原生对象
     */
    RoboSDP::Kinematics::Dto::NativeJacobianDryRunResultDto InspectNativeJacobianDryRun(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const std::vector<double>& joint_positions_deg) const;

    /**
     * @brief 保存当前运动学草稿到本地 JSON
     * @param projectRootPath 项目根目录
     * @param state 需要保存的完整工作区状态
     * @return 保存操作结果
     */
    KinematicSaveResult SaveDraft(
        const QString& projectRootPath,
        const RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& state) const;

    /**
     * @brief 从本地 JSON 读取历史运动学草稿
     * @param projectRootPath 项目根目录
     * @return 加载操作结果及恢复的数据
     */
    KinematicLoadResult LoadDraft(const QString& projectRootPath) const;

    /**
     * @brief 导入 URDF 文件并提取用于 3D UI 显示的骨架预览数据
     * @details 预览场景统一来自 SharedRobotKernelRegistry + Pinocchio 零位 FK，
     * 保证中央 3D 火柴人视图与 FK / IK / Dynamics 使用同一份物理模型语义。
     * @param urdfFilePath 待解析的 URDF 绝对路径
     * @return 预览场景数据结果
     */
    UrdfImportResult ImportUrdfPreview(const QString& urdfFilePath) const;

    /**
     * @brief 高频刷新 URDF 预览中各 link 的全局位姿。
     * @details
     * 该接口只复用 SharedRobotKernelRegistry 中已经缓存的共享内核，
     * 不重新解析 URDF、不读取 Mesh 文件、不触发任何磁盘 I/O，专用于拖动关节时刷新 VTK Actor 矩阵。
     * @param model 已导入或已归一化的统一机器人模型 DTO
     * @param joint_positions_deg 按共享内核关节顺序传入的关节角，单位 deg
     * @return link_name -> 全局位姿 的轻量结果
     */
    PreviewPoseUpdateResult UpdatePreviewPoses(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const std::vector<double>& joint_positions_deg) const;

private:
    /**
     * @brief 核心业务方法：执行具体的从拓扑到运动学模型的数据映射
     * @param topologyModel 拓扑输入模型
     * @return 转换后的运动学模型 DTO
     */
    RoboSDP::Kinematics::Dto::KinematicModelDto BuildModelFromTopology(
        const RoboSDP::Topology::Dto::RobotTopologyModelDto& topologyModel) const;

private:
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage& m_storage;        // 运动学持久化组件
    RoboSDP::Topology::Persistence::TopologyJsonStorage& m_topology_storage;  // 拓扑持久化组件
    RoboSDP::Logging::ILogger* m_logger = nullptr;                            // 日志记录器

    /// @brief 负责 FK 和 Workspace 采样的底层算法引擎适配器
    std::unique_ptr<RoboSDP::Kinematics::Adapter::IKinematicBackendAdapter> m_backend_adapter;

    /// @brief 负责 IK 求解的底层算法引擎适配器
    std::unique_ptr<RoboSDP::Kinematics::Adapter::IIkSolverAdapter> m_ik_solver_adapter;
};

} // namespace RoboSDP::Kinematics::Service
