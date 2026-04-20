#pragma once

#include <QString>

#include <array>
#include <optional>
#include <vector>

namespace RoboSDP::Kinematics::Dto
{

/// @brief 笛卡尔位姿 DTO，统一使用平移 + RPY 欧拉角表达刚体坐标系。
struct CartesianPoseDto
{
    std::array<double, 3> position_m {0.0, 0.0, 0.0};
    std::array<double, 3> rpy_deg {0.0, 0.0, 0.0};
};

/// @brief 运动学模型元信息 DTO。
struct KinematicMetaDto
{
    QString kinematic_id = QStringLiteral("kinematic_001");
    QString name = QStringLiteral("6R 串联运动学模型");
    int version = 1;
    QString source = QStringLiteral("topology");
    QString status = QStringLiteral("draft");
    QString topology_ref;
    QString requirement_ref;
};

/// @brief 单节连杆的 DH/MDH 参数 DTO。
struct KinematicLinkParameterDto
{
    QString link_id;
    double a = 0.0;
    double alpha = 0.0;
    double d = 0.0;
    double theta_offset = 0.0;
};

/// @brief 关节软硬限位及动态约束 DTO。
struct KinematicJointLimitDto
{
    QString joint_id;
    std::array<double, 2> soft_limit {-180.0, 180.0};
    std::array<double, 2> hard_limit {-180.0, 180.0};
    double max_velocity = 180.0;
    double max_acceleration = 360.0;
};

/// @brief TCP 相对法兰坐标系的安装位姿 DTO。
struct TcpFrameDto
{
    std::array<double, 3> translation_m {0.0, 0.0, 0.1};
    std::array<double, 3> rpy_deg {0.0, 0.0, 0.0};
};

/// @brief IK 求解配置 DTO。
struct IkSolverConfigDto
{
    QString solver_type = QStringLiteral("numeric_jacobian_transpose");
    QString branch_policy = QStringLiteral("nearest_seed");
    int max_iterations = 200;
    double position_tolerance_mm = 1.0;
    double orientation_tolerance_deg = 1.0;
    double step_gain = 0.35;
};

/**
 * @brief KinematicModel 主 DTO。
 *
 * 第一阶段最小闭环保留以下核心字段：
 * 1. DH/MDH 参数；
 * 2. 基坐标系、法兰坐标系、工具/工件参考系、TCP 坐标系；
 * 3. 关节限位；
 * 4. IK 配置；
 * 5. 与 Requirement / Topology 的引用关系。
 */
struct KinematicModelDto
{
    KinematicMetaDto meta;

    /// @brief 当前模型采用的建模语义模式，用于区分 DH、MDH 还是 URDF 导入模型。
    QString modeling_mode = QStringLiteral("DH");

    /// @brief 指向统一机器人模型快照的逻辑引用，后续用于运动学与动力学共享同一内部模型语义。
    QString unified_robot_model_ref;

    /// @brief 记录当前模型来自手工种子、Topology 派生还是 URDF 导入，便于后续增量刷新与回溯来源。
    QString model_source_mode = QStringLiteral("manual_seed");

    /// @brief 记录当前模型保存时默认关联的运动学后端类型，用于业务诊断与后续切换策略。
    QString backend_type = QStringLiteral("pinocchio_kinematic_backend");

    QString parameter_convention = QStringLiteral("DH");
    int joint_count = 6;

    /// @brief 当建模模式为 URDF 时，记录原始 URDF 文件路径，便于重新导入和问题定位。
    QString urdf_source_path;

    /// @brief 标记共享 Pinocchio 内部模型是否已准备完成，防止上层流程误判后端已切换。
    bool pinocchio_model_ready = false;

    /// @brief 标记当前内部坐标系语义版本，后续用于兼容旧项目中的基座、法兰、TCP 解释口径。
    int frame_semantics_version = 1;

    /// @brief 记录当前关节顺序签名，后续用于 Jacobian、状态量映射和跨模块关节顺序一致性校验。
    QString joint_order_signature;

    /// @brief URDF 外部 Mesh 的搜索目录列表，用于解析 package://、相对路径或工作空间中的资源文件。
    std::vector<QString> mesh_search_directories;

    /// @brief 机器人基坐标系，相对项目世界坐标系定义。
    CartesianPoseDto base_frame;

    std::vector<KinematicLinkParameterDto> links;

    /// @brief 关节软硬限位和速度/加速度上限。
    std::vector<KinematicJointLimitDto> joint_limits;

    /// @brief 法兰坐标系，相对末端连杆坐标系定义。
    CartesianPoseDto flange_frame;

    /// @brief 工具坐标系，可选，通常用于规划场景或工艺定义。
    std::optional<CartesianPoseDto> tool_frame;

    /// @brief 工件坐标系，可选，通常用于规划场景或工艺定义。
    std::optional<CartesianPoseDto> workpiece_frame;

    /// @brief TCP 坐标系，相对法兰坐标系定义。
    TcpFrameDto tcp_frame;

    IkSolverConfigDto ik_solver_config;

    /// @brief 创建可直接进入编辑、FK/IK 与工作空间采样流程的默认 6R 串联模型。
    static KinematicModelDto CreateDefault()
    {
        KinematicModelDto dto;
        dto.links = {
            {QStringLiteral("link_1"), 0.0, 90.0, 0.35, 0.0},
            {QStringLiteral("link_2"), 0.30, 0.0, 0.0, 0.0},
            {QStringLiteral("link_3"), 0.25, 0.0, 0.0, 0.0},
            {QStringLiteral("link_4"), 0.0, 90.0, 0.12, 0.0},
            {QStringLiteral("link_5"), 0.0, -90.0, 0.10, 0.0},
            {QStringLiteral("link_6"), 0.0, 0.0, 0.08, 0.0}};

        dto.joint_limits = {
            {QStringLiteral("joint_1"), {-175.0, 175.0}, {-185.0, 185.0}, 180.0, 360.0},
            {QStringLiteral("joint_2"), {-130.0, 80.0}, {-135.0, 85.0}, 180.0, 360.0},
            {QStringLiteral("joint_3"), {-145.0, 145.0}, {-150.0, 150.0}, 180.0, 360.0},
            {QStringLiteral("joint_4"), {-190.0, 190.0}, {-200.0, 200.0}, 260.0, 520.0},
            {QStringLiteral("joint_5"), {-120.0, 120.0}, {-125.0, 125.0}, 260.0, 520.0},
            {QStringLiteral("joint_6"), {-350.0, 350.0}, {-360.0, 360.0}, 320.0, 640.0}};

        QString signature;
        for (std::size_t index = 0; index < dto.joint_limits.size(); ++index)
        {
            if (index > 0)
            {
                signature.append(QLatin1Char('|'));
            }
            signature.append(dto.joint_limits[index].joint_id);
        }
        dto.joint_order_signature = signature;

        return dto;
    }
};

} // namespace RoboSDP::Kinematics::Dto
