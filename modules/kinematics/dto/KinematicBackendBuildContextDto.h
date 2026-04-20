#pragma once

#include "core/errors/ErrorCode.h"
#include "modules/kinematics/dto/KinematicModelDto.h"

#include <QString>

#include <vector>

namespace RoboSDP::Kinematics::Dto
{

/**
 * @brief 统一后的运动学模型语义摘要 DTO。
 *
 * 本 DTO 不直接保存 Pinocchio 原生对象，只负责把 KinematicModelDto
 * 中与 backend 构建相关的业务语义整理成稳定、可测试的轻量结果：
 * 1. 固定建模模式口径；
 * 2. 固定参数约定口径；
 * 3. 固定来源模式和目标 backend 类型；
 * 4. 固定关节顺序签名与 frame 语义版本。
 */
struct NormalizedKinematicModelSemanticDto
{
    /// @brief 归一后的建模模式，只允许 DH / MDH / URDF。
    QString modeling_mode;

    /// @brief 归一后的参数约定，对 URDF 语义占位模型统一标记为 URDF。
    QString parameter_convention;

    /// @brief 模型中声明的 backend 类型，用于回溯当前草稿保存时的默认主路径。
    QString declared_backend_type;

    /// @brief 本次 build-context 实际面向的 backend 类型，当前阶段固定指向 Pinocchio 诊断后端。
    QString normalized_backend_type;

    /// @brief 归一后的模型来源模式，固定为 manual_seed / topology_derived / urdf_imported / urdf_placeholder。
    QString model_source_mode;

    /// @brief 统一机器人模型逻辑引用，后续用于跨模块共享同一内部模型语义。
    QString unified_robot_model_ref;

    /// @brief 归一后的关节数量，用于后续 Pinocchio q 向量大小和 Jacobian 维度校验。
    int joint_count = 0;

    /// @brief 归一后的关节顺序签名，后续作为 joint_positions_deg 与共享内核顺序一致性的依据。
    QString joint_order_signature;

    /// @brief 当模式来源于 URDF 时保留原始文件路径；仅语义占位时允许为空。
    QString urdf_source_path;

    /// @brief 当前 frame 语义版本，用于保证 base / flange / tcp 的解释口径稳定。
    int frame_semantics_version = 1;

    /// @brief 标记当前 URDF 模式是否只是语义占位，尚未映射到真实 Pinocchio 模型。
    bool is_urdf_semantic_placeholder = false;
};

/// @brief 统一后的 frame 语义映射 DTO，用于冻结 base / flange / tcp 的业务角色。
struct FrameSemanticMappingDto
{
    QString base_frame_role;
    QString flange_frame_role;
    QString tcp_frame_role;
    bool semantics_complete = false;
};

/// @brief 统一后的关节顺序映射 DTO，用于描述 DTO 顺序到共享内核顺序的一致性。
struct JointOrderMappingDto
{
    QString signature;
    int expected_joint_count = 0;
    bool mapping_complete = false;
    std::vector<QString> ordered_joint_ids;
};

/// @brief backend build-context 构建状态 DTO，用于向 Service 与测试暴露轻量诊断结果。
struct KinematicBackendBuildStatusDto
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::UnknownError;
    QString status_code;
    QString status_message;
    bool normalized_semantics_ready = false;
    bool frame_semantics_ready = false;
    bool joint_order_ready = false;
    bool build_context_ready = false;
    bool shared_robot_kernel_ready = false;

    /// @brief 原生 Pinocchio 模型是否命中缓存，便于确认重复诊断不会反复构建大对象。
    bool native_model_cache_hit = false;

    /// @brief 当前 adapter 生命周期内真实构建 Pinocchio Model/Data 的累计次数。
    int native_model_build_count = 0;

    /// @brief 最近一次原生模型中的关节数量，用于调试 DTO 关节顺序与 Pinocchio 内部维度是否一致。
    int native_joint_count = 0;

    /// @brief 最近一次原生模型中的 frame 数量，用于确认 base / flange / tcp 语义 frame 已进入内核。
    int native_frame_count = 0;

    /// @brief 最近一次原生模型缓存键，不保存原生指针，只用于定位“为什么没有重建”。
    QString native_model_cache_key;
};

/// @brief backend build-context DTO，封装统一语义、frame 语义与关节顺序映射结果。
struct KinematicBackendBuildContextDto
{
    NormalizedKinematicModelSemanticDto normalized_model;
    FrameSemanticMappingDto frame_mapping;
    JointOrderMappingDto joint_order_mapping;
};

/// @brief build-context 调试结果 DTO，用于承载 adapter 对外暴露的状态摘要。
struct KinematicBackendBuildContextResultDto
{
    QString backend_id;
    QString backend_description;
    KinematicBackendBuildContextDto context;
    KinematicBackendBuildStatusDto status;

    bool IsSuccess() const
    {
        return status.error_code == RoboSDP::Errors::ErrorCode::Ok && status.build_context_ready;
    }
};

/**
 * @brief Pinocchio 原生 FK 干跑结果 DTO。
 *
 * @details
 * 本 DTO 只承载经过安全转换后的轻量结果，不保存 Eigen::VectorXd、
 * pinocchio::SE3 或任何原生指针。它用于验证 Pinocchio 内部 Model/Data
 * 是否能完成真实 forwardKinematics 与 frame placement 更新。
 */
struct NativeFkDryRunResultDto
{
    /// @brief 干跑是否成功；失败时 message 必须给出明确中文原因。
    bool success = false;

    /// @brief 中文状态信息，用于测试、日志和后续诊断面板显示。
    QString message;

    /// @brief 输入关节角度，单位为度，按当前 joint_order_signature 顺序保存。
    std::vector<double> joint_positions_deg;

    /// @brief 输入关节角度叠加零位偏移后的原生弧度值，使用标准容器暴露，不暴露 Eigen 向量。
    std::vector<double> joint_positions_rad;

    /// @brief Pinocchio 内部 base 语义 frame 的世界位姿。
    CartesianPoseDto base_pose;

    /// @brief Pinocchio 内部 flange 语义 frame 的世界位姿。
    CartesianPoseDto flange_pose;

    /// @brief Pinocchio 内部 tcp 语义 frame 的世界位姿。
    CartesianPoseDto tcp_pose;
};

/**
 * @brief Pinocchio 原生 Jacobian 干跑结果 DTO。
 *
 * @details
 * 该 DTO 只保存已经脱离 Eigen / Pinocchio 原生类型的轻量矩阵数据。
 * jacobian_matrix 使用行主序展平：matrix[row * cols + col] 对应 6xN 雅可比中的一个元素。
 * 本阶段只用于诊断验证，不进入默认 FK/IK/Workspace 主链。
 */
struct NativeJacobianDryRunResultDto
{
    /// @brief 干跑是否成功；失败时 message 必须说明明确中文原因。
    bool success = false;

    /// @brief 中文状态信息，用于测试、日志或后续诊断面板展示。
    QString message;

    /// @brief 空间雅可比矩阵行数，固定为 6，前三行为线速度分量，后三行为角速度分量。
    int rows = 6;

    /// @brief 空间雅可比矩阵列数，对应 Pinocchio 原生模型的 nv，也就是速度自由度数量。
    int cols = 0;

    /// @brief 参考坐标系口径说明；当前固定为 LOCAL_WORLD_ALIGNED，便于和世界系位姿诊断对齐。
    QString reference_frame;

    /// @brief 输入关节角，单位为度，顺序遵循 joint_order_signature。
    std::vector<double> joint_positions_deg;

    /// @brief 叠加零位偏移后的原生弧度输入，用标准容器暴露，避免暴露 Eigen::VectorXd。
    std::vector<double> joint_positions_rad;

    /// @brief 6xN Jacobian 的行主序展平数组，不暴露 Eigen::Matrix。
    std::vector<double> jacobian_matrix;
};

} // namespace RoboSDP::Kinematics::Dto
