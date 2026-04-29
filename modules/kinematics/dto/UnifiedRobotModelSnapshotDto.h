#pragma once

#include <QString>

namespace RoboSDP::Kinematics::Dto
{

/**
 * @brief 统一工程主链快照 DTO。
 * @details
 * 该 DTO 不承载 Pinocchio / URDF 原生重对象，只保存跨模块共享所需的“主链摘要语义”，用于：
 * 1. 让 Dynamics / Planning 明确依赖的是哪一份统一机器人模型；
 * 2. 避免各模块重复猜测当前主模型模式和共享内核就绪状态；
 * 3. 为后续显式 DH -> 派生 URDF 主链提供稳定的持久化挂点。
 */
struct UnifiedRobotModelSnapshotDto
{
    /// @brief 快照结构版本号，用于后续平滑扩展统一工程主链描述字段。
    int snapshot_version = 1;

    /// @brief 统一机器人模型逻辑引用，后续可扩展为派生 URDF / 内核缓存键的稳定标识。
    QString unified_robot_model_ref;

    /// @brief 生成该快照时对应的运动学模型 ID，便于跨模块回溯来源。
    QString source_kinematic_id;

    /// @brief 当前生成该快照时的主模型类型：dh_mdh / urdf。
    QString master_model_type = QStringLiteral("dh_mdh");

    /// @brief 当前建模语义：DH / MDH / URDF。
    QString modeling_mode = QStringLiteral("DH");

    /// @brief 当前参数约定口径，便于下游模块显示和排查。
    QString parameter_convention = QStringLiteral("DH");

    /// @brief 当前共享运动学后端类型，第一阶段默认 pinocchio_kinematic_backend。
    QString backend_type = QStringLiteral("pinocchio_kinematic_backend");

    /// @brief 当前主链使用的关节顺序签名，用于跨模块状态量一致性校验。
    QString joint_order_signature;

    /// @brief 共享 Pinocchio 内核是否已经就绪。
    bool pinocchio_model_ready = false;

    /// @brief 当前 frame 语义版本，便于后续兼容旧项目。
    int frame_semantics_version = 1;

    /// @brief 生成该快照时的主模型来源模式：manual_seed / topology_derived / urdf_imported。
    QString model_source_mode = QStringLiteral("manual_seed");

    /// @brief 当前主链转换/同步摘要，供 UI 与日志直接展示。
    QString conversion_diagnostics;

    /// @brief 派生工程产物的相对项目路径。第二阶段先用于轻量描述“逻辑派生路径”。
    QString derived_artifact_relative_path;

    /// @brief 派生产物的轻量版本标识，便于下游判断是否与当前主链快照对齐。
    QString derived_artifact_version;

    /// @brief 派生产物最近一次生成的 UTC 时间。当前阶段先作为描述字段持久化。
    QString derived_artifact_generated_at_utc;

    /// @brief 派生产物状态码，例如 logical_only / external_master / external_missing。
    QString derived_artifact_state_code = QStringLiteral("missing");

    /// @brief 当前派生产物是否已经物理存在。
    bool derived_artifact_exists = false;

    /// @brief 当前派生产物是否可以认为与上游主链快照是新鲜对齐的。
    bool derived_artifact_fresh = false;
};

} // namespace RoboSDP::Kinematics::Dto
