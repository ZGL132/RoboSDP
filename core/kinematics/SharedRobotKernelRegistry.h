#pragma once

#include "modules/kinematics/dto/UrdfPreviewSceneDto.h"

#include <QString>

#include <memory>
#include <vector>

#if defined(ROBOSDP_HAVE_PINOCCHIO)
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/joint/joint-mimic.hpp>
#include <pinocchio/multibody/joint/joint-revolute.hpp>
#include <pinocchio/multibody/model.hpp>
#endif

namespace RoboSDP::Kinematics::Dto
{
struct KinematicModelDto;
}

namespace RoboSDP::Dynamics::Dto
{
struct DynamicModelDto;
}

namespace RoboSDP::Core::Kinematics
{

#if defined(ROBOSDP_HAVE_PINOCCHIO)
/**
 * @brief 共享机器人内核统一采用的最小 Pinocchio 关节集合。
 * @details
 * 这里继续沿用项目中已经验证过的 MinGW 友好接法：
 * 只保留 Revolute-Z 与 Composite/Mimic 组合，避免额外引入不稳定的模板实例化依赖。
 */
template<typename Scalar, int Options>
struct SharedRevoluteZJointCollectionTpl
{
    typedef pinocchio::JointModelRevoluteTpl<Scalar, Options, 2> JointModelRZ;
    typedef pinocchio::JointDataRevoluteTpl<Scalar, Options, 2> JointDataRZ;
    typedef pinocchio::JointModelCompositeTpl<Scalar, Options, SharedRevoluteZJointCollectionTpl>
        JointModelComposite;
    typedef pinocchio::JointDataCompositeTpl<Scalar, Options, SharedRevoluteZJointCollectionTpl>
        JointDataComposite;
    typedef pinocchio::JointModelMimicTpl<Scalar, Options, SharedRevoluteZJointCollectionTpl>
        JointModelMimic;
    typedef pinocchio::JointDataMimicTpl<Scalar, Options, SharedRevoluteZJointCollectionTpl>
        JointDataMimic;
    typedef boost::variant<
        JointModelRZ,
        boost::recursive_wrapper<JointModelComposite>,
        boost::recursive_wrapper<JointModelMimic>>
        JointModelVariant;
    typedef boost::variant<
        JointDataRZ,
        boost::recursive_wrapper<JointDataComposite>,
        boost::recursive_wrapper<JointDataMimic>>
        JointDataVariant;
};

/// @brief 共享注册表中缓存的不可变 Pinocchio Model 类型。
using SharedPinocchioModel = pinocchio::ModelTpl<double, 0, SharedRevoluteZJointCollectionTpl>;

/// @brief 适配器各自独立持有的 Pinocchio Data 类型。
using SharedPinocchioData = pinocchio::DataTpl<double, 0, SharedRevoluteZJointCollectionTpl>;

/// @brief 共享内核构建时使用的 Revolute-Z 关节模型类型。
using SharedJointModelRZ = pinocchio::JointModelRevoluteTpl<double, 0, 2>;
#else
struct SharedPinocchioModel;
struct SharedPinocchioData;
#endif

/**
 * @brief 共享机器人内核缓存键请求。
 * @details
 * 该请求对象只携带构建缓存键与建树所需的最小输入。
 * Registry 会基于 unified_robot_model_ref + modeling_mode + joint_order_signature + inertial_hash
 * 生成严格缓存键，并按需从 Kinematic/Dynamic DTO 中提取建树数据。
 */
struct SharedRobotKernelRequest
{
    const RoboSDP::Kinematics::Dto::KinematicModelDto* kinematic_model = nullptr;
    const RoboSDP::Dynamics::Dto::DynamicModelDto* dynamic_model = nullptr;
    QString unified_robot_model_ref;
    QString modeling_mode;
    QString joint_order_signature;
    QString inertial_hash;
    bool allow_structural_alias = false;
};

/**
 * @brief 供 URDF 骨架预览使用的节点语义元数据。
 * @details 该结构只保存 UI 组装预览场景所需的 link 名称与 frame_id，
 * Service 会在零位前向运动学后从 Data::oMf 中提取绝对坐标。
 */
struct SharedRobotPreviewNodeMetadata
{
    QString link_name;
    int frame_id = 0;
};

/**
 * @brief 供 URDF 骨架预览使用的线段语义元数据。
 * @details 该结构冻结 joint/link 命名和起止 frame_id 的映射关系，
 * 用于保证中央 3D 预览与共享物理内核读取的是同一份事实来源。
 */
struct SharedRobotPreviewSegmentMetadata
{
    QString joint_name;
    QString joint_type;
    QString parent_link_name;
    QString child_link_name;
    int start_frame_id = 0;
    int end_frame_id = 0;
};

/**
 * @brief 共享机器人内核的轻量元数据。
 * @details
 * Registry 只缓存对业务层有意义的安全摘要，不暴露三方原生对象。
 * 运动学和动力学适配器将复用这里的 frame 索引、关节偏移和诊断状态，
 * 同时各自独立维护自己的 Pinocchio Data。
 */
struct SharedRobotKernelMetadata
{
    QString cache_key;
    QString structural_key;
    QString inertial_hash;
    QString model_name;
    QString status_code;
    QString status_message;
    QString warning_message;
    bool shared_kernel_ready = false;
    bool contains_explicit_inertia = false;
    int base_frame_id = 0;
    int flange_frame_id = 0;
    int tcp_frame_id = 0;
    std::vector<int> link_frame_ids;
    std::vector<SharedRobotPreviewNodeMetadata> preview_nodes;
    std::vector<SharedRobotPreviewSegmentMetadata> preview_segments;
    std::vector<RoboSDP::Kinematics::Dto::GeometryObjectDto> visual_geometries;
    std::vector<RoboSDP::Kinematics::Dto::GeometryObjectDto> collision_geometries;
    std::vector<double> native_position_offsets_deg;
    int native_joint_count = 0;
    int native_frame_count = 0;
    int registry_build_count = 0;
};

/**
 * @brief Registry 获取或构建共享内核后的返回结果。
 * @details
 * 成功时返回共享的不可变 Model 与元数据；失败时仅通过 metadata.status_message 说明原因。
 */
struct SharedRobotKernelAcquireResult
{
    bool success = false;
    bool cache_hit = false;
    std::shared_ptr<const SharedPinocchioModel> model;
    SharedRobotKernelMetadata metadata;
};

/**
 * @brief 共享机器人内核注册表。
 * @details
 * 本类负责在 Kinematics / Dynamics 之间共享静态 Pinocchio Model，
 * 以消除双重建树（Dual-Tree Building）问题。
 * 注意：Registry 只缓存不可变 Model；绝不缓存 Pinocchio Data。
 */
class SharedRobotKernelRegistry final
{
public:
    static SharedRobotKernelRegistry& Instance();

    SharedRobotKernelAcquireResult GetOrBuildKernel(const SharedRobotKernelRequest& request);

private:
    SharedRobotKernelRegistry() = default;
};

} // namespace RoboSDP::Core::Kinematics
