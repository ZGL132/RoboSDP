#include "modules/kinematics/service/KinematicsService.h"

#include "core/kinematics/SharedRobotKernelRegistry.h"
#include "modules/kinematics/adapter/PinocchioIkSolverAdapter.h"
#include "modules/kinematics/adapter/PinocchioKinematicBackendAdapter.h"
#include "modules/topology/dto/TopologyRecommendationDto.h"

#include <QDir>
#include <QFileInfo>
#include <QSet>

#include <cmath>

// 条件编译：仅在 CMake 中探测到并开启了 Pinocchio 库时，引入其头文件。
// 这是为了保证在没有安装复杂物理引擎的精简环境下依然能够编译。
#if defined(ROBOSDP_HAVE_PINOCCHIO)
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#endif

namespace RoboSDP::Kinematics::Service
{

// 匿名命名空间，用于限制内部辅助函数的可见性，避免符号污染其他编译单元。
// 这些函数只在当前文件内部生效。
namespace
{
/**
 * @brief 根据硬限位计算推荐的软限位值
 * @param hardLimit 物理硬限位值（角度或位置）
 * @param isMinimum true 表示当前计算的是下限（最小值），false 表示上限（最大值）
 * @return 软限位值
 * @details 设计意图：软限位是控制系统软件层面的保护边界，必须比机械硬限位更保守。
 * 这里采用向内收缩 5 度的方式，为电机减速停机留出安全制动余量。
 */
double PreferredSoftLimit(double hardLimit, bool isMinimum)
{
    // 如果是最小值，向正方向收缩 +5；如果是最大值，向负方向收缩 -5
    return hardLimit + (isMinimum ? 5.0 : -5.0);
}

/**
 * @brief 模型参数合法性校验结果结构体，用于在验证流程中传递状态和错误信息
 */
struct ModelValidationResult
{
    bool success = true; // 默认校验通过
    QString message;     // 失败时的错误提示
};

/**
 * @brief 构造一个表示“校验失败”的结果对象的快捷工厂函数
 * @param message 错误信息描述
 * @return 初始化为失败状态的 ModelValidationResult
 */
ModelValidationResult MakeValidationFailure(const QString& message)
{
    return {false, message};
}

/**
 * @brief 检查浮点数是否为安全的有限数值
 * @param value 待检查的浮点数
 * @return 如果是有效数字返回 true，如果是 NaN (非数字) 或 Inf (无穷大) 返回 false
 * @details 在将数据传递给底层矩阵运算前，防止这些非法值导致程序崩溃
 */
bool IsFiniteValue(double value)
{
    return std::isfinite(value);
}

/**
 * @brief 校验单个笛卡尔坐标系位姿数据（位置和姿态）的合法性
 * @param frameName 坐标系名称，用于在报错信息中提示用户哪个坐标系出错
 * @param pose 待检查的位姿数据 DTO
 * @return 校验结果
 * @details 包含安全边界检查：
 * 1. 确保所有分量都是合法数字。
 * 2. 位置限制在 ±10m 内（基于常规工业机器人的合理物理尺寸）。
 * 3. 欧拉角限制在 ±360° 内。
 */
ModelValidationResult ValidatePoseFrame(
    const QString& frameName,
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    constexpr double kPositionLimitM = 10.0;  // 位置阈值 10米
    constexpr double kAngleLimitDeg = 360.0;  // 角度阈值 360度

    // 1. 校验 XYZ 平移分量
    for (std::size_t index = 0; index < pose.position_m.size(); ++index)
    {
        if (!IsFiniteValue(pose.position_m[index]))
        {
            return MakeValidationFailure(QStringLiteral("%1 的位置分量存在非数值内容。").arg(frameName));
        }
        if (std::abs(pose.position_m[index]) > kPositionLimitM)
        {
            return MakeValidationFailure(QStringLiteral("%1 的位置分量超出允许范围 ±%2 m。")
                                             .arg(frameName)
                                             .arg(kPositionLimitM, 0, 'f', 1));
        }
    }

    // 2. 校验 RPY (Roll, Pitch, Yaw) 旋转分量
    for (std::size_t index = 0; index < pose.rpy_deg.size(); ++index)
    {
        if (!IsFiniteValue(pose.rpy_deg[index]))
        {
            return MakeValidationFailure(QStringLiteral("%1 的姿态分量存在非数值内容。").arg(frameName));
        }
        if (std::abs(pose.rpy_deg[index]) > kAngleLimitDeg)
        {
            return MakeValidationFailure(QStringLiteral("%1 的姿态分量超出允许范围 ±%2 deg。")
                                             .arg(frameName)
                                             .arg(kAngleLimitDeg, 0, 'f', 0));
        }
    }

    return {}; // 返回默认成功状态
}

/**
 * @brief 校验工具中心点 (TCP) 坐标系的合法性
 * @param tcpFrame 待校验的 TCP 结构
 * @return 校验结果
 * @note 由于 TCP 的 DTO 结构定义不同（平移属性名为 translation_m），
 * 我们在此将其转换为标准的 CartesianPoseDto，以复用 ValidatePoseFrame 的校验逻辑。
 */
ModelValidationResult ValidateTcpFrame(const RoboSDP::Kinematics::Dto::TcpFrameDto& tcpFrame)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = tcpFrame.translation_m;
    pose.rpy_deg = tcpFrame.rpy_deg;
    return ValidatePoseFrame(QStringLiteral("TCP Frame"), pose);
}

/**
 * @brief 校验模型中所有关节的运动限位与物理约束配置
 * @param model 待校验的运动学模型
 * @return 校验结果
 * @details 
 * 该函数执行一系列严格的逻辑一致性检查，防呆防错：
 * 1. 数量对齐检查（限位数组大小 == 连杆数量 == 关节数）。
 * 2. 标识符排重。
 * 3. 动力学约束（速度、加速度必须 > 0）。
 * 4. 边界合法性（软硬限位中 Min 必须 <= Max）。
 * 5. 安全包裹（软限位区间必须严格在硬限位区间内部）。
 */
ModelValidationResult ValidateJointLimits(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    if (model.links.empty())
    {
        return MakeValidationFailure(QStringLiteral("links 为空，无法校验 joint_limits。"));
    }

    if (model.joint_count != static_cast<int>(model.links.size()))
    {
        return MakeValidationFailure(QStringLiteral("joint_count 必须与 links 数量一致。"));
    }

    if (model.joint_limits.size() != model.links.size())
    {
        return MakeValidationFailure(QStringLiteral("joint_limits 数量必须与 links 数量一致。"));
    }

    QSet<QString> jointIds; // 借助 QSet 用于查重 joint_id
    for (std::size_t index = 0; index < model.joint_limits.size(); ++index)
    {
        const auto& limit = model.joint_limits[index];
        const QString displayIndex = QString::number(static_cast<int>(index) + 1);

        // 检查 ID 是否为空
        if (limit.joint_id.trimmed().isEmpty())
        {
            return MakeValidationFailure(QStringLiteral("第 %1 个 joint_limits 条目缺少 joint_id。").arg(displayIndex));
        }

        // 检查 ID 是否重复
        if (jointIds.contains(limit.joint_id))
        {
            return MakeValidationFailure(QStringLiteral("joint_limits 中存在重复的 joint_id：%1。").arg(limit.joint_id));
        }
        jointIds.insert(limit.joint_id);

        // 检查限制数组中的值是否是有效数字
        for (double value : limit.soft_limit)
        {
            if (!IsFiniteValue(value)) return MakeValidationFailure(QStringLiteral("关节 %1 的 soft_limit 存在非数值内容。").arg(limit.joint_id));
        }
        for (double value : limit.hard_limit)
        {
            if (!IsFiniteValue(value)) return MakeValidationFailure(QStringLiteral("关节 %1 的 hard_limit 存在非数值内容。").arg(limit.joint_id));
        }

        // 动力学约束检查：速度、加速度不能为负数或0
        if (!IsFiniteValue(limit.max_velocity) || limit.max_velocity <= 0.0)
        {
            return MakeValidationFailure(QStringLiteral("关节 %1 的 max_velocity 必须大于 0。").arg(limit.joint_id));
        }
        if (!IsFiniteValue(limit.max_acceleration) || limit.max_acceleration <= 0.0)
        {
            return MakeValidationFailure(QStringLiteral("关节 %1 的 max_acceleration 必须大于 0。").arg(limit.joint_id));
        }

        // 范围合法性检查：区间的最小值 (index 0) 必须 <= 最大值 (index 1)
        if (limit.soft_limit[0] > limit.soft_limit[1])
        {
            return MakeValidationFailure(QStringLiteral("关节 %1 的 soft_limit 最小值不能大于最大值。").arg(limit.joint_id));
        }
        if (limit.hard_limit[0] > limit.hard_limit[1])
        {
            return MakeValidationFailure(QStringLiteral("关节 %1 的 hard_limit 最小值不能大于最大值。").arg(limit.joint_id));
        }

        // 包含关系检查：软限位区间必须是硬限位区间的子集
        if (limit.soft_limit[0] < limit.hard_limit[0] || limit.soft_limit[1] > limit.hard_limit[1])
        {
            return MakeValidationFailure(QStringLiteral("关节 %1 的 soft_limit 必须落在 hard_limit 范围内。").arg(limit.joint_id));
        }
    }

    return {};
}

/**
 * @brief 对整个运动学模型执行完整深度校验的总入口
 * @param model 待校验的运动学模型
 * @return 校验结果
 * @details 按照坐标系顺序，依次校验基础系、法兰系、工具系、工件系、TCP，以及所有关节限制。
 * 只有通过此校验的模型才能被送入底层的算法求解器 (FK/IK) 或保存至磁盘。
 */
ModelValidationResult ValidateModel(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    // 校验参数约定类型
    if (model.parameter_convention != QStringLiteral("DH") &&
        model.parameter_convention != QStringLiteral("MDH"))
    {
        return MakeValidationFailure(QStringLiteral("parameter_convention 仅支持 DH 或 MDH。"));
    }

    // 级联校验所有的关键坐标系
    auto validation = ValidatePoseFrame(QStringLiteral("Base Frame"), model.base_frame);
    if (!validation.success) return validation;

    validation = ValidatePoseFrame(QStringLiteral("Flange Frame"), model.flange_frame);
    if (!validation.success) return validation;

    // 工具和工件系是可选的，只有配置了才去校验
    if (model.tool_frame.has_value())
    {
        validation = ValidatePoseFrame(QStringLiteral("Tool Frame"), model.tool_frame.value());
        if (!validation.success) return validation;
    }

    if (model.workpiece_frame.has_value())
    {
        validation = ValidatePoseFrame(QStringLiteral("Workpiece Frame"), model.workpiece_frame.value());
        if (!validation.success) return validation;
    }

    validation = ValidateTcpFrame(model.tcp_frame);
    if (!validation.success) return validation;

    // 最后校验关节限制的业务逻辑
    return ValidateJointLimits(model);
}

#if defined(ROBOSDP_HAVE_PINOCCHIO)
/**
 * @brief 将 Pinocchio 的三维平移向量转换为预览 DTO 使用的标准数组。
 */
std::array<double, 3> ToPreviewPosition(const Eigen::Vector3d& translation)
{
    return {translation[0], translation[1], translation[2]};
}

/**
 * @brief 将业务层角度单位 deg 转换为 Pinocchio 使用的 rad。
 */
double PreviewDegToRad(double degrees)
{
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    return degrees * kDegToRad;
}

/**
 * @brief 将旋转矩阵安全转换为预览链路使用的 RPY 欧拉角（单位：度）。
 * @details
 * 这里沿用项目中 pose -> SE3 的 yaw * pitch * roll 约定，
 * 因此反解时按 ZYX 顺序提取欧拉角，再回填为 roll/pitch/yaw。
 */
std::array<double, 3> ToPreviewRpyDeg(const Eigen::Matrix3d& rotation)
{
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    const Eigen::Vector3d yawPitchRoll = rotation.eulerAngles(2, 1, 0);
    return {
        yawPitchRoll[2] * kRadToDeg,
        yawPitchRoll[1] * kRadToDeg,
        yawPitchRoll[0] * kRadToDeg};
}

/**
 * @brief 将 Pinocchio 的 frame 位姿转换为预览节点使用的轻量全局位姿 DTO。
 * @details 该 DTO 后续会被 VTK Mesh 渲染链路复用，保证“骨架”和“皮肤”读取同一份零位事实。
 */
RoboSDP::Kinematics::Dto::CartesianPoseDto ToPreviewPose(const pinocchio::SE3& placement)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = ToPreviewPosition(placement.translation());
    pose.rpy_deg = ToPreviewRpyDeg(placement.rotation());
    return pose;
}

/**
 * @brief 为 URDF 预览链路生成最小统一机器人模型 DTO。
 * @details
 * 预览链路只关心共享内核建树所需的最小语义：
 * modeling_mode / source_path / 缓存键。
 * 这里显式将 flange 与 TCP 置零，保证最后一个 URDF link 节点与 Dry-Run TCP 位姿保持一致。
 */
RoboSDP::Kinematics::Dto::KinematicModelDto BuildUrdfPreviewModel(const QString& absoluteUrdfPath)
{
    RoboSDP::Kinematics::Dto::KinematicModelDto model;
    const QFileInfo urdfFileInfo(absoluteUrdfPath);
    model.meta.kinematic_id = QStringLiteral("urdf_preview_%1").arg(urdfFileInfo.completeBaseName());
    model.meta.name = urdfFileInfo.completeBaseName();
    model.meta.source = QStringLiteral("urdf_preview");
    model.modeling_mode = QStringLiteral("URDF");
    model.parameter_convention = QStringLiteral("URDF");
    model.model_source_mode = QStringLiteral("urdf_imported");
    model.backend_type = QStringLiteral("pinocchio_kinematic_backend");
    model.unified_robot_model_ref = QStringLiteral("urdf_preview::%1").arg(absoluteUrdfPath);
    model.joint_order_signature = absoluteUrdfPath;
    model.urdf_source_path = absoluteUrdfPath;
    model.mesh_search_directories = {
        urdfFileInfo.absolutePath(),
        QDir(urdfFileInfo.absolutePath()).absoluteFilePath(QStringLiteral(".."))};
    model.joint_count = 0;
    model.links.clear();
    model.joint_limits.clear();
    model.flange_frame = {};
    model.tcp_frame.translation_m = {0.0, 0.0, 0.0};
    model.tcp_frame.rpy_deg = {0.0, 0.0, 0.0};
    return model;
}

/**
 * @brief 从共享物理内核直接生成 URDF 骨架预览。
 * @details
 * 这是中央 3D 火柴人视图的唯一合法数据来源：
 * 1. 从 SharedRobotKernelRegistry 获取统一 Pinocchio Model；
 * 2. 在 neutral(q) 零位上执行 FK；
 * 3. 按 preview_nodes / preview_segments 元数据组装 DTO。
 */
UrdfImportResult ImportUrdfPreviewWithSharedKernel(
    const QString& urdfFilePath,
    RoboSDP::Logging::ILogger* logger)
{
    UrdfImportResult result;
    result.urdf_file_path = QFileInfo(urdfFilePath).absoluteFilePath();
    result.preview_scene.urdf_file_path = result.urdf_file_path;

    if (urdfFilePath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("URDF 文件路径不能为空。");
        return result;
    }

    const QFileInfo urdfFileInfo(urdfFilePath);
    if (!urdfFileInfo.exists() || !urdfFileInfo.isFile())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::IoError;
        result.message = QStringLiteral("URDF 文件不存在：%1").arg(urdfFileInfo.absoluteFilePath());
        return result;
    }

    auto previewModel = BuildUrdfPreviewModel(urdfFileInfo.absoluteFilePath());
    RoboSDP::Core::Kinematics::SharedRobotKernelRequest request;
    request.kinematic_model = &previewModel;
    request.unified_robot_model_ref = previewModel.unified_robot_model_ref;
    request.modeling_mode = previewModel.modeling_mode;
    request.joint_order_signature = previewModel.joint_order_signature;
    request.allow_structural_alias = false;

    const auto acquireResult =
        RoboSDP::Core::Kinematics::SharedRobotKernelRegistry::Instance().GetOrBuildKernel(request);
    if (!acquireResult.success || acquireResult.model == nullptr)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = acquireResult.metadata.status_message.trimmed().isEmpty()
            ? QStringLiteral("共享 Pinocchio 内核未能生成 URDF 预览模型。")
            : acquireResult.metadata.status_message;
        return result;
    }

    try
    {
        RoboSDP::Core::Kinematics::SharedPinocchioData data(*acquireResult.model);
        // 当前共享内核只承载 Revolute-Z 关节族；对这组关节来说，全零 q 与 neutral(q) 数学等价。
        // 这里显式构造零位配置，避免 MinGW + 自定义 JointCollection 下 neutral 模板实例化失败。
        const Eigen::VectorXd zeroConfiguration =
            Eigen::VectorXd::Zero(static_cast<Eigen::Index>(acquireResult.model->nq));
        pinocchio::forwardKinematics(*acquireResult.model, data, zeroConfiguration);
        pinocchio::updateFramePlacements(*acquireResult.model, data);

        result.preview_scene.model_name = acquireResult.metadata.model_name.trimmed().isEmpty()
            ? QString::fromStdString(acquireResult.model->name)
            : acquireResult.metadata.model_name.trimmed();
        result.preview_scene.visual_geometries = acquireResult.metadata.visual_geometries;
        result.preview_scene.collision_geometries = acquireResult.metadata.collision_geometries;
        // 中文说明：把本次导入对应的统一模型 DTO 一并返回给 UI，
        // 后续拖动关节只需调用 UpdatePreviewPoses，不再重新解析 URDF 或 Mesh。
        previewModel.joint_count = static_cast<int>(acquireResult.model->nq);
        result.preview_model = previewModel;
        if (result.preview_scene.model_name.trimmed().isEmpty())
        {
            result.preview_scene.model_name = urdfFileInfo.completeBaseName();
        }

        for (const auto& previewNode : acquireResult.metadata.preview_nodes)
        {
            if (previewNode.frame_id < 0 ||
                previewNode.frame_id >= static_cast<int>(acquireResult.model->nframes))
            {
                result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
                result.message = QStringLiteral("共享 Pinocchio 内核返回了非法的预览节点 frame_id=%1。")
                    .arg(previewNode.frame_id);
                return result;
            }

            RoboSDP::Kinematics::Dto::UrdfPreviewNodeDto node;
            node.link_name = previewNode.link_name;
            node.world_pose = ToPreviewPose(data.oMf[static_cast<std::size_t>(previewNode.frame_id)]);
            node.position_m = node.world_pose.position_m;
            result.preview_scene.nodes.push_back(node);
        }

        for (const auto& previewSegment : acquireResult.metadata.preview_segments)
        {
            if (previewSegment.start_frame_id < 0 ||
                previewSegment.start_frame_id >= static_cast<int>(acquireResult.model->nframes) ||
                previewSegment.end_frame_id < 0 ||
                previewSegment.end_frame_id >= static_cast<int>(acquireResult.model->nframes))
            {
                result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
                result.message = QStringLiteral(
                    "共享 Pinocchio 内核返回了非法的预览线段 frame_id=%1 -> %2。")
                    .arg(previewSegment.start_frame_id)
                    .arg(previewSegment.end_frame_id);
                return result;
            }

            RoboSDP::Kinematics::Dto::UrdfPreviewSegmentDto segment;
            segment.joint_name = previewSegment.joint_name;
            segment.joint_type = previewSegment.joint_type;
            segment.parent_link_name = previewSegment.parent_link_name;
            segment.child_link_name = previewSegment.child_link_name;
            segment.joint_axis_xyz = previewSegment.joint_axis_xyz;
            segment.start_position_m =
                ToPreviewPosition(data.oMf[static_cast<std::size_t>(previewSegment.start_frame_id)].translation());
            segment.end_position_m =
                ToPreviewPosition(data.oMf[static_cast<std::size_t>(previewSegment.end_frame_id)].translation());
            result.preview_scene.segments.push_back(segment);
        }

        if (result.preview_scene.IsEmpty())
        {
            result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
            result.message = QStringLiteral("共享 Pinocchio 内核已解析 URDF，但未提取到可预览的骨架节点。");
            return result;
        }

        result.message = QStringLiteral("Pinocchio 共享内核 URDF 导入成功：模型 %1，节点 %2，连杆段 %3。")
            .arg(result.preview_scene.model_name)
            .arg(result.preview_scene.nodes.size())
            .arg(result.preview_scene.segments.size());
        if (!acquireResult.metadata.warning_message.trimmed().isEmpty())
        {
            result.message.append(QStringLiteral(" 提示：%1").arg(acquireResult.metadata.warning_message.trimmed()));
            if (logger != nullptr)
            {
                logger->Log(
                    RoboSDP::Logging::LogLevel::Warning,
                    QStringLiteral("[Kinematics] URDF 预览存在兼容性提示：%1")
                        .arg(acquireResult.metadata.warning_message.trimmed()));
            }
        }
        return result;
    }
    catch (const std::exception& exception)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
        result.message = QStringLiteral("共享 Pinocchio 内核生成 URDF 预览异常：%1")
            .arg(QString::fromUtf8(exception.what()));
        return result;
    }
    catch (...)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
        result.message = QStringLiteral("共享 Pinocchio 内核生成 URDF 预览时出现未知异常。");
        return result;
    }
}
#endif

} // namespace

// =================================================================================
// KinematicsService 成员方法实现区域
// =================================================================================

/**
 * @brief 普通构造函数（委托调用）
 * @details 当调用者没有提供具体的 Adapter 适配器时，通过委托构造函数（Delegating Constructor）
 * 将纯 Pinocchio 的默认实现装配进入依赖注入路径。
 * Stage 16 之后 FK / IK / Workspace 都统一走共享内核，不再保留 Stage1Legacy 回退链。
 */
KinematicsService::KinematicsService(
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage& storage,
    RoboSDP::Topology::Persistence::TopologyJsonStorage& topologyStorage,
    RoboSDP::Logging::ILogger* logger)
    : KinematicsService( 
          storage,
          topologyStorage,
          std::make_unique<RoboSDP::Kinematics::Adapter::PinocchioKinematicBackendAdapter>(logger), 
          std::make_unique<RoboSDP::Kinematics::Adapter::PinocchioIkSolverAdapter>(logger),         
          logger)
{
}

/**
 * @brief 完整参数的依赖注入构造函数
 * @details 负责装载各类依赖项到成员变量。
 */
KinematicsService::KinematicsService(
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage& storage,
    RoboSDP::Topology::Persistence::TopologyJsonStorage& topologyStorage,
    std::unique_ptr<RoboSDP::Kinematics::Adapter::IKinematicBackendAdapter> backendAdapter,
    std::unique_ptr<RoboSDP::Kinematics::Adapter::IIkSolverAdapter> ikSolverAdapter,
    RoboSDP::Logging::ILogger* logger)
    : m_storage(storage)
    , m_topology_storage(topologyStorage)
    , m_logger(logger)
    , m_backend_adapter(std::move(backendAdapter))
    , m_ik_solver_adapter(std::move(ikSolverAdapter))
{
    // 如果由于某种原因（比如误操作）外部注入了 nullptr，我们在此兜住默认装配，
    // 保证当前业务主链始终由 Pinocchio 后端负责。
    if (m_backend_adapter == nullptr)
    {
        m_backend_adapter = std::make_unique<RoboSDP::Kinematics::Adapter::PinocchioKinematicBackendAdapter>(logger);
    }

    if (m_ik_solver_adapter == nullptr)
    {
        // IK 默认也统一装配 Pinocchio 数值求解器，避免出现历史后端回流。
        m_ik_solver_adapter = std::make_unique<RoboSDP::Kinematics::Adapter::PinocchioIkSolverAdapter>(logger);
    }
}

/**
 * @brief 代理生成空模型 DTO
 */
RoboSDP::Kinematics::Dto::KinematicModelDto KinematicsService::CreateDefaultModel() const
{
    return RoboSDP::Kinematics::Dto::KinematicModelDto::CreateDefault();
}

/**
 * @brief 代理生成空状态 DTO
 */
RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto KinematicsService::CreateDefaultState() const
{
    return RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto::CreateDefault();
}

/**
 * @brief 基于指定的项目目录，加载之前的拓扑(Topology)数据来构建运动学初稿
 * @param projectRootPath 项目根目录路径
 * @return 构造成功的相关状态包裹体
 */
KinematicBuildResult KinematicsService::BuildFromTopology(const QString& projectRootPath) const
{
    KinematicBuildResult result;
    result.state = CreateDefaultState();
    result.topology_file_path = m_topology_storage.BuildAbsoluteFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法从 Topology 构建 Kinematics。");
        return result;
    }

    // 1. 读取前置模块 (Topology) 生成的草稿状态
    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto topologyState;
    result.error_code = m_topology_storage.Load(projectRootPath, topologyState);
    if (result.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.message = QStringLiteral("读取 Topology 草稿失败，请先保存 Topology：%1").arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        return result;
    }

    // 2. 检查模型的约束条件（当前业务逻辑只支持特定的 6R 串联机器人）
    if (topologyState.current_model.robot_definition.robot_type != QStringLiteral("6R_serial") ||
        topologyState.current_model.robot_definition.joint_count != 6)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("当前 Kinematics 最小实现仅支持模板化 6R 串联构型。");
        return result;
    }

    // 3. 执行核心业务映射，构建具体的模型 DTO
    result.state.current_model = BuildModelFromTopology(topologyState.current_model);
    result.message = QStringLiteral("已基于 Topology 生成最小 KinematicModel。");
    return result;
}

/**
 * @brief 执行正运动学计算服务
 * @param model 目标机器人模型参数
 * @param request 计算请求参数（需包含待求角度）
 * @return 计算结果
 */
RoboSDP::Kinematics::Dto::FkResultDto KinematicsService::SolveFk(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const RoboSDP::Kinematics::Dto::FkRequestDto& request) const
{
    // 在下发到底层 C++ 算法库或物理引擎前，执行严格的数据防呆校验
    const auto validation = ValidateModel(model);
    if (!validation.success)
    {
        RoboSDP::Kinematics::Dto::FkResultDto result;
        result.joint_positions_deg = request.joint_positions_deg; // 保持原样返回以匹配请求
        result.message = validation.message; // 报告不通过的原因
        return result; // 校验失败直接提前返回，切断下发途径保护底层安全
    }

    // 校验通过，安全地代理调用适配器的对应计算方法
    return m_backend_adapter->SolveFk(model, request);
}

/**
 * @brief 执行逆运动学计算服务
 * @param model 目标机器人模型参数
 * @param request 计算请求参数（需包含待求的末端笛卡尔位姿）
 * @return 计算结果
 */
RoboSDP::Kinematics::Dto::IkResultDto KinematicsService::SolveIk(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const RoboSDP::Kinematics::Dto::IkRequestDto& request) const
{
    // 同样执行严格模型拦截
    const auto validation = ValidateModel(model);
    if (!validation.success)
    {
        RoboSDP::Kinematics::Dto::IkResultDto result;
        result.message = validation.message;
        return result;
    }

    // 交给独立的 IK Solver 适配器去解决（因为逆解算法往往不同于前向的普通框架）
    return m_ik_solver_adapter->SolveIk(model, request);
}

/**
 * @brief 触发工作空间采样云的生成服务
 * @param model 目标模型
 * @param request 采样请求
 * @return 带有大量 3D 点坐标的结果包
 */
RoboSDP::Kinematics::Dto::WorkspaceResultDto KinematicsService::SampleWorkspace(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const RoboSDP::Kinematics::Dto::WorkspaceRequestDto& request) const
{
    const auto validation = ValidateModel(model);
    if (!validation.success)
    {
        RoboSDP::Kinematics::Dto::WorkspaceResultDto result;
        result.message = validation.message;
        result.requested_sample_count = request.sample_count;
        return result;
    }

    return m_backend_adapter->SampleWorkspace(model, request);
}

/**
 * @brief 检测并返回底层构建的真实运动学语义状态
 * @details 用于验证上层 UI 参数在底层数学库中是否被正确解析
 */
RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto
KinematicsService::InspectBackendBuildContext(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const
{
    RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto result;
    // 获取正在生效的底层提供商信息
    result.backend_id = m_backend_adapter != nullptr ? m_backend_adapter->BackendId() : QStringLiteral("backend_unavailable");
    result.backend_description = m_backend_adapter != nullptr ? m_backend_adapter->BackendDescription() : QStringLiteral("当前未注入运动学 backend。");

    if (m_backend_adapter == nullptr)
    {
        result.status.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
        result.status.status_code = QStringLiteral("backend_missing");
        result.status.status_message = QStringLiteral("当前未注入运动学 backend，无法执行 build-context 校验。");
        return result;
    }

    // C++ RTTI: 尝试将基类指针向下安全转型为具备诊断能力的特定接口
    // 并不是所有的引擎后端都有诊断功能，如果无法转型 (得到 nullptr)，说明不支持。
    const auto* diagnosticsAdapter = dynamic_cast<const RoboSDP::Kinematics::Adapter::IKinematicBackendDiagnosticsAdapter*>(m_backend_adapter.get());
    if (diagnosticsAdapter == nullptr)
    {
        result.status.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
        result.status.status_code = QStringLiteral("backend_diagnostics_unavailable");
        result.status.status_message = QStringLiteral("当前 backend=%1 未实现 build-context 诊断接口，无法输出统一机器人模型语义映射结果。").arg(result.backend_id);
        return result;
    }

    // 支持诊断，返回底层映射结果
    return diagnosticsAdapter->BuildNormalizedContext(model);
}

/**
 * @brief 透传 Pinocchio 原生 FK 干跑诊断
 * @details 该入口不调用 ValidateModel，也不参与 SolveFk 主链选择；
 * 它只确认诊断型 backend 是否能安全执行底层 FK 数学调用。
 */
RoboSDP::Kinematics::Dto::NativeFkDryRunResultDto
KinematicsService::InspectNativeFkDryRun(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const std::vector<double>& joint_positions_deg) const
{
    RoboSDP::Kinematics::Dto::NativeFkDryRunResultDto result;
    result.joint_positions_deg = joint_positions_deg;

    if (m_backend_adapter == nullptr)
    {
        result.message = QStringLiteral("当前未注入运动学 backend，无法执行 Pinocchio FK 干跑。");
        return result;
    }

    const auto* diagnosticsAdapter =
        dynamic_cast<const RoboSDP::Kinematics::Adapter::IKinematicBackendDiagnosticsAdapter*>(m_backend_adapter.get());
    if (diagnosticsAdapter == nullptr)
    {
        result.message = QStringLiteral("当前 backend=%1 未实现 FK 干跑诊断接口。")
            .arg(m_backend_adapter->BackendId());
        return result;
    }

    return diagnosticsAdapter->EvaluateNativeFkDryRun(model, joint_positions_deg);
}

/**
 * @brief 透传 Pinocchio 原生 Jacobian 干跑诊断
 * @details 该入口只服务于测试和诊断；用于查看当前 Pinocchio 共享内核的 build-context 健康度。
 */
RoboSDP::Kinematics::Dto::NativeJacobianDryRunResultDto
KinematicsService::InspectNativeJacobianDryRun(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const std::vector<double>& joint_positions_deg) const
{
    RoboSDP::Kinematics::Dto::NativeJacobianDryRunResultDto result;
    result.joint_positions_deg = joint_positions_deg;

    if (m_backend_adapter == nullptr)
    {
        result.message = QStringLiteral("当前未注入运动学 backend，无法执行 Pinocchio Jacobian 干跑。");
        return result;
    }

    const auto* diagnosticsAdapter =
        dynamic_cast<const RoboSDP::Kinematics::Adapter::IKinematicBackendDiagnosticsAdapter*>(m_backend_adapter.get());
    if (diagnosticsAdapter == nullptr)
    {
        result.message = QStringLiteral("当前 backend=%1 未实现 Jacobian 干跑诊断接口。")
            .arg(m_backend_adapter->BackendId());
        return result;
    }

    return diagnosticsAdapter->EvaluateNativeJacobianDryRun(model, joint_positions_deg);
}

/**
 * @brief 将服务状态进行持久化（存盘到 JSON）
 * @param projectRootPath 项目目录
 * @param state 当前包含所有内容的内存状态
 * @return 存盘结果
 */
KinematicSaveResult KinematicsService::SaveDraft(
    const QString& projectRootPath,
    const RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& state) const
{
    KinematicSaveResult result;
    result.model_file_path = m_storage.BuildAbsoluteModelFilePath(projectRootPath);
    result.workspace_file_path = m_storage.BuildAbsoluteWorkspaceFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法保存 Kinematics 草稿。");
        return result;
    }

    // 保存前极其重要的拦截：拒绝将错误的、非法的脏数据写入磁盘污染项目
    const auto validation = ValidateModel(state.current_model);
    if (!validation.success)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = validation.message;
        return result;
    }

    // 先保存核心的模型信息
    result.error_code = m_storage.SaveModel(projectRootPath, state);
    if (result.error_code == RoboSDP::Errors::ErrorCode::Ok)
    {
        // 只有核心模型确保安全落盘后，才去保存庞大的工作区点云采样缓存
        result.error_code = m_storage.SaveWorkspaceCache(projectRootPath, state.last_workspace_result);
    }

    // 格式化友好的反馈信息
    result.message = result.IsSuccess()
        ? QStringLiteral("Kinematics 草稿已保存到：%1").arg(result.model_file_path)
        : QStringLiteral("Kinematics 草稿保存失败：%1").arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
    return result;
}

/**
 * @brief 从项目目录下读取历史保存的 JSON 恢复工作状态
 * @param projectRootPath 项目目录
 * @return 读取恢复的结果包
 */
KinematicLoadResult KinematicsService::LoadDraft(const QString& projectRootPath) const
{
    KinematicLoadResult result;
    result.model_file_path = m_storage.BuildAbsoluteModelFilePath(projectRootPath);
    result.workspace_file_path = m_storage.BuildAbsoluteWorkspaceFilePath(projectRootPath);
    result.state = CreateDefaultState();

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法加载 Kinematics 草稿。");
        return result;
    }

    // 调用持久化层读取文件内容反序列化进 state
    result.error_code = m_storage.LoadModel(projectRootPath, result.state);
    if (result.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.message = QStringLiteral("Kinematics 加载失败：%1").arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        return result;
    }

    // 安全防御点：即使从本地读上来的模型也必须校验，
    // 因为外部可能存在手动使用文本编辑器篡改 JSON 注入恶意/非法参数的行为。
    const auto validation = ValidateModel(result.state.current_model);
    if (!validation.success)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = validation.message;
        return result;
    }

    // 尝试加载可能存在的采样点云缓存
    RoboSDP::Kinematics::Dto::WorkspaceResultDto workspaceResult;
    const RoboSDP::Errors::ErrorCode workspaceLoadError = m_storage.LoadWorkspaceCache(projectRootPath, workspaceResult);
    if (workspaceLoadError == RoboSDP::Errors::ErrorCode::Ok)
    {
        result.state.last_workspace_result = workspaceResult;
        result.message = QStringLiteral("Kinematics 已从 JSON 重新加载。");
    }
    else
    {
        // 容错处理：采样缓存较大或可丢弃，它的缺失不应该阻断核心模型的正常加载
        result.message = QStringLiteral("Kinematics 模型已加载，工作空间缓存未找到，已保留默认空结果。");
    }

    return result;
}

/**
 * @brief 根据文件路径智能导入 URDF 并生成预览数据
 * @details 预览数据统一来自 SharedRobotKernelRegistry，不再保留 Qt XML 手工解析分支。
 * @param urdfFilePath 待读取文件
 * @return 解析结果
 */
UrdfImportResult KinematicsService::ImportUrdfPreview(const QString& urdfFilePath) const
{
#if defined(ROBOSDP_HAVE_PINOCCHIO)
    return ImportUrdfPreviewWithSharedKernel(urdfFilePath, m_logger);
#else
    UrdfImportResult result;
    result.urdf_file_path = QFileInfo(urdfFilePath).absoluteFilePath();
    result.preview_scene.urdf_file_path = result.urdf_file_path;
    result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
    result.message = QStringLiteral("当前构建未启用 Pinocchio，共享物理内核不可用，无法导入 URDF 预览。");
    return result;
#endif
}

/**
 * @brief 高频刷新 URDF 预览 link 全局位姿。
 * @details
 * 该方法只复用 SharedRobotKernelRegistry 中的共享 Pinocchio Model，
 * 执行 FK 并提取 preview_nodes 对应 frame 的绝对位姿。
 * 整个流程不读取 Mesh/URDF 文件，避免拖动关节时发生磁盘 I/O。
 */
PreviewPoseUpdateResult KinematicsService::UpdatePreviewPoses(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const std::vector<double>& joint_positions_deg) const
{
    PreviewPoseUpdateResult result;

#if defined(ROBOSDP_HAVE_PINOCCHIO)
    RoboSDP::Core::Kinematics::SharedRobotKernelRequest request;
    request.kinematic_model = &model;
    request.unified_robot_model_ref = model.unified_robot_model_ref;
    request.modeling_mode = model.modeling_mode;
    request.joint_order_signature = model.joint_order_signature;
    request.allow_structural_alias = true;

    const auto acquireResult =
        RoboSDP::Core::Kinematics::SharedRobotKernelRegistry::Instance().GetOrBuildKernel(request);
    if (!acquireResult.success || acquireResult.model == nullptr)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = acquireResult.metadata.status_message.trimmed().isEmpty()
            ? QStringLiteral("预览姿态刷新失败：共享 Pinocchio 内核不可用。")
            : acquireResult.metadata.status_message;
        return result;
    }

    const auto expectedJointCount = static_cast<std::size_t>(acquireResult.model->nq);
    if (joint_positions_deg.size() != expectedJointCount)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("预览姿态刷新失败：输入关节角数量=%1，但共享内核 nq=%2。")
            .arg(joint_positions_deg.size())
            .arg(expectedJointCount);
        return result;
    }

    if (acquireResult.metadata.native_position_offsets_deg.size() != expectedJointCount)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("预览姿态刷新失败：内部零位偏移数量=%1，但共享内核 nq=%2。")
            .arg(acquireResult.metadata.native_position_offsets_deg.size())
            .arg(expectedJointCount);
        return result;
    }

    try
    {
        RoboSDP::Core::Kinematics::SharedPinocchioData data(*acquireResult.model);
        Eigen::VectorXd q = Eigen::VectorXd::Zero(acquireResult.model->nq);
        for (std::size_t index = 0; index < joint_positions_deg.size(); ++index)
        {
            if (!IsFiniteValue(joint_positions_deg[index]))
            {
                result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
                result.message = QStringLiteral("预览姿态刷新失败：第 %1 个关节角不是有效数值。").arg(index + 1);
                return result;
            }

            // 中文说明：DTO 输入为 deg，Pinocchio q 使用 rad；DH/MDH 零位偏移也在这里统一叠加。
            const double nativeDegrees =
                joint_positions_deg[index] + acquireResult.metadata.native_position_offsets_deg[index];
            q[static_cast<Eigen::Index>(index)] = PreviewDegToRad(nativeDegrees);
        }

        pinocchio::forwardKinematics(*acquireResult.model, data, q);
        pinocchio::updateFramePlacements(*acquireResult.model, data);

        for (const auto& previewNode : acquireResult.metadata.preview_nodes)
        {
            if (previewNode.frame_id < 0 ||
                previewNode.frame_id >= static_cast<int>(acquireResult.model->nframes))
            {
                result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
                result.message = QStringLiteral("预览姿态刷新失败：非法 frame_id=%1。").arg(previewNode.frame_id);
                return result;
            }

            result.link_world_poses[previewNode.link_name] =
                ToPreviewPose(data.oMf[static_cast<std::size_t>(previewNode.frame_id)]);
        }

        result.message = QStringLiteral("预览姿态刷新完成：已更新 %1 个 link 全局位姿。")
            .arg(result.link_world_poses.size());
        return result;
    }
    catch (const std::exception& exception)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
        result.message = QStringLiteral("预览姿态刷新异常：%1").arg(QString::fromUtf8(exception.what()));
        return result;
    }
    catch (...)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
        result.message = QStringLiteral("预览姿态刷新出现未知异常，已拦截以避免崩溃。");
        return result;
    }
#else
    Q_UNUSED(model);
    Q_UNUSED(joint_positions_deg);
    result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
    result.message = QStringLiteral("当前构建未启用 Pinocchio，无法刷新 URDF 预览姿态。");
    return result;
#endif
}

/**
 * @brief 核心业务映射：将抽象拓扑模型具体化为数学运动学模型
 * @param topologyModel 由系统上游提供的抽象机械结构描述
 * @return 细化的运动学表达对象 DTO
 */
RoboSDP::Kinematics::Dto::KinematicModelDto KinematicsService::BuildModelFromTopology(
    const RoboSDP::Topology::Dto::RobotTopologyModelDto& topologyModel) const
{
    using namespace RoboSDP::Kinematics::Dto;

    KinematicModelDto model = KinematicModelDto::CreateDefault();
    
    // 1. 同步并继承和映射上游带来的元数据信息
    model.meta.kinematic_id = QStringLiteral("kinematic_%1").arg(topologyModel.meta.topology_id);
    model.meta.name = QStringLiteral("%1 运动学模型").arg(topologyModel.meta.name);
    model.meta.source = QStringLiteral("topology");
    model.meta.status = QStringLiteral("ready");
    model.meta.topology_ref = topologyModel.meta.topology_id; // 确立追踪关联
    model.meta.requirement_ref = topologyModel.meta.requirement_ref;
    
    // 初始化后端引擎配置与状态标记
    model.modeling_mode = model.parameter_convention;
    model.unified_robot_model_ref.clear();
    model.model_source_mode = QStringLiteral("topology_derived");
    model.backend_type = QStringLiteral("pinocchio_kinematic_backend");
    model.urdf_source_path.clear();
    model.pinocchio_model_ready = false;
    model.frame_semantics_version = 1;
    model.joint_count = topologyModel.robot_definition.joint_count;

    // 2. 将物理规格落实为实际的几何参数映射
    // 根据拓扑中的底座高度设定基坐标系 BaseFrame 的 Z 轴垂直物理偏移
    model.base_frame.position_m[2] = topologyModel.robot_definition.base_height_m;

    // 业务特性体现：根据是否需要"空心手腕"(管线内置)结构调整 DH 参数。
    // 这说明特殊的机械构造会影响到数学骨架中连杆长度 (d) 的数值。
    if (topologyModel.layout.hollow_wrist_required)
    {
        model.links[4].d = 0.12;
        model.links[5].d = 0.10;
        model.tcp_frame.translation_m[2] = 0.12; // 同时补偿 TCP
    }

    // 安装姿态补偿：根据底座挂载类型，在世界基坐标系中硬性注入预偏转角
    if (topologyModel.robot_definition.base_mount_type == QStringLiteral("wall")) // 壁挂式
    {
        model.base_frame.rpy_deg[1] = 90.0; // 绕 Y 轴转 90 度
    }
    else if (topologyModel.robot_definition.base_mount_type == QStringLiteral("ceiling")) // 倒挂/吊装式
    {
        model.base_frame.rpy_deg[0] = 180.0; // 绕 X 轴翻转 180 度
    }

    // 3. 构建并计算关节动力学限制参数
    model.joint_limits.clear();
    for (const auto& topologyJoint : topologyModel.joints)
    {
        KinematicJointLimitDto limit;
        limit.joint_id = topologyJoint.joint_id;
        limit.hard_limit = topologyJoint.motion_range_deg; // 机械硬止挡位
        // 调用我们先前编写的辅助函数，动态计算带有退让缓冲区的控制软件限位
        limit.soft_limit = {
            PreferredSoftLimit(topologyJoint.motion_range_deg[0], true),
            PreferredSoftLimit(topologyJoint.motion_range_deg[1], false)};
            
        // 经验公式法则：根据所在轴位置给不同关节赋予默认的速度/加速度限值。
        // （通常靠近末端的手腕轴惯量小，允许运行得更快；底座重轴相对较慢）
        limit.max_velocity = topologyJoint.axis_index >= 4 ? 260.0 : 180.0;
        limit.max_acceleration = topologyJoint.axis_index >= 4 ? 520.0 : 360.0;
        model.joint_limits.push_back(limit);
    }

    // 4. 将关节的顺序拼接成防篡改字符串签名，
    // 用于确保序列化传输后的顺序与映射验证时高度一致
    QString jointOrderSignature;
    for (std::size_t index = 0; index < model.joint_limits.size(); ++index)
    {
        if (index > 0)
        {
            jointOrderSignature.append(QLatin1Char('|'));
        }
        jointOrderSignature.append(model.joint_limits[index].joint_id);
    }
    model.joint_order_signature = jointOrderSignature;

    return model;
}

} // namespace RoboSDP::Kinematics::Service
