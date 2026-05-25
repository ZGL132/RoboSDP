#include "modules/kinematics/service/KinematicsService.h"
#include "modules/kinematics/service/KinematicsServiceInternal.h"

#include <QDir>
#include <QFileInfo>
#include <QSaveFile>
#include <QTextStream>

#if defined(ROBOSDP_HAVE_PINOCCHIO)
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#endif

namespace RoboSDP::Kinematics::Service
{

using namespace Internal;

namespace
{
pinocchio::SE3 BuildArtifactPoseSe3(const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    const Eigen::AngleAxisd roll(PreviewDegToRad(pose.rpy_deg[0]), Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(PreviewDegToRad(pose.rpy_deg[1]), Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(PreviewDegToRad(pose.rpy_deg[2]), Eigen::Vector3d::UnitZ());
    const Eigen::Matrix3d rotation = (yaw * pitch * roll).toRotationMatrix();
    const Eigen::Vector3d translation(pose.position_m[0], pose.position_m[1], pose.position_m[2]);
    return pinocchio::SE3(rotation, translation);
}

/**
 * @brief 将 TCP DTO 转换为 Pinocchio SE3，语义上表示 flange -> tcp 的固定安装关系。
 */
pinocchio::SE3 BuildArtifactTcpSe3(const RoboSDP::Kinematics::Dto::TcpFrameDto& tcpFrame)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = tcpFrame.translation_m;
    pose.rpy_deg = tcpFrame.rpy_deg;
    return BuildArtifactPoseSe3(pose);
}

/**
 * @brief 将 Pinocchio SE3 重新转换为通用位姿 DTO，供派生 URDF origin 字段格式化复用。
 */
RoboSDP::Kinematics::Dto::CartesianPoseDto BuildArtifactPoseFromSe3(const pinocchio::SE3& transform)
{
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    const Eigen::Vector3d yawPitchRoll = transform.rotation().eulerAngles(2, 1, 0);

    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = {
        transform.translation()[0],
        transform.translation()[1],
        transform.translation()[2]};
    pose.rpy_deg = {
        yawPitchRoll[2] * kRadToDeg,
        yawPitchRoll[1] * kRadToDeg,
        yawPitchRoll[0] * kRadToDeg};
    return pose;
}

/**
 * @brief 标准 DH 派生 URDF 的关节后固定段：Rz(theta_offset) * Tz(d) * Tx(a) * Rx(alpha)。
 * @details 变量关节角 q 由前一段 revolute joint 提供，因此该固定段中显式保留 theta_offset。
 */
pinocchio::SE3 BuildStandardDhArtifactPostPlacement(
    const RoboSDP::Kinematics::Dto::KinematicLinkParameterDto& link)
{
    const double theta = PreviewDegToRad(link.theta_offset);
    const double alpha = PreviewDegToRad(link.alpha);
    const Eigen::Matrix3d rotation =
        (Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const Eigen::Vector3d translation(link.a * std::cos(theta), link.a * std::sin(theta), link.d);
    return pinocchio::SE3(rotation, translation);
}

/**
 * @brief 改进 DH 派生 URDF 的关节前固定段：Tx(a) * Rx(alpha)。
 */
pinocchio::SE3 BuildModifiedDhArtifactPrePlacement(
    const RoboSDP::Kinematics::Dto::KinematicLinkParameterDto& link)
{
    const double alpha = PreviewDegToRad(link.alpha);
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Vector3d translation(link.a, 0.0, 0.0);
    return pinocchio::SE3(rotation, translation);
}

/**
 * @brief 改进 DH 派生 URDF 的关节后固定段：Rz(theta_offset) * Tz(d)。
 */
pinocchio::SE3 BuildModifiedDhArtifactPostPlacement(
    const RoboSDP::Kinematics::Dto::KinematicLinkParameterDto& link)
{
    const double theta = PreviewDegToRad(link.theta_offset);
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d translation(0.0, 0.0, link.d);
    return pinocchio::SE3(rotation, translation);
}

/**
 * @brief 将固定安装位姿格式化为 URDF origin 属性字符串。
 */
// BuildUrdfOriginAttributes is provided by KinematicsServiceInternal.h.

/**
 * @brief 生成一个最小 link 片段；当前阶段仅承载运动学主链，不写入虚假的质量或 Mesh 数据。
 */
// BuildUrdfLinkBlock is provided by KinematicsServiceInternal.h.

} // namespace

RoboSDP::Errors::ErrorCode KinematicsService::WriteDerivedUrdfArtifact(
    const QString& projectRootPath,
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    RoboSDP::Kinematics::Dto::UnifiedRobotModelSnapshotDto& snapshot,
    QString& diagnosticMessage) const
{
#if defined(ROBOSDP_HAVE_PINOCCHIO)
    using namespace RoboSDP::Kinematics::Dto;

    if (projectRootPath.trimmed().isEmpty())
    {
        snapshot.derived_artifact_state_code = QStringLiteral("project_root_missing");
        snapshot.derived_artifact_exists = false;
        snapshot.derived_artifact_fresh = false;
        diagnosticMessage = QStringLiteral("当前项目目录为空，无法写出派生 URDF 文件。");
        return RoboSDP::Errors::ErrorCode::InvalidArgument;
    }

    if (model.links.empty() || model.joint_limits.empty() || model.links.size() != model.joint_limits.size())
    {
        snapshot.derived_artifact_state_code = QStringLiteral("invalid_kinematic_chain");
        snapshot.derived_artifact_exists = false;
        snapshot.derived_artifact_fresh = false;
        diagnosticMessage = QStringLiteral("当前 DH/MDH 主链不完整，无法生成派生 URDF 文件。");
        return RoboSDP::Errors::ErrorCode::InvalidArgument;
    }

    if (snapshot.derived_artifact_relative_path.trimmed().isEmpty())
    {
        const QString normalizedKinematicId = model.meta.kinematic_id.trimmed().isEmpty()
            ? QStringLiteral("default")
            : model.meta.kinematic_id.trimmed();
        snapshot.derived_artifact_relative_path =
            QStringLiteral("kinematics/derived/%1.urdf").arg(normalizedKinematicId);
    }

    const QString artifactAbsolutePath =
        QDir(projectRootPath).absoluteFilePath(snapshot.derived_artifact_relative_path);
    const QFileInfo artifactFileInfo(artifactAbsolutePath);
    QDir artifactDirectory = artifactFileInfo.dir();
    if (!artifactDirectory.exists() && !artifactDirectory.mkpath(QStringLiteral(".")))
    {
        snapshot.derived_artifact_state_code = QStringLiteral("artifact_dir_create_failed");
        snapshot.derived_artifact_exists = false;
        snapshot.derived_artifact_fresh = false;
        diagnosticMessage = QStringLiteral("无法创建派生 URDF 目录：%1").arg(artifactDirectory.absolutePath());
        return RoboSDP::Errors::ErrorCode::RepositoryWriteFailed;
    }

    const QString robotName = model.meta.name.trimmed().isEmpty()
        ? QStringLiteral("derived_dh_robot")
        : model.meta.name.trimmed();
    const QString baseRootLinkName = QStringLiteral("base_root");
    const QString baseLinkName = QStringLiteral("base_link");
    const QString flangeLinkName = QStringLiteral("flange_link");
    const QString tcpLinkName = QStringLiteral("tcp_link");
    const bool useModifiedDh = model.parameter_convention == QStringLiteral("MDH");

    QString urdfText;
    QTextStream stream(&urdfText);
    stream.setRealNumberNotation(QTextStream::FixedNotation);
    stream.setRealNumberPrecision(8);
    stream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    stream << "<robot name=\"" << robotName << "\">\n";
    stream << "  <!-- 中文说明：该文件由 DH/MDH 参数化设计模型自动派生，仅承载最小运动学主链与固定坐标系语义。 -->\n";
    stream << "  <!-- 当前阶段不导出真实 visual / collision / inertial 数据，避免误解为高保真工程数字样机。 -->\n";
    stream << BuildUrdfLinkBlock(baseRootLinkName);
    stream << BuildUrdfLinkBlock(baseLinkName);
    stream << "  <joint name=\"base_mount_joint\" type=\"fixed\">\n";
    stream << "    <parent link=\"" << baseRootLinkName << "\"/>\n";
    stream << "    <child link=\"" << baseLinkName << "\"/>\n";
    stream << "    <origin " << BuildUrdfOriginAttributes(model.base_frame) << "/>\n";
    stream << "  </joint>\n";

    QString currentParentLink = baseLinkName;
    for (std::size_t index = 0; index < model.links.size(); ++index)
    {
        const auto& link = model.links[index];
        const auto& jointLimit = model.joint_limits[index];
        const QString safeJointId = jointLimit.joint_id.trimmed().isEmpty()
            ? QStringLiteral("joint_%1").arg(static_cast<int>(index) + 1)
            : jointLimit.joint_id.trimmed();
        const QString safeLinkId = link.link_id.trimmed().isEmpty()
            ? QStringLiteral("link_%1").arg(static_cast<int>(index) + 1)
            : link.link_id.trimmed();
        const QString preLinkName = QStringLiteral("%1_pre_link").arg(safeJointId);
        const QString axisLinkName = QStringLiteral("%1_axis_link").arg(safeJointId);
        const QString postFixedJointName = QStringLiteral("%1_post_fixed").arg(safeJointId);

        if (useModifiedDh)
        {
            const auto prePose = BuildArtifactPoseFromSe3(BuildModifiedDhArtifactPrePlacement(link));
            stream << BuildUrdfLinkBlock(preLinkName);
            stream << "  <joint name=\"" << safeJointId << "_pre_fixed\" type=\"fixed\">\n";
            stream << "    <parent link=\"" << currentParentLink << "\"/>\n";
            stream << "    <child link=\"" << preLinkName << "\"/>\n";
            stream << "    <origin " << BuildUrdfOriginAttributes(prePose) << "/>\n";
            stream << "  </joint>\n";
            currentParentLink = preLinkName;
        }

        stream << BuildUrdfLinkBlock(axisLinkName);
        stream << "  <joint name=\"" << safeJointId << "\" type=\"revolute\">\n";
        stream << "    <parent link=\"" << currentParentLink << "\"/>\n";
        stream << "    <child link=\"" << axisLinkName << "\"/>\n";
        stream << "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
        stream << "    <axis xyz=\"0 0 1\"/>\n";
        stream << "    <limit lower=\"" << NormalizeArtifactNumber(PreviewDegToRad(jointLimit.hard_limit[0])) << "\" "
               << "upper=\"" << NormalizeArtifactNumber(PreviewDegToRad(jointLimit.hard_limit[1])) << "\" "
               << "effort=\"1000.00000000\" "
               << "velocity=\"" << NormalizeArtifactNumber(PreviewDegToRad(jointLimit.max_velocity)) << "\"/>\n";
        stream << "  </joint>\n";

        const auto postPose = useModifiedDh
            ? BuildArtifactPoseFromSe3(BuildModifiedDhArtifactPostPlacement(link))
            : BuildArtifactPoseFromSe3(BuildStandardDhArtifactPostPlacement(link));
        stream << BuildUrdfLinkBlock(safeLinkId);
        stream << "  <joint name=\"" << postFixedJointName << "\" type=\"fixed\">\n";
        stream << "    <parent link=\"" << axisLinkName << "\"/>\n";
        stream << "    <child link=\"" << safeLinkId << "\"/>\n";
        stream << "    <origin " << BuildUrdfOriginAttributes(postPose) << "/>\n";
        stream << "  </joint>\n";
        currentParentLink = safeLinkId;
    }

    stream << BuildUrdfLinkBlock(flangeLinkName);
    stream << "  <joint name=\"flange_fixed_joint\" type=\"fixed\">\n";
    stream << "    <parent link=\"" << currentParentLink << "\"/>\n";
    stream << "    <child link=\"" << flangeLinkName << "\"/>\n";
    stream << "    <origin " << BuildUrdfOriginAttributes(model.flange_frame) << "/>\n";
    stream << "  </joint>\n";

    const auto tcpPose = BuildArtifactPoseFromSe3(BuildArtifactTcpSe3(model.tcp_frame));
    stream << BuildUrdfLinkBlock(tcpLinkName);
    stream << "  <joint name=\"tcp_fixed_joint\" type=\"fixed\">\n";
    stream << "    <parent link=\"" << flangeLinkName << "\"/>\n";
    stream << "    <child link=\"" << tcpLinkName << "\"/>\n";
    stream << "    <origin " << BuildUrdfOriginAttributes(tcpPose) << "/>\n";
    stream << "  </joint>\n";
    stream << "</robot>\n";

    QSaveFile artifactFile(artifactAbsolutePath);
    if (!artifactFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        snapshot.derived_artifact_state_code = QStringLiteral("artifact_open_failed");
        snapshot.derived_artifact_exists = false;
        snapshot.derived_artifact_fresh = false;
        diagnosticMessage = QStringLiteral("无法写入派生 URDF 文件：%1").arg(artifactAbsolutePath);
        return RoboSDP::Errors::ErrorCode::RepositoryWriteFailed;
    }

    const QByteArray payload = urdfText.toUtf8();
    if (artifactFile.write(payload) != payload.size() || !artifactFile.commit())
    {
        snapshot.derived_artifact_state_code = QStringLiteral("artifact_write_failed");
        snapshot.derived_artifact_exists = false;
        snapshot.derived_artifact_fresh = false;
        diagnosticMessage = QStringLiteral("派生 URDF 文件写出失败：%1").arg(artifactAbsolutePath);
        return RoboSDP::Errors::ErrorCode::RepositoryWriteFailed;
    }

    const QFileInfo writtenFileInfo(artifactAbsolutePath);
    snapshot.derived_artifact_relative_path =
        QDir(projectRootPath).relativeFilePath(artifactAbsolutePath).replace(QLatin1Char('\\'), QLatin1Char('/'));
    snapshot.derived_artifact_version = QStringLiteral("file_v%1").arg(snapshot.snapshot_version);
    snapshot.derived_artifact_generated_at_utc =
        writtenFileInfo.lastModified().toUTC().toString(Qt::ISODateWithMs);
    snapshot.derived_artifact_state_code = QStringLiteral("file_generated");
    snapshot.derived_artifact_exists = writtenFileInfo.exists();
    snapshot.derived_artifact_fresh = writtenFileInfo.exists();
    diagnosticMessage = QStringLiteral("当前设计真源为 DH/MDH 参数化模型，派生最小 URDF 已写出：%1")
        .arg(snapshot.derived_artifact_relative_path);
    return RoboSDP::Errors::ErrorCode::Ok;
#else
    Q_UNUSED(projectRootPath);
    Q_UNUSED(model);
    snapshot.derived_artifact_state_code = QStringLiteral("pinocchio_disabled");
    snapshot.derived_artifact_exists = false;
    snapshot.derived_artifact_fresh = false;
    diagnosticMessage = QStringLiteral("当前构建未启用 Pinocchio，无法写出派生 URDF 文件。");
    return RoboSDP::Errors::ErrorCode::UnknownError;
#endif
}

RoboSDP::Kinematics::Dto::KinematicModelDto KinematicsService::BuildModelFromTopology(
    const RoboSDP::Topology::Dto::RobotTopologyModelDto& topologyModel) const
{
    using namespace RoboSDP::Kinematics::Dto;

    KinematicModelDto model = KinematicModelDto::CreateDefault();

    // =====================================================================
    // 1. 同步并继承和映射上游带来的元数据信息
    // =====================================================================
    model.meta.kinematic_id = QStringLiteral("kinematic_%1").arg(topologyModel.meta.topology_id);
    model.meta.name = QStringLiteral("%1 运动学模型").arg(topologyModel.meta.name);
    model.meta.source = QStringLiteral("topology");
    model.meta.status = QStringLiteral("ready");
    model.meta.topology_ref = topologyModel.meta.topology_id; // 确立追踪关联
    model.meta.requirement_ref = topologyModel.meta.requirement_ref;
    model.master_model_type = QStringLiteral("dh_mdh");
    model.derived_model_state = QStringLiteral("fresh");
    model.dh_editable = true;
    model.urdf_editable = false;
    model.conversion_diagnostics = DefaultConversionDiagnostics(model.master_model_type);

    // 初始化后端引擎配置与状态标记
    model.modeling_mode = QStringLiteral("DH"); // 强制明确指定为标准 DH
    model.parameter_convention = model.modeling_mode;
    model.unified_robot_model_ref.clear();
    model.model_source_mode = QStringLiteral("topology_derived");
    model.backend_type = QStringLiteral("pinocchio_kinematic_backend");
    model.urdf_source_path.clear();
    model.pinocchio_model_ready = false;
    model.frame_semantics_version = 1;
    model.joint_count = topologyModel.robot_definition.joint_count;

    // =====================================================================
    // 2. 将物理规格落实为实际的几何参数映射 (核心修复区)
    // =====================================================================

    // (1) 清理 base_frame，遵循标准 DH 规范，基座高度 d1 将放置在 Link 1 中
    model.base_frame.position_m = {0.0, 0.0, 0.0};

    // 安装姿态补偿：根据底座挂载类型，在世界基坐标系中硬性注入预偏转角
    if (topologyModel.robot_definition.base_mount_type == QStringLiteral("wall")) // 壁挂式
    {
        model.base_frame.rpy_deg[1] = 90.0; // 绕 Y 轴转 90 度
    }
    else if (topologyModel.robot_definition.base_mount_type == QStringLiteral("ceiling")) // 倒挂/吊装式
    {
        model.base_frame.rpy_deg[0] = 180.0; // 绕 X 轴翻转 180 度
    }

    // (2) 提取拓扑界面的物理尺寸
    const double d1 = topologyModel.robot_definition.base_height_m;      // 基座高度
    const double a1 = topologyModel.robot_definition.shoulder_offset_m;  // 肩部偏置
    const double a2 = topologyModel.robot_definition.upper_arm_length_m; // 大臂长度
    const double a3 = topologyModel.robot_definition.elbow_offset_m;     // 肘部偏移
    const double d4 = topologyModel.robot_definition.forearm_length_m;   // 小臂长度
    const double d6 = topologyModel.robot_definition.wrist_offset_m;     // 腕部偏置

    // (3) 严格按照 6R 机器人的标准 D-H 规范映射参数
    if (model.links.size() >= 6)
    {
        // Link 1 (腰关节)
        model.links[0].a = a1;         // 肩部偏置
        model.links[0].alpha = 90.0;
        model.links[0].d = d1;         // 基座高度作为 d1
        model.links[0].theta_offset = 0.0;

        // Link 2 (肩关节)
        model.links[1].a = a2;         // 大臂长度作为 a2
        model.links[1].alpha = 0.0;
        model.links[1].d = 0.0;
        // 补偿 -90 度，使模型在零位时大臂前伸（与 Topology 预览 L 型对齐）
        model.links[1].theta_offset = 90.0;

        // Link 3 (肘关节)
        model.links[2].a = a3;
        model.links[2].alpha = 90.0;
        model.links[2].d = 0.0;
        // 补偿 90 度，使模型在零位时小臂垂直（与 Topology 预览 L 型对齐）
        model.links[2].theta_offset = 90.0;

        // Link 4 (小臂滚动)
        model.links[3].a = 0.0;
        model.links[3].alpha = -90.0;
        model.links[3].d = d4;         // 小臂长度作为 d4
        model.links[3].theta_offset = 0.0;

        // Link 5 (手腕摆动)
        model.links[4].a = 0.0;
        model.links[4].alpha = 90.0;
        model.links[4].d = 0.0;
        model.links[4].theta_offset = 0.0;

        // Link 6 (法兰盘)
        model.links[5].a = 0.0;
        model.links[5].alpha = 0.0;
        model.links[5].d = d6;         // 腕部偏置作为 d6
        model.links[5].theta_offset = 0.0;

        // 业务特性体现：空心手腕补偿
        if (topologyModel.layout.hollow_wrist_required)
        {
            model.links[4].d = 0.12;
            model.links[5].d = 0.10;
            model.tcp_frame.translation_m[2] = 0.12; // 确保 TCP 也同步前移
        }
    }

    // =====================================================================
    // 3. 构建并计算关节动力学限制参数
    // =====================================================================
    model.joint_limits.clear();
    for (const auto& topologyJoint : topologyModel.joints)
    {
        KinematicJointLimitDto limit;
        limit.joint_id = topologyJoint.joint_id;
        limit.hard_limit = topologyJoint.motion_range_deg; // 机械硬止挡位

        // 调用辅助函数，动态计算带有退让缓冲区的控制软件限位
        limit.soft_limit = {
            PreferredSoftLimit(topologyJoint.motion_range_deg[0], true),
            PreferredSoftLimit(topologyJoint.motion_range_deg[1], false)};

        // 经验公式法则：靠近末端的手腕轴惯量小，允许运行得更快；底座重轴相对较慢
        limit.max_velocity = topologyJoint.axis_index >= 4 ? 260.0 : 180.0;
        limit.max_acceleration = topologyJoint.axis_index >= 4 ? 520.0 : 360.0;
        model.joint_limits.push_back(limit);
    }

    // =====================================================================
    // 4. 生成防篡改字符串签名
    // =====================================================================
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
