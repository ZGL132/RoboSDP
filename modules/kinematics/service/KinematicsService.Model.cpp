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

// ============================================================================
// BuildModelFromTopology — 从拓扑映射为运动学模型
// ============================================================================

RoboSDP::Kinematics::Dto::KinematicModelDto KinematicsService::BuildModelFromTopology(
    const RoboSDP::Topology::Dto::RobotTopologyModelDto& topologyModel) const
{
    using namespace RoboSDP::Kinematics::Dto;

    KinematicModelDto model = KinematicModelDto::CreateDefault();
    model.meta.kinematic_id = QStringLiteral("kinematic_%1").arg(topologyModel.meta.topology_id);
    model.meta.name = QStringLiteral("%1 运动学模型").arg(topologyModel.meta.name);
    model.meta.source = QStringLiteral("topology");
    model.meta.status = QStringLiteral("ready");
    model.meta.topology_ref = topologyModel.meta.topology_id;
    model.meta.requirement_ref = topologyModel.meta.requirement_ref;
    model.master_model_type = QStringLiteral("dh_mdh");
    model.derived_model_state = QStringLiteral("fresh");
    model.dh_editable = true;
    model.urdf_editable = false;
    model.conversion_diagnostics = DefaultConversionDiagnostics(model.master_model_type);
    model.modeling_mode = QStringLiteral("DH");
    model.parameter_convention = model.modeling_mode;
    model.unified_robot_model_ref.clear();
    model.model_source_mode = QStringLiteral("topology_derived");
    model.backend_type = QStringLiteral("pinocchio_kinematic_backend");
    model.urdf_source_path.clear();
    model.pinocchio_model_ready = false;
    model.frame_semantics_version = 1;
    model.joint_count = topologyModel.robot_definition.joint_count;

    model.base_frame.position_m = {0.0, 0.0, 0.0};
    if (topologyModel.robot_definition.base_mount_type == QStringLiteral("wall"))
        model.base_frame.rpy_deg[1] = 90.0;
    else if (topologyModel.robot_definition.base_mount_type == QStringLiteral("ceiling"))
        model.base_frame.rpy_deg[0] = 180.0;

    const double d1 = topologyModel.robot_definition.base_height_m;
    const double a1 = topologyModel.robot_definition.shoulder_offset_m;
    const double a2 = topologyModel.robot_definition.upper_arm_length_m;
    const double d4 = topologyModel.robot_definition.forearm_length_m;
    const double d6 = topologyModel.robot_definition.wrist_offset_m;

    if (model.links.size() >= 6)
    {
        model.links[0].a = a1; model.links[0].d = d1;
        model.links[0].alpha = -90.0; model.links[0].theta_offset = -90.0;
        model.links[0].link_id = QStringLiteral("link_1");

        model.links[1].a = a2; model.links[1].d = 0.0;
        model.links[1].alpha = 0.0; model.links[1].theta_offset = 0.0;
        model.links[1].link_id = QStringLiteral("link_2");

        model.links[2].a = 0.0; model.links[2].d = 0.0;
        model.links[2].alpha = 90.0; model.links[2].theta_offset = 90.0;
        model.links[2].link_id = QStringLiteral("link_3");

        model.links[3].a = 0.0; model.links[3].d = d4;
        model.links[3].alpha = -90.0;
        model.links[3].link_id = QStringLiteral("link_4");

        model.links[4].a = 0.0; model.links[4].d = 0.0;
        model.links[4].alpha = 90.0;
        model.links[4].link_id = QStringLiteral("link_5");

        model.links[5].a = 0.0; model.links[5].d = d6;
        model.links[5].alpha = 0.0;
        model.links[5].link_id = QStringLiteral("link_6");
    }

    if (topologyModel.layout.hollow_wrist_required)
    {
        model.links[4].d = 0.12;
        model.links[5].d = 0.10;
        model.tcp_frame.translation_m[2] = 0.12;
    }

    model.joint_limits.clear();
    for (const auto& topologyJoint : topologyModel.joints)
    {
        KinematicJointLimitDto limit;
        limit.joint_id = topologyJoint.joint_id;
        limit.hard_limit = topologyJoint.motion_range_deg;
        limit.soft_limit = {
            PreferredSoftLimit(topologyJoint.motion_range_deg[0], true),
            PreferredSoftLimit(topologyJoint.motion_range_deg[1], false)};
        limit.max_velocity = topologyJoint.axis_index >= 4 ? 260.0 : 180.0;
        limit.max_acceleration = topologyJoint.axis_index >= 4 ? 520.0 : 360.0;
        model.joint_limits.push_back(limit);
    }

    QString jointOrderSignature;
    for (std::size_t index = 0; index < model.joint_limits.size(); ++index)
    {
        if (index > 0) jointOrderSignature.append(QLatin1Char('|'));
        jointOrderSignature.append(model.joint_limits[index].joint_id);
    }
    model.joint_order_signature = jointOrderSignature;

    return model;
}

// ============================================================================
// WriteDerivedUrdfArtifact — 写出派生 URDF 文件（Pinocchio 依赖）
// ============================================================================

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
            ? QStringLiteral("default") : model.meta.kinematic_id.trimmed();
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
        ? QStringLiteral("derived_dh_robot") : model.meta.name.trimmed();
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
    stream << "  <!-- Auto-generated URDF from DH/MDH master model -->\n";
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
            ? QStringLiteral("joint_%1").arg(static_cast<int>(index) + 1) : jointLimit.joint_id.trimmed();
        const QString safeLinkId = link.link_id.trimmed().isEmpty()
            ? QStringLiteral("link_%1").arg(static_cast<int>(index) + 1) : link.link_id.trimmed();
        const QString preLinkName = QStringLiteral("%1_pre_link").arg(safeJointId);
        const QString axisLinkName = QStringLiteral("%1_axis_link").arg(safeJointId);
        const QString postFixedJointName = QStringLiteral("%1_post_fixed").arg(safeJointId);

        if (useModifiedDh)
        {
            stream << BuildUrdfLinkBlock(preLinkName);
            stream << "  <joint name=\"" << safeJointId << "_pre_fixed\" type=\"fixed\">\n";
            stream << "    <parent link=\"" << currentParentLink << "\"/>\n";
            stream << "    <child link=\"" << preLinkName << "\"/>\n";
            stream << "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
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
        stream << BuildUrdfLinkBlock(safeLinkId);
        stream << "  <joint name=\"" << postFixedJointName << "\" type=\"fixed\">\n";
        stream << "    <parent link=\"" << axisLinkName << "\"/>\n";
        stream << "    <child link=\"" << safeLinkId << "\"/>\n";
        stream << "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
        stream << "  </joint>\n";
        currentParentLink = safeLinkId;
    }

    stream << BuildUrdfLinkBlock(flangeLinkName);
    stream << "  <joint name=\"flange_fixed_joint\" type=\"fixed\">\n";
    stream << "    <parent link=\"" << currentParentLink << "\"/>\n";
    stream << "    <child link=\"" << flangeLinkName << "\"/>\n";
    stream << "    <origin " << BuildUrdfOriginAttributes(model.flange_frame) << "/>\n";
    stream << "  </joint>\n";
    stream << BuildUrdfLinkBlock(tcpLinkName);
    stream << "  <joint name=\"tcp_fixed_joint\" type=\"fixed\">\n";
    stream << "    <parent link=\"" << flangeLinkName << "\"/>\n";
    stream << "    <child link=\"" << tcpLinkName << "/>\n";
    stream << "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
    stream << "  </joint>\n";
    stream << "</robot>\n";

    QSaveFile artifactFile(artifactAbsolutePath);
    if (!artifactFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        snapshot.derived_artifact_state_code = QStringLiteral("artifact_open_failed");
        diagnosticMessage = QStringLiteral("无法写入派生 URDF 文件：%1").arg(artifactAbsolutePath);
        return RoboSDP::Errors::ErrorCode::RepositoryWriteFailed;
    }

    const QByteArray payload = urdfText.toUtf8();
    if (artifactFile.write(payload) != payload.size() || !artifactFile.commit())
    {
        snapshot.derived_artifact_state_code = QStringLiteral("artifact_write_failed");
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
    diagnosticMessage = QStringLiteral("当前草稿以 DH/MDH 参数为主模型，派生 URDF 已写出：%1")
        .arg(snapshot.derived_artifact_relative_path);
    return RoboSDP::Errors::ErrorCode::Ok;
#else
    Q_UNUSED(projectRootPath); Q_UNUSED(model);
    snapshot.derived_artifact_state_code = QStringLiteral("pinocchio_disabled");
    snapshot.derived_artifact_exists = false;
    snapshot.derived_artifact_fresh = false;
    diagnosticMessage = QStringLiteral("当前构建未启用 Pinocchio，无法写出派生 URDF 文件。");
    return RoboSDP::Errors::ErrorCode::UnknownError;
#endif
}

} // namespace RoboSDP::Kinematics::Service
