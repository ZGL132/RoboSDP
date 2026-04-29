#include "modules/dynamics/service/DynamicsService.h"

#include <QDir>
#include <QFileInfo>

#include <cmath>
#include <limits>

namespace RoboSDP::Dynamics::Service
{

namespace
{

double EstimateMassByIndex(std::size_t index)
{
    static const double kMasses[] {8.0, 6.5, 5.0, 3.5, 2.4, 1.5};
    return index < std::size(kMasses) ? kMasses[index] : 1.0;
}

double ComputeRmsTorque(const RoboSDP::Dynamics::Dto::JointLoadCurveDto& curve)
{
    if (curve.samples.empty())
    {
        return 0.0;
    }

    double sum = 0.0;
    for (const auto& sample : curve.samples)
    {
        sum += sample.torque_nm * sample.torque_nm;
    }
    return std::sqrt(sum / static_cast<double>(curve.samples.size()));
}

QString BuildSnapshotSummary(const RoboSDP::Kinematics::Dto::UnifiedRobotModelSnapshotDto& snapshot)
{
    const QString ref = snapshot.unified_robot_model_ref.trimmed().isEmpty()
        ? QStringLiteral("not_generated")
        : snapshot.unified_robot_model_ref.trimmed();
    const QString mode = QStringLiteral("%1/%2")
        .arg(snapshot.master_model_type.trimmed().isEmpty() ? QStringLiteral("unknown") : snapshot.master_model_type.trimmed())
        .arg(snapshot.modeling_mode.trimmed().isEmpty() ? QStringLiteral("unknown") : snapshot.modeling_mode.trimmed());
    const QString readiness = snapshot.pinocchio_model_ready
        ? QStringLiteral("ready")
        : QStringLiteral("not_ready");
    const QString artifactState = snapshot.derived_artifact_state_code.trimmed().isEmpty()
        ? QStringLiteral("unknown_artifact")
        : snapshot.derived_artifact_state_code.trimmed();
    const QString freshness = snapshot.derived_artifact_fresh
        ? QStringLiteral("fresh")
        : (snapshot.derived_artifact_exists ? QStringLiteral("stale") : QStringLiteral("missing"));
    return QStringLiteral("%1 | %2 | %3 | %4/%5").arg(ref, mode, readiness, artifactState, freshness);
}

void RefreshArtifactRuntimeState(
    const QString& projectRootPath,
    RoboSDP::Kinematics::Dto::UnifiedRobotModelSnapshotDto& snapshot)
{
    if (snapshot.derived_artifact_relative_path.trimmed().isEmpty())
    {
        return;
    }

    const QString absoluteArtifactPath =
        QDir(projectRootPath).absoluteFilePath(snapshot.derived_artifact_relative_path);
    const QFileInfo artifactInfo(absoluteArtifactPath);
    snapshot.derived_artifact_exists = artifactInfo.exists();

    if (!artifactInfo.exists())
    {
        if (snapshot.derived_artifact_state_code == QStringLiteral("file_generated"))
        {
            snapshot.derived_artifact_state_code = QStringLiteral("artifact_missing_on_disk");
        }
        snapshot.derived_artifact_fresh = false;
        return;
    }

    const QString actualGeneratedAtUtc = artifactInfo.lastModified().toUTC().toString(Qt::ISODateWithMs);
    const bool timestampMatches =
        snapshot.derived_artifact_generated_at_utc.trimmed().isEmpty() ||
        snapshot.derived_artifact_generated_at_utc == actualGeneratedAtUtc;
    snapshot.derived_artifact_fresh = timestampMatches;
    if (!timestampMatches)
    {
        snapshot.derived_artifact_state_code = QStringLiteral("artifact_stale");
    }
}

} // namespace

DynamicsService::DynamicsService(
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage& storage,
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage& kinematicStorage,
    RoboSDP::Logging::ILogger* logger)
    : m_storage(storage)
    , m_kinematic_storage(kinematicStorage)
    , m_logger(logger)
    , m_pinocchio_dynamics_backend_adapter(logger)
{
}

RoboSDP::Dynamics::Dto::DynamicModelDto DynamicsService::CreateDefaultModel() const
{
    return RoboSDP::Dynamics::Dto::DynamicModelDto::CreateDefault();
}

RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto DynamicsService::CreateDefaultState() const
{
    return RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto::CreateDefault();
}

DynamicsBuildResult DynamicsService::BuildFromKinematics(const QString& projectRootPath) const
{
    DynamicsBuildResult result;
    result.state = CreateDefaultState();
    result.kinematic_file_path = m_kinematic_storage.BuildAbsoluteModelFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法从 Kinematics 构建 Dynamics。");
        return result;
    }

    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto kinematicState;
    result.error_code = m_kinematic_storage.LoadModel(projectRootPath, kinematicState);
    if (result.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.message = QStringLiteral("读取 Kinematics 草稿失败，请先保存 Kinematics：%1")
                             .arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        return result;
    }

    result.state.current_model = BuildModelFromKinematics(kinematicState);
    RefreshArtifactRuntimeState(projectRootPath, result.state.current_model.meta.unified_robot_snapshot);
    result.upstream_snapshot_ready =
        result.state.current_model.meta.unified_robot_snapshot.pinocchio_model_ready;
    result.upstream_derived_artifact_exists =
        result.state.current_model.meta.unified_robot_snapshot.derived_artifact_exists;
    result.upstream_derived_artifact_fresh =
        result.state.current_model.meta.unified_robot_snapshot.derived_artifact_fresh;
    result.upstream_snapshot_summary =
        BuildSnapshotSummary(result.state.current_model.meta.unified_robot_snapshot);
    if (!result.upstream_snapshot_ready)
    {
        result.message = QStringLiteral("已基于 KinematicModel 生成最小 DynamicModel，但上游统一主链快照尚未就绪。");
    }
    else if (!result.upstream_derived_artifact_exists)
    {
        result.message = QStringLiteral("已基于 KinematicModel 生成最小 DynamicModel，但上游派生 URDF 文件缺失。");
    }
    else if (!result.upstream_derived_artifact_fresh)
    {
        result.message = QStringLiteral("已基于 KinematicModel 生成最小 DynamicModel，但上游派生 URDF 文件不是最新版本。");
    }
    else
    {
        result.message = QStringLiteral("已基于 KinematicModel 生成最小 DynamicModel，上游统一主链与派生 URDF 文件均已就绪。");
    }
    return result;
}

DynamicsAnalyzeResult DynamicsService::RunInverseDynamicsChain(
    const QString& projectRootPath,
    const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& inputState) const
{
    DynamicsAnalyzeResult result;
    result.state = inputState;
    result.kinematic_file_path = m_kinematic_storage.BuildAbsoluteModelFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法执行 Dynamics 分析。");
        return result;
    }

    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto kinematicState;
    result.error_code = m_kinematic_storage.LoadModel(projectRootPath, kinematicState);
    if (result.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.message = QStringLiteral("读取 Kinematics 草稿失败，无法执行逆动力学：%1")
                             .arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        return result;
    }

    if (result.state.current_model.trajectories.empty())
    {
        result.state.current_model.trajectories = m_trajectory_factory.CreateMinimumSet(kinematicState.current_model);
    }

    result.state.parameterized_trajectories.clear();
    result.state.trajectory_results.clear();
    result.state.peak_stats.clear();
    result.state.rms_stats.clear();
    result.state.load_envelope.joints.clear();

    bool hasFailedTrajectory = false;
    QString firstFailureMessage;

    for (const auto& trajectory : result.state.current_model.trajectories)
    {
        const auto parameterized = m_time_parameterization.Parameterize(trajectory);
        result.state.parameterized_trajectories.push_back(parameterized);

        // 中文说明：Dynamics 主链已经统一交给共享 Pinocchio 内核负责。
        const auto analysis = m_pinocchio_dynamics_backend_adapter.Analyze(
            kinematicState.current_model,
            result.state.current_model,
            parameterized);
        result.state.trajectory_results.push_back(analysis);

        if (!analysis.success)
        {
            hasFailedTrajectory = true;
            if (firstFailureMessage.trimmed().isEmpty())
            {
                firstFailureMessage = analysis.message;
            }
            continue;
        }

        for (const auto& curve : analysis.joint_curves)
        {
            double peakTorque = 0.0;
            double peakSpeed = 0.0;
            double peakPower = 0.0;
            for (const auto& sample : curve.samples)
            {
                peakTorque = std::max(peakTorque, std::abs(sample.torque_nm));
                peakSpeed = std::max(peakSpeed, std::abs(sample.speed_deg_s));
                peakPower = std::max(peakPower, std::abs(sample.power_w));
            }

            result.state.peak_stats.push_back({
                curve.joint_id,
                analysis.trajectory_id,
                peakTorque,
                peakSpeed,
                peakPower});

            result.state.rms_stats.push_back({
                curve.joint_id,
                analysis.trajectory_id,
                ComputeRmsTorque(curve)});
        }
    }

    const std::size_t jointCount = result.state.current_model.joints.size();
    for (std::size_t jointIndex = 0; jointIndex < jointCount; ++jointIndex)
    {
        RoboSDP::Dynamics::Dto::LoadEnvelopeJointDto envelopeJoint;
        envelopeJoint.joint_id = result.state.current_model.joints[jointIndex].joint_id;
        envelopeJoint.peak_torque_nm = -std::numeric_limits<double>::infinity();
        envelopeJoint.rms_torque_nm = -std::numeric_limits<double>::infinity();
        envelopeJoint.peak_power_w = -std::numeric_limits<double>::infinity();

        for (const auto& peak : result.state.peak_stats)
        {
            if (peak.joint_id == envelopeJoint.joint_id && peak.peak_torque_nm >= envelopeJoint.peak_torque_nm)
            {
                envelopeJoint.peak_torque_nm = peak.peak_torque_nm;
                envelopeJoint.peak_power_w = peak.peak_power_w;
                envelopeJoint.source_trajectory_id = peak.trajectory_id;
            }
        }

        for (const auto& rms : result.state.rms_stats)
        {
            if (rms.joint_id == envelopeJoint.joint_id)
            {
                envelopeJoint.rms_torque_nm = std::max(envelopeJoint.rms_torque_nm, rms.rms_torque_nm);
            }
        }

        if (std::isfinite(envelopeJoint.peak_torque_nm))
        {
            result.state.load_envelope.joints.push_back(envelopeJoint);
        }
    }

    if (hasFailedTrajectory)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
        result.message = QStringLiteral("逆动力学主链执行失败：%1")
                             .arg(firstFailureMessage.trimmed().isEmpty()
                                      ? QStringLiteral("至少有一条轨迹未能完成逆动力学计算。")
                                      : firstFailureMessage);
        return result;
    }

    result.message = QStringLiteral("逆动力学主链执行完成，已输出峰值、RMS 与负载包络；本轮计算由共享 Pinocchio 内核完成。");
    return result;
}

DynamicsSaveResult DynamicsService::SaveDraft(
    const QString& projectRootPath,
    const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& state) const
{
    DynamicsSaveResult result;
    result.model_file_path = m_storage.BuildAbsoluteModelFilePath(projectRootPath);
    result.mass_file_path = m_storage.BuildAbsoluteMassFilePath(projectRootPath);
    result.envelope_file_path = m_storage.BuildAbsoluteEnvelopeFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法保存 Dynamics 草稿。");
        return result;
    }

    result.error_code = m_storage.Save(projectRootPath, state);
    result.message = result.IsSuccess()
        ? QStringLiteral("Dynamics 草稿已保存到：%1").arg(result.model_file_path)
        : QStringLiteral("Dynamics 草稿保存失败：%1").arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
    return result;
}

DynamicsLoadResult DynamicsService::LoadDraft(const QString& projectRootPath) const
{
    DynamicsLoadResult result;
    result.model_file_path = m_storage.BuildAbsoluteModelFilePath(projectRootPath);
    result.mass_file_path = m_storage.BuildAbsoluteMassFilePath(projectRootPath);
    result.envelope_file_path = m_storage.BuildAbsoluteEnvelopeFilePath(projectRootPath);
    result.state = CreateDefaultState();

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法加载 Dynamics 草稿。");
        return result;
    }

    result.error_code = m_storage.Load(projectRootPath, result.state);
    result.message = result.IsSuccess()
        ? QStringLiteral("Dynamics 已从 JSON 重新加载。")
        : QStringLiteral("Dynamics 加载失败：%1").arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
    return result;
}

RoboSDP::Dynamics::Dto::NativeRneaDryRunResultDto DynamicsService::InspectNativeRneaDryRun(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& kinematicModel,
    const RoboSDP::Dynamics::Dto::DynamicModelDto& dynamicModel,
    const std::vector<double>& joint_positions_deg,
    const std::vector<double>& joint_velocities_deg_s,
    const std::vector<double>& joint_accelerations_deg_s2) const
{
    return m_pinocchio_dynamics_backend_adapter.EvaluateNativeRneaDryRun(
        kinematicModel,
        dynamicModel,
        joint_positions_deg,
        joint_velocities_deg_s,
        joint_accelerations_deg_s2);
}

RoboSDP::Dynamics::Dto::DynamicModelDto DynamicsService::BuildModelFromKinematics(
    const RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& kinematicState) const
{
    RoboSDP::Dynamics::Dto::DynamicModelDto model = RoboSDP::Dynamics::Dto::DynamicModelDto::CreateDefault();
    const auto& kinematicModel = kinematicState.current_model;
    auto upstreamSnapshot = kinematicModel.unified_robot_snapshot;
    if (upstreamSnapshot.unified_robot_model_ref.trimmed().isEmpty())
    {
        upstreamSnapshot.unified_robot_model_ref = kinematicModel.unified_robot_model_ref;
        upstreamSnapshot.source_kinematic_id = kinematicModel.meta.kinematic_id;
        upstreamSnapshot.master_model_type = kinematicModel.master_model_type;
        upstreamSnapshot.modeling_mode = kinematicModel.modeling_mode;
        upstreamSnapshot.parameter_convention = kinematicModel.parameter_convention;
        upstreamSnapshot.backend_type = kinematicModel.backend_type;
        upstreamSnapshot.joint_order_signature = kinematicModel.joint_order_signature;
        upstreamSnapshot.pinocchio_model_ready = kinematicModel.pinocchio_model_ready;
        upstreamSnapshot.frame_semantics_version = kinematicModel.frame_semantics_version;
        upstreamSnapshot.model_source_mode = kinematicModel.model_source_mode;
        upstreamSnapshot.conversion_diagnostics = kinematicModel.conversion_diagnostics;
    }
    model.meta.dynamic_id = QStringLiteral("dynamic_%1").arg(kinematicState.current_model.meta.kinematic_id);
    model.meta.name = QStringLiteral("%1 动力学模型").arg(kinematicState.current_model.meta.name);
    model.meta.source = QStringLiteral("kinematics");
    model.meta.status = QStringLiteral("ready");
    model.meta.kinematic_ref = kinematicState.current_model.meta.kinematic_id;
    model.meta.topology_ref = kinematicState.current_model.meta.topology_ref;
    model.meta.requirement_ref = kinematicState.current_model.meta.requirement_ref;
    model.meta.unified_robot_model_ref = upstreamSnapshot.unified_robot_model_ref;
    model.meta.kinematic_kernel_ready = upstreamSnapshot.pinocchio_model_ready;
    model.meta.kinematic_conversion_diagnostics = upstreamSnapshot.conversion_diagnostics;
    model.meta.unified_robot_snapshot = upstreamSnapshot;
    model.installation_pose.position_m = kinematicModel.base_frame.position_m;
    model.installation_pose.rpy_deg = kinematicModel.base_frame.rpy_deg;

    model.links.clear();
    for (std::size_t index = 0; index < kinematicModel.links.size(); ++index)
    {
        const auto& kinematicLink = kinematicModel.links[index];
        const double mass = EstimateMassByIndex(index);
        const double cogX = kinematicLink.a * 0.5;
        const double cogZ = kinematicLink.d * 0.5;
        model.links.push_back({
            kinematicLink.link_id,
            mass,
            {cogX, 0.0, cogZ},
            {mass * 0.01, mass * 0.01, mass * 0.005, 0.0, 0.0, 0.0},
            QStringLiteral("estimated_from_shared_pinocchio_kernel")});
    }

    model.joints.clear();
    for (std::size_t index = 0; index < kinematicModel.joint_limits.size(); ++index)
    {
        const auto& limit = kinematicModel.joint_limits[index];
        const double ratio = index < 3 ? 120.0 - 10.0 * static_cast<double>(index)
                                       : 80.0 - 10.0 * static_cast<double>(index - 3);
        const double efficiency = index < 3 ? 0.92 : 0.94;
        model.joints.push_back({
            limit.joint_id,
            ratio,
            efficiency,
            {0.08 / static_cast<double>(index + 1), 0.45 / static_cast<double>(index + 1), 0.50 / static_cast<double>(index + 1)}});
    }

    model.end_effector.mass = 1.0;
    model.end_effector.cog = {
        kinematicModel.tcp_frame.translation_m[0],
        kinematicModel.tcp_frame.translation_m[1],
        kinematicModel.tcp_frame.translation_m[2] * 0.5};
    model.end_effector.inertia_tensor = {0.004, 0.004, 0.002, 0.0, 0.0, 0.0};
    model.trajectories = m_trajectory_factory.CreateMinimumSet(kinematicState.current_model);
    return model;
}

} // namespace RoboSDP::Dynamics::Service
