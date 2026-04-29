#include "modules/planning/service/PlanningVerificationService.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>

#include <algorithm>
#include <array>

namespace RoboSDP::Planning::Service
{

namespace
{

double ClampToRange(double value, double lower, double upper)
{
    return std::max(lower, std::min(upper, value));
}

QString BuildSceneId(const QString& kinematicId)
{
    return QStringLiteral("planning_scene_%1").arg(kinematicId.isEmpty() ? QStringLiteral("001") : kinematicId);
}

QString BuildRequestId()
{
    return QStringLiteral("planning_request_%1")
        .arg(QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMddhhmmsszzz")));
}

QString BuildVerificationId()
{
    return QStringLiteral("planning_verification_%1")
        .arg(QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMddhhmmsszzz")));
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

PlanningVerificationService::PlanningVerificationService(
    RoboSDP::Planning::Persistence::PlanningJsonStorage& storage,
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage& kinematicStorage,
    RoboSDP::Selection::Persistence::SelectionJsonStorage& selectionStorage,
    RoboSDP::Logging::ILogger* logger)
    : m_storage(storage)
    , m_kinematic_storage(kinematicStorage)
    , m_selection_storage(selectionStorage)
    , m_logger(logger)
    , m_adapter(logger)
{
}

RoboSDP::Planning::Dto::PlanningWorkspaceStateDto PlanningVerificationService::CreateDefaultState() const
{
    return RoboSDP::Planning::Dto::PlanningWorkspaceStateDto::CreateDefault();
}

PlanningSceneBuildResult PlanningVerificationService::BuildPlanningScene(const QString& projectRootPath) const
{
    PlanningSceneBuildResult result;
    result.state = CreateDefaultState();
    result.kinematic_file_path = m_kinematic_storage.BuildAbsoluteModelFilePath(projectRootPath);
    result.selection_file_path = m_selection_storage.BuildAbsoluteDriveTrainFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法构建 PlanningScene。");
        return result;
    }

    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto kinematicState;
    result.error_code = m_kinematic_storage.LoadModel(projectRootPath, kinematicState);
    if (result.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.message = QStringLiteral("读取 Kinematics 草稿失败，无法构建 PlanningScene：%1")
                             .arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        return result;
    }

    RoboSDP::Selection::Dto::SelectionWorkspaceStateDto selectionState;
    const auto selectionError = m_selection_storage.Load(projectRootPath, selectionState);
    const bool hasSelection = selectionError == RoboSDP::Errors::ErrorCode::Ok;

    result.state.current_scene = BuildSceneFromUpstream(
        kinematicState,
        hasSelection ? &selectionState : nullptr);
    RefreshArtifactRuntimeState(projectRootPath, result.state.current_scene.meta.unified_robot_snapshot);
    result.upstream_snapshot_ready =
        result.state.current_scene.meta.unified_robot_snapshot.pinocchio_model_ready;
    result.upstream_derived_artifact_exists =
        result.state.current_scene.meta.unified_robot_snapshot.derived_artifact_exists;
    result.upstream_derived_artifact_fresh =
        result.state.current_scene.meta.unified_robot_snapshot.derived_artifact_fresh;
    result.upstream_snapshot_summary =
        BuildSnapshotSummary(result.state.current_scene.meta.unified_robot_snapshot);
    if (!result.upstream_snapshot_ready)
    {
        result.message = hasSelection
            ? QStringLiteral("已根据 Kinematics / Selection 草稿生成 PlanningScene，但上游统一主链快照尚未就绪。")
            : QStringLiteral("已根据 Kinematics 草稿生成 PlanningScene，但上游统一主链快照尚未就绪，当前未读取到 Selection 草稿。");
    }
    else if (!result.upstream_derived_artifact_exists)
    {
        result.message = hasSelection
            ? QStringLiteral("已根据 Kinematics / Selection 草稿生成 PlanningScene，但上游派生 URDF 文件缺失。")
            : QStringLiteral("已根据 Kinematics 草稿生成 PlanningScene，但上游派生 URDF 文件缺失，当前未读取到 Selection 草稿。");
    }
    else if (!result.upstream_derived_artifact_fresh)
    {
        result.message = hasSelection
            ? QStringLiteral("已根据 Kinematics / Selection 草稿生成 PlanningScene，但上游派生 URDF 文件不是最新版本。")
            : QStringLiteral("已根据 Kinematics 草稿生成 PlanningScene，但上游派生 URDF 文件不是最新版本，当前未读取到 Selection 草稿。");
    }
    else
    {
        result.message = hasSelection
            ? QStringLiteral("已根据 Kinematics / Selection 草稿生成 PlanningScene，上游统一主链与派生 URDF 文件均已就绪。")
            : QStringLiteral("已根据 Kinematics 草稿生成 PlanningScene，上游统一主链与派生 URDF 文件均已就绪，当前未读取到 Selection 草稿。");
    }
    return result;
}

PlanningRunResult PlanningVerificationService::RunPointToPointVerification(
    const QString& projectRootPath,
    const RoboSDP::Planning::Dto::PlanningWorkspaceStateDto& inputState) const
{
    PlanningRunResult result;
    result.state = inputState;
    result.request_file_path = m_storage.BuildAbsoluteRequestFilePath(projectRootPath);
    result.result_file_path = m_storage.BuildAbsoluteResultFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法执行规划验证。");
        return result;
    }

    const auto request = BuildRequestFromScene(result.state.current_scene);
    const auto response = m_adapter.VerifyPointToPoint(result.state.current_scene, request);
    const auto verificationResult = BuildVerificationResult(result.state.current_scene, request, response);

    result.state.requests.push_back(request);
    result.state.results.push_back(verificationResult);
    result.message = verificationResult.message;
    return result;
}

PlanningSaveResult PlanningVerificationService::SaveDraft(
    const QString& projectRootPath,
    const RoboSDP::Planning::Dto::PlanningWorkspaceStateDto& state) const
{
    PlanningSaveResult result;
    result.scene_file_path = m_storage.BuildAbsoluteSceneFilePath(projectRootPath);
    result.request_file_path = m_storage.BuildAbsoluteRequestFilePath(projectRootPath);
    result.result_file_path = m_storage.BuildAbsoluteResultFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法保存 Planning 草稿。");
        return result;
    }

    result.error_code = m_storage.Save(projectRootPath, state);
    result.message = result.IsSuccess()
        ? QStringLiteral("Planning 草稿已保存。")
        : QStringLiteral("Planning 草稿保存失败：%1").arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
    return result;
}

PlanningLoadResult PlanningVerificationService::LoadDraft(const QString& projectRootPath) const
{
    PlanningLoadResult result;
    result.scene_file_path = m_storage.BuildAbsoluteSceneFilePath(projectRootPath);
    result.request_file_path = m_storage.BuildAbsoluteRequestFilePath(projectRootPath);
    result.result_file_path = m_storage.BuildAbsoluteResultFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法加载 Planning 草稿。");
        return result;
    }

    result.error_code = m_storage.Load(projectRootPath, result.state);
    result.message = result.IsSuccess()
        ? QStringLiteral("Planning 草稿已从 JSON 重新加载。")
        : QStringLiteral("Planning 草稿加载失败：%1").arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
    return result;
}

RoboSDP::Planning::Dto::PlanningSceneDto PlanningVerificationService::BuildSceneFromUpstream(
    const RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& kinematicState,
    const RoboSDP::Selection::Dto::SelectionWorkspaceStateDto* selectionState) const
{
    RoboSDP::Planning::Dto::PlanningSceneDto scene = RoboSDP::Planning::Dto::PlanningSceneDto::CreateDefault();
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
    scene.meta.planning_scene_id = BuildSceneId(kinematicState.current_model.meta.kinematic_id);
    scene.meta.name = QStringLiteral("%1 点到点规划场景").arg(kinematicState.current_model.meta.name);
    scene.meta.kinematic_ref = kinematicState.current_model.meta.kinematic_id;
    scene.meta.requirement_ref = kinematicState.current_model.meta.requirement_ref;
    scene.meta.unified_robot_model_ref = upstreamSnapshot.unified_robot_model_ref;
    scene.meta.kinematic_kernel_ready = upstreamSnapshot.pinocchio_model_ready;
    scene.meta.kinematic_conversion_diagnostics = upstreamSnapshot.conversion_diagnostics;
    scene.meta.unified_robot_snapshot = upstreamSnapshot;
    scene.robot_collision_model_ref =
        QStringLiteral("robot_collision_model_%1").arg(kinematicState.current_model.meta.kinematic_id);

    if (selectionState != nullptr)
    {
        scene.meta.dynamic_ref = selectionState->drive_train_result.dynamic_ref;
        scene.meta.selection_ref = m_selection_storage.RelativeDriveTrainFilePath();
    }

    scene.joint_limits.clear();
    scene.start_state.joint_ids.clear();
    scene.start_state.joint_values_deg.clear();
    scene.goal_state.joint_ids.clear();
    scene.goal_state.joint_values_deg.clear();

    const std::array<double, 6> defaultGoalValues {15.0, -35.0, 45.0, 10.0, 15.0, 20.0};
    for (std::size_t index = 0; index < kinematicState.current_model.joint_limits.size(); ++index)
    {
        const auto& jointLimit = kinematicState.current_model.joint_limits.at(index);
        const double lower = jointLimit.soft_limit[0];
        const double upper = jointLimit.soft_limit[1];
        scene.joint_limits.push_back({jointLimit.joint_id, lower, upper});
        scene.start_state.joint_ids.push_back(jointLimit.joint_id);
        scene.goal_state.joint_ids.push_back(jointLimit.joint_id);
        scene.start_state.joint_values_deg.push_back(ClampToRange(0.0, lower, upper));

        const double defaultGoal = index < defaultGoalValues.size()
            ? defaultGoalValues.at(index)
            : (lower + upper) * 0.25;
        scene.goal_state.joint_values_deg.push_back(ClampToRange(defaultGoal, lower, upper));
    }

    return scene;
}

RoboSDP::Planning::Dto::PlanningRequestDto PlanningVerificationService::BuildRequestFromScene(
    const RoboSDP::Planning::Dto::PlanningSceneDto& scene) const
{
    RoboSDP::Planning::Dto::PlanningRequestDto request = RoboSDP::Planning::Dto::PlanningRequestDto::CreateDefault();
    request.request_id = BuildRequestId();
    request.planning_scene_ref = scene.meta.planning_scene_id;
    request.motion_group = scene.planning_config.motion_group;
    request.planner_id = scene.planning_config.planner_id;
    request.service_endpoint = scene.planning_config.service_endpoint;
    request.allowed_planning_time_s = scene.planning_config.allowed_planning_time_s;
    request.target_cycle_time_s = scene.planning_config.target_cycle_time_s;
    request.check_collision = scene.planning_config.check_collision;
    request.check_self_collision = scene.planning_config.check_self_collision;
    request.joint_ids = scene.start_state.joint_ids;
    request.start_joint_values_deg = scene.start_state.joint_values_deg;
    request.goal_joint_values_deg = scene.goal_state.joint_values_deg;
    return request;
}

RoboSDP::Planning::Dto::PlanningVerificationResultDto PlanningVerificationService::BuildVerificationResult(
    const RoboSDP::Planning::Dto::PlanningSceneDto& scene,
    const RoboSDP::Planning::Dto::PlanningRequestDto& request,
    const RoboSDP::Planning::Dto::PlanningResponseDto& response) const
{
    RoboSDP::Planning::Dto::PlanningVerificationResultDto result =
        RoboSDP::Planning::Dto::PlanningVerificationResultDto::CreateDefault();
    result.verification_id = BuildVerificationId();
    result.planning_scene_ref = scene.meta.planning_scene_id;
    result.request_id = request.request_id;
    result.success = response.transport_success && response.planning_success;
    result.message = response.message;
    result.cycle_time_result = response.cycle_time_result;
    result.collision_results = response.collision_results;
    result.self_collision_results = response.self_collision_results;

    if (!response.trajectory_result.request_id.isEmpty())
    {
        result.trajectory_results.push_back(response.trajectory_result);
    }

    result.feasibility_summary = result.success
        ? QStringLiteral("点到点规划验证通过，当前场景未发现碰撞且节拍已评估。")
        : QStringLiteral("点到点规划验证失败，请检查限位、碰撞或节拍约束。");
    return result;
}

} // namespace RoboSDP::Planning::Service
