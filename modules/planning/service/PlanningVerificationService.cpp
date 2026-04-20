#include "modules/planning/service/PlanningVerificationService.h"

#include <QDateTime>

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
    result.message = hasSelection
        ? QStringLiteral("已根据 Kinematics / Selection 草稿生成 PlanningScene。")
        : QStringLiteral("已根据 Kinematics 草稿生成 PlanningScene，当前未读取到 Selection 草稿。");
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
    scene.meta.planning_scene_id = BuildSceneId(kinematicState.current_model.meta.kinematic_id);
    scene.meta.name = QStringLiteral("%1 点到点规划场景").arg(kinematicState.current_model.meta.name);
    scene.meta.kinematic_ref = kinematicState.current_model.meta.kinematic_id;
    scene.meta.requirement_ref = kinematicState.current_model.meta.requirement_ref;
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
