#include "core/repository/LocalJsonRepository.h"
#include "modules/dynamics/dto/DynamicsResultDto.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/kinematics/dto/KinematicSolverResultDto.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/planning/dto/PlanningVerificationResultDto.h"
#include "modules/planning/persistence/PlanningJsonStorage.h"
#include "modules/requirement/dto/RequirementModelDto.h"
#include "modules/requirement/persistence/RequirementJsonStorage.h"
#include "modules/scheme/persistence/SchemeJsonStorage.h"
#include "modules/scheme/service/SchemeExportService.h"
#include "modules/scheme/service/SchemeSnapshotService.h"
#include "modules/selection/dto/DriveTrainSelectionDto.h"
#include "modules/selection/persistence/SelectionJsonStorage.h"
#include "modules/topology/dto/TopologyRecommendationDto.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

#include <QCoreApplication>
#include <QFileInfo>
#include <QTemporaryDir>

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    QTemporaryDir tempProjectDirectory;
    if (!tempProjectDirectory.isValid())
    {
        return 1;
    }

    const QString projectRoot = tempProjectDirectory.path();

    RoboSDP::Repository::LocalJsonRepository repository;
    RoboSDP::Requirement::Persistence::RequirementJsonStorage requirementStorage(repository);
    RoboSDP::Topology::Persistence::TopologyJsonStorage topologyStorage(repository);
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage kinematicStorage(repository);
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage dynamicStorage(repository);
    RoboSDP::Selection::Persistence::SelectionJsonStorage selectionStorage(repository);
    RoboSDP::Planning::Persistence::PlanningJsonStorage planningStorage(repository);
    RoboSDP::Scheme::Persistence::SchemeJsonStorage schemeStorage(repository);

    RoboSDP::Requirement::Dto::RequirementModelDto requirementModel =
        RoboSDP::Requirement::Dto::RequirementModelDto::CreateDefault();
    requirementModel.project_meta.project_name = QStringLiteral("Scheme Smoke Project");
    requirementModel.project_meta.scenario_type = QStringLiteral("welding");
    requirementModel.load_requirements.rated_payload = 8.5;
    requirementModel.load_requirements.max_payload = 10.0;
    requirementModel.motion_requirements.takt_time = 4.2;
    if (requirementStorage.Save(projectRoot, requirementModel) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 2;
    }

    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto topologyState =
        RoboSDP::Topology::Dto::TopologyWorkspaceStateDto::CreateDefault();
    topologyState.current_model.meta.topology_id = QStringLiteral("topology_scheme_smoke");
    topologyState.current_model.meta.template_id = QStringLiteral("6r-floor-general");
    topologyState.current_model.meta.requirement_ref = QStringLiteral("requirements/requirement-model.json");
    topologyState.recommendation.recommended_candidate_id = QStringLiteral("candidate_001");
    topologyState.candidates.push_back({
        QStringLiteral("candidate_001"),
        QStringLiteral("6R 串联标准构型"),
        QStringLiteral("6r-floor-general"),
        88.0,
        true,
        {QStringLiteral("满足阶段 1 烟雾测试要求。")},
        topologyState.current_model});
    if (topologyStorage.Save(projectRoot, topologyState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 3;
    }

    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto kinematicState =
        RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto::CreateDefault();
    kinematicState.current_model.meta.kinematic_id = QStringLiteral("kinematic_scheme_smoke");
    kinematicState.current_model.meta.topology_ref = QStringLiteral("topology_scheme_smoke");
    kinematicState.current_model.meta.requirement_ref = QStringLiteral("requirements/requirement-model.json");
    kinematicState.last_fk_result.success = true;
    kinematicState.last_ik_result.success = true;
    kinematicState.last_workspace_result.success = true;
    kinematicState.last_workspace_result.reachable_sample_count = 24;
    kinematicState.last_workspace_result.max_radius_m = 1.26;
    if (kinematicStorage.SaveModel(projectRoot, kinematicState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 4;
    }
    if (kinematicStorage.SaveWorkspaceCache(projectRoot, kinematicState.last_workspace_result) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 5;
    }

    RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto dynamicsState =
        RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto::CreateDefault();
    dynamicsState.current_model.meta.dynamic_id = QStringLiteral("dynamic_scheme_smoke");
    dynamicsState.current_model.meta.kinematic_ref = QStringLiteral("kinematic_scheme_smoke");
    dynamicsState.current_model.meta.topology_ref = QStringLiteral("topology_scheme_smoke");
    dynamicsState.current_model.meta.requirement_ref = QStringLiteral("requirements/requirement-model.json");
    dynamicsState.parameterized_trajectories.push_back(
        {QStringLiteral("traj_001"), QStringLiteral("P2P"), 1.8, {}});
    dynamicsState.peak_stats.push_back(
        {QStringLiteral("joint_1"), QStringLiteral("traj_001"), 82.5, 90.0, 380.0});
    dynamicsState.rms_stats.push_back(
        {QStringLiteral("joint_1"), QStringLiteral("traj_001"), 35.2});
    dynamicsState.load_envelope.joints.push_back(
        {QStringLiteral("joint_1"), QStringLiteral("traj_001"), 82.5, 35.2, 380.0});
    if (dynamicStorage.Save(projectRoot, dynamicsState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 6;
    }

    RoboSDP::Selection::Dto::SelectionWorkspaceStateDto selectionState =
        RoboSDP::Selection::Dto::SelectionWorkspaceStateDto::CreateDefault();
    selectionState.drive_train_result.dynamic_ref = QStringLiteral("dynamic_scheme_smoke");
    selectionState.drive_train_result.kinematic_ref = QStringLiteral("kinematic_scheme_smoke");
    selectionState.drive_train_result.topology_ref = QStringLiteral("topology_scheme_smoke");
    selectionState.drive_train_result.requirement_ref = QStringLiteral("requirements/requirement-model.json");
    selectionState.drive_train_result.success = true;
    RoboSDP::Selection::Dto::DriveTrainJointSelectionDto jointSelection;
    jointSelection.joint_id = QStringLiteral("joint_1");
    jointSelection.has_recommendation = true;
    jointSelection.recommended.motor_id = QStringLiteral("motor_a");
    jointSelection.recommended.reducer_id = QStringLiteral("reducer_a");
    jointSelection.recommended.recommendation_reason = QStringLiteral("满足最小闭环要求。");
    selectionState.drive_train_result.joint_selections.push_back(jointSelection);
    if (selectionStorage.Save(projectRoot, selectionState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 7;
    }

    RoboSDP::Planning::Dto::PlanningWorkspaceStateDto planningState =
        RoboSDP::Planning::Dto::PlanningWorkspaceStateDto::CreateDefault();
    planningState.current_scene.meta.planning_scene_id = QStringLiteral("planning_scene_scheme_smoke");
    planningState.current_scene.meta.kinematic_ref = QStringLiteral("kinematic_scheme_smoke");
    planningState.current_scene.meta.dynamic_ref = QStringLiteral("dynamic_scheme_smoke");
    planningState.current_scene.meta.selection_ref = QStringLiteral("selection/drivetrain-selection.json");
    planningState.current_scene.meta.requirement_ref = QStringLiteral("requirements/requirement-model.json");
    planningState.requests.push_back(RoboSDP::Planning::Dto::PlanningRequestDto::CreateDefault());
    RoboSDP::Planning::Dto::PlanningVerificationResultDto planningResult =
        RoboSDP::Planning::Dto::PlanningVerificationResultDto::CreateDefault();
    planningResult.verification_id = QStringLiteral("planning_result_001");
    planningResult.planning_scene_ref = QStringLiteral("planning_scene_scheme_smoke");
    planningResult.success = true;
    planningResult.cycle_time_result.within_target = true;
    planningResult.feasibility_summary = QStringLiteral("点到点规划验证通过。");
    planningState.results.push_back(planningResult);
    if (planningStorage.Save(projectRoot, planningState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 8;
    }

    RoboSDP::Scheme::Service::SchemeSnapshotService snapshotService(
        schemeStorage,
        requirementStorage,
        topologyStorage,
        kinematicStorage,
        dynamicStorage,
        selectionStorage,
        planningStorage,
        nullptr);

    const auto buildResult = snapshotService.BuildSnapshot(projectRoot);
    if (!buildResult.IsSuccess() || buildResult.snapshot.aggregate.available_module_count != 6)
    {
        return 9;
    }

    const auto saveResult = snapshotService.SaveSnapshot(projectRoot, buildResult.snapshot);
    if (!saveResult.IsSuccess())
    {
        return 10;
    }

    const auto loadResult = snapshotService.LoadSnapshot(projectRoot);
    if (!loadResult.IsSuccess() ||
        loadResult.snapshot.aggregate.available_module_count != 6 ||
        loadResult.snapshot.aggregate.refs.topology_ref != QStringLiteral("topology_scheme_smoke"))
    {
        return 11;
    }

    RoboSDP::Scheme::Service::SchemeExportService exportService(schemeStorage, nullptr);
    const auto exportResult = exportService.ExportAsJson(projectRoot, loadResult.snapshot);
    if (!exportResult.IsSuccess() || !QFileInfo::exists(exportResult.export_file_path))
    {
        return 12;
    }

    return 0;
}
