#include "core/repository/LocalJsonRepository.h"
#include "core/schema/ModuleValidationRegistry.h"
#include "core/schema/SchemaDtoConsistencyChecker.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/planning/persistence/PlanningJsonStorage.h"
#include "modules/requirement/persistence/RequirementJsonStorage.h"
#include "modules/scheme/persistence/SchemeJsonStorage.h"
#include "modules/selection/persistence/SelectionJsonStorage.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

#include <QCoreApplication>
#include <QDir>
#include <QTextStream>

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    const QString projectRoot =
        QDir::current().absoluteFilePath(QStringLiteral("schema-smoke-project"));

    RoboSDP::Repository::LocalJsonRepository repository;
    RoboSDP::Requirement::Persistence::RequirementJsonStorage requirementStorage(repository);
    RoboSDP::Topology::Persistence::TopologyJsonStorage topologyStorage(repository);
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage kinematicStorage(repository);
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage dynamicStorage(repository);
    RoboSDP::Selection::Persistence::SelectionJsonStorage selectionStorage(repository);
    RoboSDP::Planning::Persistence::PlanningJsonStorage planningStorage(repository);
    RoboSDP::Scheme::Persistence::SchemeJsonStorage schemeStorage(repository);

    auto requirementModel = RoboSDP::Requirement::Dto::RequirementModelDto::CreateDefault();
    requirementModel.project_meta.project_name = QStringLiteral("Schema Smoke Project");
    // 中文说明：schema smoke 需要显式填入当前 validator 要求的最小合法需求值，
    // 否则会因为默认 DTO 仍保留“待用户填写”的 0 值而被统一校验拦截。
    requirementModel.workspace_requirements.max_radius = 1.6;
    requirementModel.workspace_requirements.min_radius = 0.2;
    requirementModel.workspace_requirements.max_height = 1.8;
    requirementModel.workspace_requirements.min_height = 0.1;
    requirementModel.motion_requirements.max_linear_speed = 1.2;
    requirementModel.motion_requirements.max_angular_speed = 180.0;
    requirementModel.motion_requirements.max_acceleration = 2.0;
    requirementModel.motion_requirements.max_angular_acceleration = 360.0;
    requirementModel.motion_requirements.jerk_limit = 5.0;
    requirementModel.motion_requirements.takt_time = 4.0;
    if (requirementStorage.Save(projectRoot, requirementModel) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 1;
    }

    auto topologyState = RoboSDP::Topology::Dto::TopologyWorkspaceStateDto::CreateDefault();
    topologyState.current_model.meta.requirement_ref = QStringLiteral("requirements/requirement-model.json");
    topologyState.current_model.meta.template_id = QStringLiteral("topology_template_schema_smoke");
    if (topologyStorage.Save(projectRoot, topologyState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 2;
    }

    auto kinematicsState = RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto::CreateDefault();
    kinematicsState.current_model.meta.topology_ref = topologyState.current_model.meta.topology_id;
    kinematicsState.current_model.meta.requirement_ref = QStringLiteral("requirements/requirement-model.json");
    if (kinematicStorage.SaveModel(projectRoot, kinematicsState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 3;
    }

    RoboSDP::Kinematics::Dto::WorkspaceResultDto workspaceResult;
    workspaceResult.success = true;
    workspaceResult.message = QStringLiteral("schema smoke workspace");
    workspaceResult.requested_sample_count = 8;
    workspaceResult.reachable_sample_count = 8;
    workspaceResult.max_radius_m = 1.2;
    if (kinematicStorage.SaveWorkspaceCache(projectRoot, workspaceResult) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 4;
    }

    auto dynamicsState = RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto::CreateDefault();
    dynamicsState.current_model.meta.kinematic_ref = kinematicsState.current_model.meta.kinematic_id;
    dynamicsState.current_model.meta.topology_ref = topologyState.current_model.meta.topology_id;
    dynamicsState.current_model.meta.requirement_ref = QStringLiteral("requirements/requirement-model.json");
    RoboSDP::Dynamics::Dto::BenchmarkTrajectoryDto benchmarkTrajectory;
    benchmarkTrajectory.trajectory_id = QStringLiteral("dynamic_trajectory_schema_smoke");
    benchmarkTrajectory.name = QStringLiteral("Schema Smoke Trajectory");
    benchmarkTrajectory.profile_type = QStringLiteral("trapezoid");
    benchmarkTrajectory.active_joint_id = QStringLiteral("joint_1");
    benchmarkTrajectory.joint_start_deg = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    benchmarkTrajectory.joint_target_deg = {15.0, -20.0, 25.0, 0.0, 10.0, 5.0};
    benchmarkTrajectory.duration_s = 1.2;
    benchmarkTrajectory.sample_count = 32;
    dynamicsState.current_model.trajectories.push_back(benchmarkTrajectory);
    if (dynamicStorage.Save(projectRoot, dynamicsState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 5;
    }

    auto selectionState = RoboSDP::Selection::Dto::SelectionWorkspaceStateDto::CreateDefault();
    selectionState.drive_train_result.dynamic_ref = dynamicsState.current_model.meta.dynamic_id;
    selectionState.drive_train_result.kinematic_ref = kinematicsState.current_model.meta.kinematic_id;
    selectionState.drive_train_result.topology_ref = topologyState.current_model.meta.topology_id;
    selectionState.drive_train_result.requirement_ref = QStringLiteral("requirements/requirement-model.json");
    selectionState.drive_train_result.success = true;
    selectionState.drive_train_result.message = QStringLiteral("schema smoke selection");
    RoboSDP::Selection::Dto::DriveTrainJointSelectionDto jointSelection;
    jointSelection.joint_id = QStringLiteral("joint_1");
    jointSelection.has_recommendation = true;
    jointSelection.message = QStringLiteral("ok");
    selectionState.drive_train_result.joint_selections.push_back(jointSelection);
    if (selectionStorage.Save(projectRoot, selectionState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 6;
    }

    auto planningState = RoboSDP::Planning::Dto::PlanningWorkspaceStateDto::CreateDefault();
    planningState.current_scene.meta.kinematic_ref = kinematicsState.current_model.meta.kinematic_id;
    planningState.current_scene.meta.dynamic_ref = dynamicsState.current_model.meta.dynamic_id;
    planningState.current_scene.meta.selection_ref = QStringLiteral("selection/drivetrain-selection.json");
    planningState.current_scene.meta.requirement_ref = QStringLiteral("requirements/requirement-model.json");
    RoboSDP::Planning::Dto::PlanningVerificationResultDto planningResult;
    planningResult.verification_id = QStringLiteral("planning_verification_schema_smoke");
    planningState.results.push_back(planningResult);
    if (planningStorage.Save(projectRoot, planningState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 7;
    }

    auto schemeSnapshot = RoboSDP::Scheme::Dto::SchemeSnapshotDto::CreateDefault();
    schemeSnapshot.meta.source_project_root = projectRoot;
    schemeSnapshot.aggregate.refs.requirement_ref = QStringLiteral("requirements/requirement-model.json");
    schemeSnapshot.aggregate.refs.topology_ref = topologyState.current_model.meta.topology_id;
    schemeSnapshot.aggregate.refs.kinematic_ref = kinematicsState.current_model.meta.kinematic_id;
    schemeSnapshot.aggregate.refs.dynamic_ref = dynamicsState.current_model.meta.dynamic_id;
    schemeSnapshot.aggregate.refs.selection_ref = QStringLiteral("selection/drivetrain-selection.json");
    schemeSnapshot.aggregate.refs.planning_scene_ref = planningState.current_scene.meta.planning_scene_id;
    schemeSnapshot.aggregate.refs.planning_result_ref = planningResult.verification_id;
    schemeSnapshot.aggregate.available_module_count = 7;
    if (schemeStorage.SaveSnapshot(projectRoot, schemeSnapshot) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 8;
    }

    auto exportDto = RoboSDP::Scheme::Dto::SchemeExportDto::CreateDefault();
    exportDto.scheme_id = schemeSnapshot.meta.scheme_id;
    exportDto.output_file_path = QDir(projectRoot).filePath(QStringLiteral("exports/scheme-export.json"));
    exportDto.success = true;
    exportDto.available_module_count = 7;
    exportDto.executive_summary = QStringLiteral("schema smoke export");
    if (schemeStorage.SaveExportJson(projectRoot, exportDto) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 9;
    }

    RoboSDP::Schema::ModuleValidationRegistry registry(repository);
    const auto validationResult = registry.ValidateProject(projectRoot);
    if (!validationResult.IsValid())
    {
        QTextStream(stderr) << "[schema_smoke] module validation failed\n";
        for (const auto& issue : validationResult.messages)
        {
            QTextStream(stderr)
                << issue.field << " | " << issue.code << " | " << issue.message_zh << "\n";
        }
        return 10;
    }

    RoboSDP::Schema::SchemaDtoConsistencyChecker checker(repository);
    const auto consistencyResult = checker.CheckProject(projectRoot);
    if (!consistencyResult.IsValid())
    {
        return 11;
    }

    return 0;
}
