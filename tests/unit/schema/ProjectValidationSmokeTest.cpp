#include "core/repository/LocalJsonRepository.h"
#include "core/schema/ProjectReferenceChecker.h"
#include "core/schema/ProjectStructureChecker.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/planning/persistence/PlanningJsonStorage.h"
#include "modules/requirement/persistence/RequirementJsonStorage.h"
#include "modules/scheme/persistence/SchemeJsonStorage.h"
#include "modules/selection/persistence/SelectionJsonStorage.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

#include <QCoreApplication>
#include <QDir>

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    const QString projectRoot =
        QDir::current().absoluteFilePath(QStringLiteral("project-validation-smoke-project"));

    RoboSDP::Repository::LocalJsonRepository repository;
    RoboSDP::Requirement::Persistence::RequirementJsonStorage requirementStorage(repository);
    RoboSDP::Topology::Persistence::TopologyJsonStorage topologyStorage(repository);
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage kinematicStorage(repository);
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage dynamicStorage(repository);
    RoboSDP::Selection::Persistence::SelectionJsonStorage selectionStorage(repository);
    RoboSDP::Planning::Persistence::PlanningJsonStorage planningStorage(repository);
    RoboSDP::Scheme::Persistence::SchemeJsonStorage schemeStorage(repository);

    auto requirementModel = RoboSDP::Requirement::Dto::RequirementModelDto::CreateDefault();
    requirementModel.project_meta.project_name = QStringLiteral("Project Validation Smoke");
    if (requirementStorage.Save(projectRoot, requirementModel) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 1;
    }

    auto topologyState = RoboSDP::Topology::Dto::TopologyWorkspaceStateDto::CreateDefault();
    topologyState.current_model.meta.requirement_ref = QStringLiteral("requirements/requirement-model.json");
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
    workspaceResult.message = QStringLiteral("project validation workspace");
    workspaceResult.requested_sample_count = 4;
    workspaceResult.reachable_sample_count = 4;
    workspaceResult.max_radius_m = 1.0;
    if (kinematicStorage.SaveWorkspaceCache(projectRoot, workspaceResult) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 4;
    }

    auto dynamicsState = RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto::CreateDefault();
    dynamicsState.current_model.meta.kinematic_ref = kinematicsState.current_model.meta.kinematic_id;
    dynamicsState.current_model.meta.topology_ref = topologyState.current_model.meta.topology_id;
    dynamicsState.current_model.meta.requirement_ref = QStringLiteral("requirements/requirement-model.json");
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
    selectionState.drive_train_result.message = QStringLiteral("project validation selection");
    RoboSDP::Selection::Dto::DriveTrainJointSelectionDto jointSelection;
    jointSelection.joint_id = QStringLiteral("joint_1");
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
    planningResult.verification_id = QStringLiteral("planning_verification_smoke");
    planningState.results.push_back(planningResult);
    if (planningStorage.Save(projectRoot, planningState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 7;
    }

    auto schemeSnapshot = RoboSDP::Scheme::Dto::SchemeSnapshotDto::CreateDefault();
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
    if (schemeStorage.SaveExportJson(projectRoot, exportDto) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 9;
    }

    RoboSDP::Schema::ProjectStructureChecker structureChecker;
    const auto structureResult = structureChecker.CheckProject(projectRoot);
    if (!structureResult.IsValid() || structureResult.WarningCount() != 0)
    {
        return 10;
    }

    RoboSDP::Schema::ProjectReferenceChecker referenceChecker(repository);
    const auto referenceResult = referenceChecker.CheckProject(projectRoot);
    if (!referenceResult.IsValid() || referenceResult.WarningCount() != 0)
    {
        return 11;
    }

    return 0;
}
