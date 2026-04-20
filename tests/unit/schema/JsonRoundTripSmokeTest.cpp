#include "core/repository/LocalJsonRepository.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/planning/persistence/PlanningJsonStorage.h"
#include "modules/requirement/persistence/RequirementJsonStorage.h"
#include "modules/scheme/persistence/SchemeJsonStorage.h"
#include "modules/selection/persistence/SelectionJsonStorage.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

#include <QCoreApplication>
#include <QDir>
#include <QJsonObject>

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    const QString projectRoot =
        QDir::current().absoluteFilePath(QStringLiteral("json-roundtrip-smoke-project"));

    RoboSDP::Repository::LocalJsonRepository repository;
    RoboSDP::Requirement::Persistence::RequirementJsonStorage requirementStorage(repository);
    RoboSDP::Topology::Persistence::TopologyJsonStorage topologyStorage(repository);
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage kinematicStorage(repository);
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage dynamicStorage(repository);
    RoboSDP::Selection::Persistence::SelectionJsonStorage selectionStorage(repository);
    RoboSDP::Planning::Persistence::PlanningJsonStorage planningStorage(repository);
    RoboSDP::Scheme::Persistence::SchemeJsonStorage schemeStorage(repository);

    auto requirementModel = RoboSDP::Requirement::Dto::RequirementModelDto::CreateDefault();
    requirementModel.project_meta.project_name = QStringLiteral("RoundTrip Project");
    if (requirementStorage.Save(projectRoot, requirementModel) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 1;
    }

    RoboSDP::Requirement::Dto::RequirementModelDto loadedRequirement;
    if (requirementStorage.Load(projectRoot, loadedRequirement) != RoboSDP::Errors::ErrorCode::Ok ||
        loadedRequirement.project_meta.project_name != requirementModel.project_meta.project_name)
    {
        return 2;
    }

    auto topologyState = RoboSDP::Topology::Dto::TopologyWorkspaceStateDto::CreateDefault();
    topologyState.current_model.meta.requirement_ref = QStringLiteral("requirements/requirement-model.json");
    if (topologyStorage.Save(projectRoot, topologyState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 3;
    }

    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto loadedTopology;
    if (topologyStorage.Load(projectRoot, loadedTopology) != RoboSDP::Errors::ErrorCode::Ok ||
        loadedTopology.current_model.meta.topology_id != topologyState.current_model.meta.topology_id)
    {
        return 4;
    }

    auto kinematicsState = RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto::CreateDefault();
    kinematicsState.current_model.meta.topology_ref = topologyState.current_model.meta.topology_id;
    if (kinematicStorage.SaveModel(projectRoot, kinematicsState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 5;
    }

    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto loadedKinematics;
    if (kinematicStorage.LoadModel(projectRoot, loadedKinematics) != RoboSDP::Errors::ErrorCode::Ok ||
        loadedKinematics.current_model.meta.kinematic_id != kinematicsState.current_model.meta.kinematic_id)
    {
        return 6;
    }

    RoboSDP::Kinematics::Dto::WorkspaceResultDto workspaceResult;
    workspaceResult.success = true;
    workspaceResult.requested_sample_count = 16;
    workspaceResult.reachable_sample_count = 12;
    workspaceResult.max_radius_m = 1.35;
    if (kinematicStorage.SaveWorkspaceCache(projectRoot, workspaceResult) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 7;
    }

    RoboSDP::Kinematics::Dto::WorkspaceResultDto loadedWorkspace;
    if (kinematicStorage.LoadWorkspaceCache(projectRoot, loadedWorkspace) != RoboSDP::Errors::ErrorCode::Ok ||
        loadedWorkspace.reachable_sample_count != workspaceResult.reachable_sample_count)
    {
        return 8;
    }

    auto dynamicsState = RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto::CreateDefault();
    dynamicsState.current_model.meta.kinematic_ref = kinematicsState.current_model.meta.kinematic_id;
    if (dynamicStorage.Save(projectRoot, dynamicsState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 9;
    }

    RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto loadedDynamics;
    if (dynamicStorage.Load(projectRoot, loadedDynamics) != RoboSDP::Errors::ErrorCode::Ok ||
        loadedDynamics.current_model.meta.dynamic_id != dynamicsState.current_model.meta.dynamic_id)
    {
        return 10;
    }

    auto selectionState = RoboSDP::Selection::Dto::SelectionWorkspaceStateDto::CreateDefault();
    selectionState.drive_train_result.dynamic_ref = dynamicsState.current_model.meta.dynamic_id;
    selectionState.drive_train_result.success = true;
    RoboSDP::Selection::Dto::DriveTrainJointSelectionDto jointSelection;
    jointSelection.joint_id = QStringLiteral("joint_1");
    selectionState.drive_train_result.joint_selections.push_back(jointSelection);
    if (selectionStorage.Save(projectRoot, selectionState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 11;
    }

    RoboSDP::Selection::Dto::SelectionWorkspaceStateDto loadedSelection;
    if (selectionStorage.Load(projectRoot, loadedSelection) != RoboSDP::Errors::ErrorCode::Ok ||
        loadedSelection.drive_train_result.joint_selections.size() != 1)
    {
        return 12;
    }

    auto planningState = RoboSDP::Planning::Dto::PlanningWorkspaceStateDto::CreateDefault();
    planningState.current_scene.meta.dynamic_ref = dynamicsState.current_model.meta.dynamic_id;
    if (planningStorage.Save(projectRoot, planningState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 13;
    }

    RoboSDP::Planning::Dto::PlanningWorkspaceStateDto loadedPlanning;
    if (planningStorage.Load(projectRoot, loadedPlanning) != RoboSDP::Errors::ErrorCode::Ok ||
        loadedPlanning.current_scene.meta.planning_scene_id != planningState.current_scene.meta.planning_scene_id)
    {
        return 14;
    }

    auto snapshot = RoboSDP::Scheme::Dto::SchemeSnapshotDto::CreateDefault();
    snapshot.aggregate.available_module_count = 7;
    if (schemeStorage.SaveSnapshot(projectRoot, snapshot) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 15;
    }

    RoboSDP::Scheme::Dto::SchemeSnapshotDto loadedSnapshot;
    if (schemeStorage.LoadSnapshot(projectRoot, loadedSnapshot) != RoboSDP::Errors::ErrorCode::Ok ||
        loadedSnapshot.aggregate.available_module_count != snapshot.aggregate.available_module_count)
    {
        return 16;
    }

    auto exportDto = RoboSDP::Scheme::Dto::SchemeExportDto::CreateDefault();
    exportDto.scheme_id = snapshot.meta.scheme_id;
    exportDto.output_file_path = QDir(projectRoot).filePath(QStringLiteral("exports/scheme-export.json"));
    exportDto.success = true;
    if (schemeStorage.SaveExportJson(projectRoot, exportDto) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 17;
    }

    if (repository.OpenProject(projectRoot) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 18;
    }

    QJsonObject exportDocument;
    if (repository.ReadDocument(QStringLiteral("exports/scheme-export.json"), exportDocument) !=
            RoboSDP::Errors::ErrorCode::Ok ||
        exportDocument.value(QStringLiteral("scheme_id")).toString() != exportDto.scheme_id)
    {
        return 19;
    }

    return 0;
}
