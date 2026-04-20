#include "core/repository/LocalJsonRepository.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/dynamics/service/DynamicsService.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/planning/persistence/PlanningJsonStorage.h"
#include "modules/requirement/persistence/RequirementJsonStorage.h"
#include "modules/scheme/persistence/SchemeJsonStorage.h"
#include "modules/scheme/service/SchemeExportService.h"
#include "modules/scheme/service/SchemeSnapshotService.h"
#include "modules/selection/persistence/SelectionJsonStorage.h"
#include "modules/selection/service/DriveTrainMatchingService.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

#include <QCoreApplication>
#include <QFileInfo>
#include <QTemporaryDir>

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    QTemporaryDir temporaryProjectDirectory;
    if (!temporaryProjectDirectory.isValid())
    {
        return 1;
    }

    const QString projectRoot = temporaryProjectDirectory.path();

    RoboSDP::Repository::LocalJsonRepository repository;
    RoboSDP::Requirement::Persistence::RequirementJsonStorage requirementStorage(repository);
    RoboSDP::Topology::Persistence::TopologyJsonStorage topologyStorage(repository);
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage kinematicStorage(repository);
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage dynamicStorage(repository);
    RoboSDP::Selection::Persistence::SelectionJsonStorage selectionStorage(repository);
    RoboSDP::Planning::Persistence::PlanningJsonStorage planningStorage(repository);
    RoboSDP::Scheme::Persistence::SchemeJsonStorage schemeStorage(repository);

    // 先准备 Requirement/Topology/Kinematics 三个上游输入，保持第一阶段主链的最小真实装配。
    RoboSDP::Requirement::Dto::RequirementModelDto requirementModel =
        RoboSDP::Requirement::Dto::RequirementModelDto::CreateDefault();
    requirementModel.project_meta.project_name = QStringLiteral("Integration Main Chain Project");
    requirementModel.project_meta.scenario_type = QStringLiteral("welding");
    requirementModel.load_requirements.rated_payload = 8.5;
    requirementModel.load_requirements.max_payload = 10.0;
    requirementModel.motion_requirements.takt_time = 4.2;
    requirementModel.workspace_requirements.max_radius = 1.5;
    if (requirementStorage.Save(projectRoot, requirementModel) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 2;
    }

    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto topologyState =
        RoboSDP::Topology::Dto::TopologyWorkspaceStateDto::CreateDefault();
    topologyState.current_model.meta.topology_id = QStringLiteral("topology_mainchain_integration");
    topologyState.current_model.meta.name = QStringLiteral("6R 主链集成测试构型");
    topologyState.current_model.meta.template_id = QStringLiteral("6r-floor-general");
    topologyState.current_model.meta.requirement_ref = QStringLiteral("requirements/requirement-model.json");
    topologyState.recommendation.recommended_candidate_id = QStringLiteral("candidate_mainchain_001");
    topologyState.candidates.push_back({
        QStringLiteral("candidate_mainchain_001"),
        QStringLiteral("6R 串联标准构型"),
        QStringLiteral("6r-floor-general"),
        88.0,
        true,
        {QStringLiteral("满足第一阶段主链集成测试要求。")},
        topologyState.current_model});
    if (topologyStorage.Save(projectRoot, topologyState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 3;
    }

    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto kinematicState =
        RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto::CreateDefault();
    kinematicState.current_model.meta.kinematic_id = QStringLiteral("kinematic_mainchain_integration");
    kinematicState.current_model.meta.topology_ref = topologyState.current_model.meta.topology_id;
    kinematicState.current_model.meta.requirement_ref = QStringLiteral("requirements/requirement-model.json");
    kinematicState.last_fk_result.success = true;
    kinematicState.last_ik_result.success = true;
    kinematicState.last_workspace_result.success = true;
    kinematicState.last_workspace_result.requested_sample_count = 24;
    kinematicState.last_workspace_result.reachable_sample_count = 24;
    kinematicState.last_workspace_result.max_radius_m = 1.26;
    if (kinematicStorage.SaveModel(projectRoot, kinematicState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 4;
    }
    if (kinematicStorage.SaveWorkspaceCache(projectRoot, kinematicState.last_workspace_result) !=
        RoboSDP::Errors::ErrorCode::Ok)
    {
        return 5;
    }

    // Dynamics 通过 service 从 Kinematics 草稿真实构建，再运行最小逆动力学链。
    RoboSDP::Dynamics::Service::DynamicsService dynamicsService(dynamicStorage, kinematicStorage, nullptr);
    const auto dynamicsBuildResult = dynamicsService.BuildFromKinematics(projectRoot);
    if (!dynamicsBuildResult.IsSuccess() ||
        dynamicsBuildResult.state.current_model.meta.kinematic_ref !=
            QStringLiteral("kinematic_mainchain_integration"))
    {
        return 6;
    }

    const auto dynamicsAnalyzeResult =
        dynamicsService.RunInverseDynamicsChain(projectRoot, dynamicsBuildResult.state);
    if (!dynamicsAnalyzeResult.IsSuccess() || dynamicsAnalyzeResult.state.load_envelope.joints.empty())
    {
        return 7;
    }

    const auto dynamicsSaveResult = dynamicsService.SaveDraft(projectRoot, dynamicsAnalyzeResult.state);
    if (!dynamicsSaveResult.IsSuccess() ||
        !QFileInfo::exists(dynamicStorage.BuildAbsoluteEnvelopeFilePath(projectRoot)))
    {
        return 8;
    }

    // Selection 直接消费 Dynamics 草稿，验证主链能产出最小联合驱动推荐。
    RoboSDP::Selection::Service::DriveTrainMatchingService selectionService(
        selectionStorage,
        dynamicStorage,
        nullptr);
    const auto selectionRunResult = selectionService.RunSelection(projectRoot);
    if (!selectionRunResult.IsSuccess() ||
        !selectionRunResult.state.drive_train_result.success ||
        selectionRunResult.state.motor_results.empty() ||
        selectionRunResult.state.reducer_results.empty())
    {
        return 9;
    }

    const auto selectionSaveResult = selectionService.SaveDraft(projectRoot, selectionRunResult.state);
    if (!selectionSaveResult.IsSuccess() ||
        !QFileInfo::exists(selectionStorage.BuildAbsoluteDriveTrainFilePath(projectRoot)))
    {
        return 10;
    }

    const auto selectionLoadResult = selectionService.LoadDraft(projectRoot);
    if (!selectionLoadResult.IsSuccess() ||
        selectionLoadResult.state.drive_train_result.joint_selections.empty())
    {
        return 11;
    }

    // Scheme 在没有 Planning 数据的情况下仍应能聚合前五个模块，并给出完整快照与导出。
    RoboSDP::Scheme::Service::SchemeSnapshotService snapshotService(
        schemeStorage,
        requirementStorage,
        topologyStorage,
        kinematicStorage,
        dynamicStorage,
        selectionStorage,
        planningStorage,
        nullptr);
    const auto snapshotBuildResult = snapshotService.BuildSnapshot(projectRoot);
    if (!snapshotBuildResult.IsSuccess() ||
        snapshotBuildResult.snapshot.aggregate.available_module_count != 5 ||
        !snapshotBuildResult.snapshot.aggregate.selection_summary.success ||
        snapshotBuildResult.snapshot.aggregate.planning_summary.available)
    {
        return 12;
    }

    const auto snapshotSaveResult = snapshotService.SaveSnapshot(projectRoot, snapshotBuildResult.snapshot);
    if (!snapshotSaveResult.IsSuccess() ||
        !QFileInfo::exists(schemeStorage.BuildAbsoluteSnapshotFilePath(projectRoot)))
    {
        return 13;
    }

    const auto snapshotLoadResult = snapshotService.LoadSnapshot(projectRoot);
    if (!snapshotLoadResult.IsSuccess() ||
        snapshotLoadResult.snapshot.aggregate.refs.topology_ref != topologyState.current_model.meta.topology_id)
    {
        return 14;
    }

    RoboSDP::Scheme::Service::SchemeExportService exportService(schemeStorage, nullptr);
    const auto exportResult = exportService.ExportAsJson(projectRoot, snapshotLoadResult.snapshot);
    if (!exportResult.IsSuccess() || !QFileInfo::exists(exportResult.export_file_path))
    {
        return 15;
    }

    return 0;
}
