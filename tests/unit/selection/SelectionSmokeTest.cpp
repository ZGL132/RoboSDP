#include "core/repository/LocalJsonRepository.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/dynamics/service/DynamicsService.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/selection/persistence/SelectionJsonStorage.h"
#include "modules/selection/service/DriveTrainMatchingService.h"

#include <QCoreApplication>
#include <QDir>

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    const QString projectRoot =
        QDir::current().absoluteFilePath(QStringLiteral("selection-smoke-project"));

    RoboSDP::Repository::LocalJsonRepository repository;
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage kinematicStorage(repository);
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage dynamicStorage(repository);
    RoboSDP::Selection::Persistence::SelectionJsonStorage selectionStorage(repository);

    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto kinematicState =
        RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto::CreateDefault();
    kinematicState.current_model.meta.kinematic_id = QStringLiteral("kinematic_selection_smoke");
    kinematicState.current_model.meta.topology_ref = QStringLiteral("topology_selection_smoke");
    kinematicState.current_model.meta.requirement_ref = QStringLiteral("requirement_selection_smoke");

    if (kinematicStorage.SaveModel(projectRoot, kinematicState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 1;
    }

    RoboSDP::Dynamics::Service::DynamicsService dynamicsService(dynamicStorage, kinematicStorage, nullptr);
    const auto buildResult = dynamicsService.BuildFromKinematics(projectRoot);
    if (!buildResult.IsSuccess())
    {
        return 2;
    }

    const auto analyzeResult = dynamicsService.RunInverseDynamicsChain(projectRoot, buildResult.state);
    if (!analyzeResult.IsSuccess())
    {
        return 3;
    }

    const auto dynamicSaveResult = dynamicsService.SaveDraft(projectRoot, analyzeResult.state);
    if (!dynamicSaveResult.IsSuccess())
    {
        return 4;
    }

    RoboSDP::Selection::Service::DriveTrainMatchingService matchingService(selectionStorage, dynamicStorage, nullptr);
    const auto runResult = matchingService.RunSelection(projectRoot);
    if (!runResult.IsSuccess() ||
        runResult.state.motor_results.empty() ||
        runResult.state.reducer_results.empty() ||
        runResult.state.drive_train_result.joint_selections.empty())
    {
        return 5;
    }

    if (!runResult.state.drive_train_result.success)
    {
        return 6;
    }

    const auto saveResult = matchingService.SaveDraft(projectRoot, runResult.state);
    if (!saveResult.IsSuccess())
    {
        return 7;
    }

    const auto loadResult = matchingService.LoadDraft(projectRoot);
    if (!loadResult.IsSuccess() ||
        loadResult.state.motor_results.empty() ||
        loadResult.state.reducer_results.empty() ||
        loadResult.state.drive_train_result.joint_selections.empty())
    {
        return 8;
    }

    return 0;
}
