#include "core/repository/LocalJsonRepository.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/dynamics/service/DynamicsService.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"

#include <QCoreApplication>
#include <QDir>

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    const QString projectRoot =
        QDir::current().absoluteFilePath(QStringLiteral("dynamics-smoke-project"));

    RoboSDP::Repository::LocalJsonRepository repository;
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage kinematicStorage(repository);
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage dynamicStorage(repository);

    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto kinematicState =
        RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto::CreateDefault();
    kinematicState.current_model.meta.kinematic_id = QStringLiteral("kinematic_smoke");
    kinematicState.current_model.meta.topology_ref = QStringLiteral("topology_smoke");
    kinematicState.current_model.meta.requirement_ref = QStringLiteral("requirement_smoke");

    if (kinematicStorage.SaveModel(projectRoot, kinematicState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 1;
    }

    RoboSDP::Dynamics::Service::DynamicsService dynamicsService(dynamicStorage, kinematicStorage, nullptr);

    const auto buildResult = dynamicsService.BuildFromKinematics(projectRoot);
    if (!buildResult.IsSuccess() || buildResult.state.current_model.trajectories.empty())
    {
        return 2;
    }

    const auto analyzeResult = dynamicsService.RunInverseDynamicsChain(projectRoot, buildResult.state);
    if (!analyzeResult.IsSuccess())
    {
        return 3;
    }

    if (analyzeResult.state.peak_stats.empty() ||
        analyzeResult.state.rms_stats.empty() ||
        analyzeResult.state.load_envelope.joints.empty())
    {
        return 4;
    }

    if (analyzeResult.state.trajectory_results.empty() ||
        analyzeResult.state.trajectory_results.front().solver_backend.isEmpty())
    {
        return 7;
    }

    const auto saveResult = dynamicsService.SaveDraft(projectRoot, analyzeResult.state);
    if (!saveResult.IsSuccess())
    {
        return 5;
    }

    const auto loadResult = dynamicsService.LoadDraft(projectRoot);
    if (!loadResult.IsSuccess() ||
        loadResult.state.current_model.links.empty() ||
        loadResult.state.load_envelope.joints.empty())
    {
        return 6;
    }

    return 0;
}
