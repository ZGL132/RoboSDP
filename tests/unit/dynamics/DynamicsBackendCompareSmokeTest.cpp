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
        QDir::current().absoluteFilePath(QStringLiteral("dynamics-compare-project"));

    RoboSDP::Repository::LocalJsonRepository repository;
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage kinematicStorage(repository);
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage dynamicStorage(repository);

    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto kinematicState =
        RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto::CreateDefault();
    kinematicState.current_model.meta.kinematic_id = QStringLiteral("kinematic_compare");
    kinematicState.current_model.meta.topology_ref = QStringLiteral("topology_compare");
    kinematicState.current_model.meta.requirement_ref = QStringLiteral("requirement_compare");

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

    const auto compareResult = dynamicsService.CompareInverseDynamicsBackends(projectRoot, buildResult.state);
    if (!compareResult.IsSuccess())
    {
        return 3;
    }

    if (compareResult.rows.empty())
    {
        return 4;
    }

    if (compareResult.native_backend != QStringLiteral("pinocchio_shared_kernel"))
    {
        return 5;
    }

    if (compareResult.native_used_fallback)
    {
        return 6;
    }

    return 0;
}
