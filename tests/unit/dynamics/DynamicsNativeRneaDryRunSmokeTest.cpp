#include "core/repository/LocalJsonRepository.h"
#include "modules/dynamics/adapter/PinocchioDynamicsBackendAdapter.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/dynamics/service/DynamicsService.h"
#include "modules/kinematics/dto/KinematicModelDto.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"

#include <QCoreApplication>

#include <algorithm>
#include <cmath>

namespace
{

double MaxAbsValue(const std::vector<double>& values)
{
    double maxAbsValue = 0.0;
    for (double value : values)
    {
        maxAbsValue = std::max(maxAbsValue, std::abs(value));
    }
    return maxAbsValue;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    using namespace RoboSDP::Dynamics::Adapter;
    using namespace RoboSDP::Dynamics::Dto;
    using namespace RoboSDP::Dynamics::Persistence;
    using namespace RoboSDP::Dynamics::Service;
    using namespace RoboSDP::Kinematics::Dto;
    using namespace RoboSDP::Kinematics::Persistence;
    using namespace RoboSDP::Repository;

    const KinematicModelDto kinematicModel = KinematicModelDto::CreateDefault();
    const DynamicModelDto dynamicModel = DynamicModelDto::CreateDefault();

    PinocchioDynamicsBackendAdapter adapter;

    const auto wrongPositionResult = adapter.EvaluateNativeRneaDryRun(
        kinematicModel,
        dynamicModel,
        {0.0, 0.0},
        std::vector<double>(6, 0.0),
        std::vector<double>(6, 0.0));
    if (wrongPositionResult.success ||
        !wrongPositionResult.message.contains(QStringLiteral("输入关节位置数量")))
    {
        return 1;
    }

    const auto wrongVelocityResult = adapter.EvaluateNativeRneaDryRun(
        kinematicModel,
        dynamicModel,
        std::vector<double>(6, 0.0),
        std::vector<double>(2, 0.0),
        std::vector<double>(6, 0.0));
    if (wrongVelocityResult.success ||
        !wrongVelocityResult.message.contains(QStringLiteral("输入关节速度数量")))
    {
        return 2;
    }

    const std::vector<double> zeroPositions(6, 0.0);
    const std::vector<double> zeroVelocities(6, 0.0);
    const std::vector<double> zeroAccelerations(6, 0.0);
    const auto zeroStateResult = adapter.EvaluateNativeRneaDryRun(
        kinematicModel,
        dynamicModel,
        zeroPositions,
        zeroVelocities,
        zeroAccelerations);
    if (!zeroStateResult.success ||
        zeroStateResult.joint_torques_nm.size() != 6 ||
        MaxAbsValue(zeroStateResult.joint_torques_nm) <= 1.0e-6)
    {
        return 3;
    }

    const int firstBuildCount = adapter.GetNativeModelBuildCount();
    if (!adapter.UsesSharedRobotKernel() || firstBuildCount <= 0)
    {
        return 4;
    }

    const auto secondZeroStateResult = adapter.EvaluateNativeRneaDryRun(
        kinematicModel,
        dynamicModel,
        zeroPositions,
        zeroVelocities,
        zeroAccelerations);
    const int secondBuildCount = adapter.GetNativeModelBuildCount();
    if (!secondZeroStateResult.success || secondBuildCount != firstBuildCount)
    {
        return 5;
    }

    LocalJsonRepository repository;
    KinematicJsonStorage kinematicStorage(repository);
    DynamicJsonStorage dynamicStorage(repository);
    DynamicsService service(dynamicStorage, kinematicStorage, nullptr);
    const auto serviceDryRunResult = service.InspectNativeRneaDryRun(
        kinematicModel,
        dynamicModel,
        zeroPositions,
        zeroVelocities,
        zeroAccelerations);
    if (!serviceDryRunResult.success ||
        serviceDryRunResult.joint_torques_nm.size() != 6)
    {
        return 6;
    }

    return 0;
}
