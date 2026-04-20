#include "modules/dynamics/adapter/PinocchioDynamicsBackendAdapter.h"
#include "modules/dynamics/dto/DynamicModelDto.h"
#include "modules/kinematics/adapter/PinocchioKinematicBackendAdapter.h"
#include "modules/kinematics/dto/KinematicModelDto.h"

#include <QCoreApplication>

#include <vector>

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    using namespace RoboSDP::Dynamics::Adapter;
    using namespace RoboSDP::Dynamics::Dto;
    using namespace RoboSDP::Kinematics::Adapter;
    using namespace RoboSDP::Kinematics::Dto;

    const KinematicModelDto kinematicModel = KinematicModelDto::CreateDefault();
    const DynamicModelDto dynamicModel = DynamicModelDto::CreateDefault();
    const std::vector<double> zeroState(6, 0.0);

    PinocchioDynamicsBackendAdapter dynamicsAdapter;
    const auto dynamicsResult = dynamicsAdapter.EvaluateNativeRneaDryRun(
        kinematicModel,
        dynamicModel,
        zeroState,
        zeroState,
        zeroState);
    if (!dynamicsResult.success)
    {
        return 1;
    }

    PinocchioKinematicBackendAdapter kinematicsAdapter;
    const auto fkResult = kinematicsAdapter.EvaluateNativeFkDryRun(kinematicModel, zeroState);
    if (!fkResult.success)
    {
        return 2;
    }

    const std::uintptr_t dynamicsModelAddress = dynamicsAdapter.NativeModelAddressForDiagnostics();
    const std::uintptr_t kinematicsModelAddress = kinematicsAdapter.NativeModelAddressForDiagnostics();
    const std::uintptr_t dynamicsDataAddress = dynamicsAdapter.NativeDataAddressForDiagnostics();
    const std::uintptr_t kinematicsDataAddress = kinematicsAdapter.NativeDataAddressForDiagnostics();

    if (dynamicsModelAddress == 0U ||
        kinematicsModelAddress == 0U ||
        dynamicsModelAddress != kinematicsModelAddress)
    {
        return 3;
    }

    if (dynamicsDataAddress == 0U ||
        kinematicsDataAddress == 0U ||
        dynamicsDataAddress == kinematicsDataAddress)
    {
        return 4;
    }

    return 0;
}
