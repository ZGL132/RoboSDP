#include "core/repository/LocalJsonRepository.h"
#include "modules/dynamics/adapter/PinocchioDynamicsBackendAdapter.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/dynamics/service/DynamicsService.h"
#include "modules/kinematics/dto/KinematicModelDto.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"

#include <QCoreApplication>
#include <QDir>

#include <algorithm>
#include <cmath>

namespace
{

double ComputeMaxTorqueDelta(
    const RoboSDP::Dynamics::Dto::InverseDynamicsTrajectoryResultDto& lhs,
    const RoboSDP::Dynamics::Dto::InverseDynamicsTrajectoryResultDto& rhs)
{
    double maxDelta = 0.0;
    const std::size_t curveCount = std::min(lhs.joint_curves.size(), rhs.joint_curves.size());
    for (std::size_t curveIndex = 0; curveIndex < curveCount; ++curveIndex)
    {
        const auto& lhsCurve = lhs.joint_curves[curveIndex];
        const auto& rhsCurve = rhs.joint_curves[curveIndex];
        const std::size_t sampleCount = std::min(lhsCurve.samples.size(), rhsCurve.samples.size());
        for (std::size_t sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex)
        {
            maxDelta = std::max(
                maxDelta,
                std::abs(lhsCurve.samples[sampleIndex].torque_nm - rhsCurve.samples[sampleIndex].torque_nm));
        }
    }
    return maxDelta;
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

    LocalJsonRepository repository;
    KinematicJsonStorage kinematicStorage(repository);
    DynamicJsonStorage dynamicStorage(repository);

    {
        const QString projectRoot =
            QDir::current().absoluteFilePath(QStringLiteral("dynamics-backend-switch-project"));

        KinematicsWorkspaceStateDto kinematicState = KinematicsWorkspaceStateDto::CreateDefault();
        kinematicState.current_model.meta.kinematic_id = QStringLiteral("kinematic_backend_switch");
        kinematicState.current_model.meta.topology_ref = QStringLiteral("topology_backend_switch");
        kinematicState.current_model.meta.requirement_ref = QStringLiteral("requirement_backend_switch");
        if (kinematicStorage.SaveModel(projectRoot, kinematicState) != RoboSDP::Errors::ErrorCode::Ok)
        {
            return 1;
        }

        DynamicsService dynamicsService(dynamicStorage, kinematicStorage, nullptr);
        const auto buildResult = dynamicsService.BuildFromKinematics(projectRoot);
        if (!buildResult.IsSuccess())
        {
            return 2;
        }

        const auto analyzeResult = dynamicsService.RunInverseDynamicsChain(projectRoot, buildResult.state);
        if (!analyzeResult.IsSuccess() ||
            analyzeResult.state.trajectory_results.empty() ||
            !analyzeResult.message.contains(QStringLiteral("Pinocchio")) ||
            analyzeResult.state.trajectory_results.front().solver_backend != QStringLiteral("pinocchio_shared_kernel") ||
            analyzeResult.state.trajectory_results.front().used_fallback)
        {
            return 3;
        }
    }

    {
        const QString projectRoot =
            QDir::current().absoluteFilePath(QStringLiteral("dynamics-backend-failure-project"));

        KinematicsWorkspaceStateDto kinematicState = KinematicsWorkspaceStateDto::CreateDefault();
        kinematicState.current_model.meta.kinematic_id = QStringLiteral("kinematic_backend_failure");
        kinematicState.current_model.meta.topology_ref = QStringLiteral("topology_backend_failure");
        kinematicState.current_model.meta.requirement_ref = QStringLiteral("requirement_backend_failure");
        kinematicState.current_model.modeling_mode = QStringLiteral("UNSUPPORTED_MODE");
        if (kinematicStorage.SaveModel(projectRoot, kinematicState) != RoboSDP::Errors::ErrorCode::Ok)
        {
            return 4;
        }

        DynamicsService dynamicsService(dynamicStorage, kinematicStorage, nullptr);
        const auto buildResult = dynamicsService.BuildFromKinematics(projectRoot);
        if (!buildResult.IsSuccess())
        {
            return 5;
        }

        const auto analyzeResult = dynamicsService.RunInverseDynamicsChain(projectRoot, buildResult.state);
        if (analyzeResult.IsSuccess() ||
            analyzeResult.state.trajectory_results.empty() ||
            analyzeResult.state.trajectory_results.front().success ||
            analyzeResult.state.trajectory_results.front().message.trimmed().isEmpty() ||
            analyzeResult.state.trajectory_results.front().message.contains(QStringLiteral("Legacy")))
        {
            return 6;
        }
    }

    {
        KinematicModelDto kinematicModel = KinematicModelDto::CreateDefault();
        DynamicModelDto dynamicModel = DynamicModelDto::CreateDefault();
        for (auto& joint : dynamicModel.joints)
        {
            joint.efficiency = 1.0;
            joint.friction.viscous = 0.0;
            joint.friction.coulomb = 0.0;
            joint.friction.statik = 0.0;
        }

        ParameterizedTrajectoryDto zeroVelocityTrajectory;
        zeroVelocityTrajectory.trajectory_id = QStringLiteral("trajectory_zero_velocity");
        zeroVelocityTrajectory.name = QStringLiteral("zero_velocity");
        zeroVelocityTrajectory.duration_s = 0.2;
        zeroVelocityTrajectory.samples.push_back({
            0.1,
            {30.0, -20.0, 45.0, 10.0, 15.0, -5.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

        ParameterizedTrajectoryDto constantVelocityTrajectory = zeroVelocityTrajectory;
        constantVelocityTrajectory.trajectory_id = QStringLiteral("trajectory_constant_velocity");
        constantVelocityTrajectory.name = QStringLiteral("constant_velocity");
        constantVelocityTrajectory.samples.front().velocities_deg_s = {25.0, -18.0, 30.0, 12.0, -9.0, 6.0};

        PinocchioDynamicsBackendAdapter adapter;
        const auto zeroVelocityResult = adapter.Analyze(kinematicModel, dynamicModel, zeroVelocityTrajectory);
        const auto constantVelocityResult = adapter.Analyze(kinematicModel, dynamicModel, constantVelocityTrajectory);

        if (!zeroVelocityResult.success ||
            !constantVelocityResult.success ||
            zeroVelocityResult.used_fallback ||
            constantVelocityResult.used_fallback)
        {
            return 7;
        }

        if (ComputeMaxTorqueDelta(zeroVelocityResult, constantVelocityResult) <= 1.0e-4)
        {
            return 8;
        }
    }

    return 0;
}
