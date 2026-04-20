#include "modules/dynamics/adapter/InverseDynamicsAdapter.h"
#include "modules/dynamics/adapter/PinocchioInverseDynamicsAdapter.h"
#include "modules/dynamics/trajectory/TimeParameterizationSkeleton.h"

#include <QCoreApplication>

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    RoboSDP::Kinematics::Dto::KinematicModelDto kinematicModel =
        RoboSDP::Kinematics::Dto::KinematicModelDto::CreateDefault();
    RoboSDP::Dynamics::Dto::DynamicModelDto dynamicModel =
        RoboSDP::Dynamics::Dto::DynamicModelDto::CreateDefault();

    RoboSDP::Dynamics::Dto::BenchmarkTrajectoryDto benchmarkTrajectory;
    benchmarkTrajectory.trajectory_id = QStringLiteral("trajectory_pinocchio_compare");
    benchmarkTrajectory.name = QStringLiteral("Pinocchio Adapter Smoke");
    benchmarkTrajectory.profile_type = QStringLiteral("joint_ptp");
    benchmarkTrajectory.active_joint_id = QStringLiteral("joint_2");
    benchmarkTrajectory.joint_start_deg = {0.0, -20.0, 30.0, 0.0, 10.0, 0.0};
    benchmarkTrajectory.joint_target_deg = {10.0, -35.0, 45.0, 15.0, 12.0, 20.0};
    benchmarkTrajectory.duration_s = 1.4;
    benchmarkTrajectory.sample_count = 32;

    RoboSDP::Dynamics::Trajectory::TimeParameterizationSkeleton parameterization;
    const auto parameterized = parameterization.Parameterize(benchmarkTrajectory);
    if (parameterized.samples.empty())
    {
        return 1;
    }

    RoboSDP::Dynamics::Adapter::InverseDynamicsAdapter legacyAdapter;
    RoboSDP::Dynamics::Adapter::PinocchioInverseDynamicsAdapter pinocchioAdapter;

    const auto legacyResult = legacyAdapter.Analyze(kinematicModel, dynamicModel, parameterized);
    const auto pinocchioResult = pinocchioAdapter.Analyze(kinematicModel, dynamicModel, parameterized);

    if (!legacyResult.success || !pinocchioResult.success)
    {
        return 2;
    }

    if (legacyResult.joint_curves.size() != pinocchioResult.joint_curves.size())
    {
        return 3;
    }

    if (!RoboSDP::Dynamics::Adapter::PinocchioInverseDynamicsAdapter::IsNativePinocchioAvailable())
    {
        return 4;
    }

    if (pinocchioResult.solver_backend != QStringLiteral("pinocchio_native"))
    {
        return 5;
    }

    if (pinocchioResult.used_fallback)
    {
        return 6;
    }

    if (pinocchioResult.joint_curves.empty() || pinocchioResult.joint_curves.front().samples.empty())
    {
        return 7;
    }

    return 0;
}
