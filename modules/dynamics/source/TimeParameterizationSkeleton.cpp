#include "modules/dynamics/trajectory/TimeParameterizationSkeleton.h"

#include <algorithm>

namespace RoboSDP::Dynamics::Trajectory
{

namespace
{

double NormalizedBlend(double u)
{
    return 3.0 * u * u - 2.0 * u * u * u;
}

double NormalizedVelocity(double u)
{
    return 6.0 * u - 6.0 * u * u;
}

double NormalizedAcceleration(double u)
{
    return 6.0 - 12.0 * u;
}

} // namespace

RoboSDP::Dynamics::Dto::ParameterizedTrajectoryDto TimeParameterizationSkeleton::Parameterize(
    const RoboSDP::Dynamics::Dto::BenchmarkTrajectoryDto& trajectory) const
{
    RoboSDP::Dynamics::Dto::ParameterizedTrajectoryDto result;
    result.trajectory_id = trajectory.trajectory_id;
    result.name = trajectory.name;
    result.duration_s = std::max(0.1, trajectory.duration_s);

    const int sampleCount = std::max(2, trajectory.sample_count);
    const std::size_t jointCount =
        std::max(trajectory.joint_start_deg.size(), trajectory.joint_target_deg.size());

    for (int index = 0; index < sampleCount; ++index)
    {
        const double u = sampleCount == 1 ? 1.0 : static_cast<double>(index) / static_cast<double>(sampleCount - 1);
        const double time = u * result.duration_s;

        RoboSDP::Dynamics::Dto::JointStateSampleDto sample;
        sample.time_s = time;
        sample.positions_deg.resize(jointCount, 0.0);
        sample.velocities_deg_s.resize(jointCount, 0.0);
        sample.accelerations_deg_s2.resize(jointCount, 0.0);

        for (std::size_t jointIndex = 0; jointIndex < jointCount; ++jointIndex)
        {
            const double start =
                jointIndex < trajectory.joint_start_deg.size() ? trajectory.joint_start_deg[jointIndex] : 0.0;
            const double target =
                jointIndex < trajectory.joint_target_deg.size() ? trajectory.joint_target_deg[jointIndex] : start;
            const double delta = target - start;

            sample.positions_deg[jointIndex] = start + delta * NormalizedBlend(u);
            sample.velocities_deg_s[jointIndex] = delta * NormalizedVelocity(u) / result.duration_s;
            sample.accelerations_deg_s2[jointIndex] =
                delta * NormalizedAcceleration(u) / (result.duration_s * result.duration_s);
        }

        result.samples.push_back(sample);
    }

    return result;
}

} // namespace RoboSDP::Dynamics::Trajectory
