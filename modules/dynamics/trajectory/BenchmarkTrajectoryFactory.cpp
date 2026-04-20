#include "modules/dynamics/trajectory/BenchmarkTrajectoryFactory.h"

namespace RoboSDP::Dynamics::Trajectory
{

std::vector<RoboSDP::Dynamics::Dto::BenchmarkTrajectoryDto> BenchmarkTrajectoryFactory::CreateMinimumSet(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& kinematicModel) const
{
    const int jointCount = kinematicModel.joint_count > 0 ? kinematicModel.joint_count : 6;
    std::vector<double> zero(jointCount, 0.0);
    std::vector<double> pointToPointTarget {20.0, -30.0, 25.0, 10.0, -15.0, 35.0};
    pointToPointTarget.resize(static_cast<std::size_t>(jointCount), 0.0);

    std::vector<double> singleJointTarget(jointCount, 0.0);
    if (jointCount >= 2)
    {
        singleJointTarget[1] = 45.0;
    }
    else if (jointCount >= 1)
    {
        singleJointTarget[0] = 30.0;
    }

    std::vector<RoboSDP::Dynamics::Dto::BenchmarkTrajectoryDto> trajectories;
    trajectories.push_back({
        QStringLiteral("traj_single_joint"),
        QStringLiteral("单关节梯形速度轨迹"),
        QStringLiteral("single_joint_trapezoid"),
        jointCount >= 2 ? QStringLiteral("joint_2") : QStringLiteral("joint_1"),
        zero,
        singleJointTarget,
        1.5,
        60});

    trajectories.push_back({
        QStringLiteral("traj_point_to_point"),
        QStringLiteral("多关节点到点同步轨迹"),
        QStringLiteral("point_to_point_sync"),
        QString(),
        zero,
        pointToPointTarget,
        2.0,
        80});

    return trajectories;
}

} // namespace RoboSDP::Dynamics::Trajectory
