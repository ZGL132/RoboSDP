#include "modules/kinematics/adapter/PinocchioIkSolverAdapter.h"
#include "modules/kinematics/adapter/PinocchioKinematicBackendAdapter.h"
#include "modules/kinematics/dto/KinematicModelDto.h"
#include "modules/kinematics/dto/KinematicSolverResultDto.h"
#include "modules/kinematics/service/KinematicsService.h"
#include "modules/topology/dto/RobotTopologyModelDto.h"

#include <cmath>

namespace
{
bool NearlyEqual(double lhs, double rhs, double tolerance = 1e-9)
{
    return std::abs(lhs - rhs) <= tolerance;
}
}

int main()
{
    using namespace RoboSDP::Kinematics::Adapter;
    using namespace RoboSDP::Kinematics::Dto;

    const KinematicModelDto model = KinematicModelDto::CreateDefault();
    // PUMA-Spong 约定：a3 携带前臂长度，d4=0 保证 J4/J5/J6 在腕心重合，d5=0 保证球腕。
    const std::vector<KinematicLinkParameterDto> expectedTopologyDefaultLinks {
        {QStringLiteral("link_1"), 0.10,  90.0, 0.35, 0.0},
        {QStringLiteral("link_2"), 0.40,   0.0, 0.0,  90.0},
        {QStringLiteral("link_3"), 0.35,  90.0, 0.0,  0.0},
        {QStringLiteral("link_4"), 0.0,  -90.0, 0.0,  0.0},
        {QStringLiteral("link_5"), 0.0,   90.0, 0.0,  0.0},
        {QStringLiteral("link_6"), 0.0,    0.0, 0.10, 0.0}};
    if (model.links.size() != expectedTopologyDefaultLinks.size())
    {
        return 4;
    }
    for (std::size_t linkIndex = 0; linkIndex < model.links.size(); ++linkIndex)
    {
        const auto& defaultLink = model.links[linkIndex];
        const auto& expectedLink = expectedTopologyDefaultLinks[linkIndex];
        if (defaultLink.link_id != expectedLink.link_id ||
            !NearlyEqual(defaultLink.a, expectedLink.a) ||
            !NearlyEqual(defaultLink.alpha, expectedLink.alpha) ||
            !NearlyEqual(defaultLink.d, expectedLink.d) ||
            !NearlyEqual(defaultLink.theta_offset, expectedLink.theta_offset))
        {
            return 4;
        }
    }

    const auto topologyModel = RoboSDP::Topology::Dto::RobotTopologyModelDto::CreateDefault();
    const auto defaultPreview =
        RoboSDP::Kinematics::Service::KinematicsService::GenerateSkeletonPreview(topologyModel);
    const auto zeroInputPreview =
        RoboSDP::Kinematics::Service::KinematicsService::GenerateSkeletonPreview(
            topologyModel,
            std::vector<double> {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if (defaultPreview.nodes.size() != zeroInputPreview.nodes.size() ||
        defaultPreview.nodes.size() < 4)
    {
        return 5;
    }
    for (std::size_t nodeIndex = 0; nodeIndex < defaultPreview.nodes.size(); ++nodeIndex)
    {
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            if (!NearlyEqual(defaultPreview.nodes[nodeIndex].position_m[axis],
                             zeroInputPreview.nodes[nodeIndex].position_m[axis]))
            {
                return 6;
            }
        }
    }
    if (zeroInputPreview.nodes[2].position_m[2] <= zeroInputPreview.nodes[1].position_m[2])
    {
        return 7;
    }

    const std::vector<double> referenceJoints {10.0, -20.0, 30.0, 15.0, -10.0, 25.0};

    PinocchioKinematicBackendAdapter backend;
    FkRequestDto fkRequest;
    fkRequest.joint_positions_deg = referenceJoints;
    const FkResultDto fkResult = backend.SolveFk(model, fkRequest);
    if (!fkResult.success)
    {
        return 1;
    }

    PinocchioIkSolverAdapter ikSolver;
    IkRequestDto ikRequest;
    ikRequest.target_pose = fkResult.tcp_pose;
    ikRequest.seed_joint_positions_deg = referenceJoints;
    const IkResultDto ikResult = ikSolver.SolveIk(model, ikRequest);
    if (!ikResult.success || ikResult.position_error_mm > 5.0 || ikResult.orientation_error_deg > 5.0)
    {
        return 2;
    }

    WorkspaceRequestDto workspaceRequest;
    workspaceRequest.sample_count = 24;
    const WorkspaceResultDto workspaceResult = backend.SampleWorkspace(model, workspaceRequest);
    if (!workspaceResult.success || workspaceResult.reachable_sample_count != 24 || workspaceResult.max_radius_m <= 0.0)
    {
        return 3;
    }

    return 0;
}
