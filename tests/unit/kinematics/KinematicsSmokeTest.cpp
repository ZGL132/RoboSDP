#include "modules/kinematics/adapter/PinocchioIkSolverAdapter.h"
#include "modules/kinematics/adapter/PinocchioKinematicBackendAdapter.h"
#include "modules/kinematics/dto/KinematicModelDto.h"
#include "modules/kinematics/dto/KinematicSolverResultDto.h"

int main()
{
    using namespace RoboSDP::Kinematics::Adapter;
    using namespace RoboSDP::Kinematics::Dto;

    const KinematicModelDto model = KinematicModelDto::CreateDefault();
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
