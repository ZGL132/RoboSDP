#include "core/repository/LocalJsonRepository.h"
#include "modules/kinematics/adapter/PinocchioIkSolverAdapter.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/kinematics/service/KinematicsService.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

#include <QCoreApplication>
#include <QDir>

#include <cmath>

namespace
{

bool IsNear(double left, double right, double tolerance)
{
    return std::abs(left - right) <= tolerance;
}

bool PosePositionNear(
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& left,
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& right,
    double tolerance)
{
    return IsNear(left.position_m[0], right.position_m[0], tolerance) &&
           IsNear(left.position_m[1], right.position_m[1], tolerance) &&
           IsNear(left.position_m[2], right.position_m[2], tolerance);
}

bool PoseRpyNear(
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& left,
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& right,
    double tolerance)
{
    return IsNear(left.rpy_deg[0], right.rpy_deg[0], tolerance) &&
           IsNear(left.rpy_deg[1], right.rpy_deg[1], tolerance) &&
           IsNear(left.rpy_deg[2], right.rpy_deg[2], tolerance);
}

RoboSDP::Kinematics::Dto::KinematicModelDto MakeBaseModel()
{
    auto model = RoboSDP::Kinematics::Dto::KinematicModelDto::CreateDefault();
    model.backend_type = QStringLiteral("pinocchio_kinematic_backend");
    model.model_source_mode = QStringLiteral("manual_seed");
    model.pinocchio_model_ready = false;
    return model;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    using namespace RoboSDP::Kinematics::Adapter;
    using namespace RoboSDP::Kinematics::Dto;
    using namespace RoboSDP::Kinematics::Persistence;
    using namespace RoboSDP::Kinematics::Service;
    using namespace RoboSDP::Repository;
    using namespace RoboSDP::Topology::Persistence;

    PinocchioIkSolverAdapter directAdapter;
    if (directAdapter.SolverId() != QStringLiteral("pinocchio_numeric_ik") ||
        !directAdapter.SolverDescription().contains(QStringLiteral("Pinocchio")))
    {
        return 1;
    }

    LocalJsonRepository repository;
    KinematicJsonStorage kinematicStorage(repository);
    TopologyJsonStorage topologyStorage(repository);
    KinematicsService service(kinematicStorage, topologyStorage, nullptr);

    const auto model = MakeBaseModel();
    const std::vector<double> targetJoints {5.0, -8.0, 12.0, 4.0, -6.0, 10.0};

    FkRequestDto fkRequest;
    fkRequest.joint_positions_deg = targetJoints;
    const auto targetFk = service.SolveFk(model, fkRequest);
    if (!targetFk.success)
    {
        return 2;
    }

    IkRequestDto ikRequest;
    ikRequest.target_pose = targetFk.tcp_pose;
    ikRequest.seed_joint_positions_deg = std::vector<double>(6, 0.0);
    const auto pinocchioIkResult = service.SolveIk(model, ikRequest);
    if (!pinocchioIkResult.success ||
        !pinocchioIkResult.message.contains(QStringLiteral("Pinocchio")) ||
        pinocchioIkResult.joint_positions_deg.size() != 6)
    {
        return 3;
    }
    if (pinocchioIkResult.position_error_mm > model.ik_solver_config.position_tolerance_mm + 1.0e-3 ||
        pinocchioIkResult.orientation_error_deg > model.ik_solver_config.orientation_tolerance_deg + 1.0e-3)
    {
        return 4;
    }

    FkRequestDto solvedFkRequest;
    solvedFkRequest.joint_positions_deg = pinocchioIkResult.joint_positions_deg;
    const auto solvedFk = service.SolveFk(model, solvedFkRequest);
    if (!solvedFk.success ||
        !PosePositionNear(solvedFk.tcp_pose, targetFk.tcp_pose, 2.0e-3) ||
        !PoseRpyNear(solvedFk.tcp_pose, targetFk.tcp_pose, 0.5))
    {
        return 5;
    }

    auto failingModel = MakeBaseModel();
    failingModel.modeling_mode = QStringLiteral("URDF");
    failingModel.urdf_source_path = QDir(QStringLiteral(ROBOSDP_TEST_PROJECT_SOURCE_DIR))
        .absoluteFilePath(QStringLiteral("resources/sample-projects/stage1-demo-project/kinematics/not-exist.urdf"));

    IkRequestDto failingRequest;
    failingRequest.target_pose = targetFk.tcp_pose;
    failingRequest.seed_joint_positions_deg = std::vector<double>(6, 0.0);
    const auto failingResult = service.SolveIk(failingModel, failingRequest);
    if (failingResult.success ||
        failingResult.message.trimmed().isEmpty() ||
        failingResult.message.contains(QStringLiteral("Legacy")))
    {
        return 6;
    }

    return 0;
}
