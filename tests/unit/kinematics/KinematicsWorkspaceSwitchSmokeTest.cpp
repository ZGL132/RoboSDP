#include "core/repository/LocalJsonRepository.h"
#include "modules/kinematics/adapter/PinocchioKinematicBackendAdapter.h"
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

bool Array3Near(const std::array<double, 3>& left, const std::array<double, 3>& right, double tolerance)
{
    return IsNear(left[0], right[0], tolerance) &&
           IsNear(left[1], right[1], tolerance) &&
           IsNear(left[2], right[2], tolerance);
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

    LocalJsonRepository repository;
    KinematicJsonStorage kinematicStorage(repository);
    TopologyJsonStorage topologyStorage(repository);
    KinematicsService service(kinematicStorage, topologyStorage, nullptr);
    PinocchioKinematicBackendAdapter adapter;

    const auto model = MakeBaseModel();

    WorkspaceRequestDto primaryRequest;
    primaryRequest.sample_count = 64;
    const auto primaryWorkspace = service.SampleWorkspace(model, primaryRequest);
    if (!primaryWorkspace.success ||
        primaryWorkspace.requested_sample_count != 64 ||
        primaryWorkspace.reachable_sample_count != 64 ||
        primaryWorkspace.sampled_points.size() != 64 ||
        !primaryWorkspace.message.contains(QStringLiteral("Pinocchio")))
    {
        return 1;
    }

    WorkspaceRequestDto compareRequest;
    compareRequest.sample_count = 1000;
    const auto serviceCompare = service.SampleWorkspace(model, compareRequest);
    const auto directCompare = adapter.SampleWorkspace(model, compareRequest);
    if (!serviceCompare.success ||
        !directCompare.success ||
        serviceCompare.sampled_points.size() != 1000 ||
        directCompare.sampled_points.size() != 1000)
    {
        return 2;
    }

    if (!Array3Near(serviceCompare.min_position_m, directCompare.min_position_m, 1.0e-4) ||
        !Array3Near(serviceCompare.max_position_m, directCompare.max_position_m, 1.0e-4) ||
        !IsNear(serviceCompare.max_radius_m, directCompare.max_radius_m, 1.0e-4))
    {
        return 3;
    }

    auto failingModel = MakeBaseModel();
    failingModel.modeling_mode = QStringLiteral("URDF");
    failingModel.urdf_source_path = QDir(QStringLiteral(ROBOSDP_TEST_PROJECT_SOURCE_DIR))
        .absoluteFilePath(QStringLiteral("resources/sample-projects/stage1-demo-project/kinematics/not-exist.urdf"));

    WorkspaceRequestDto failingRequest;
    failingRequest.sample_count = 24;
    const auto failingWorkspace = service.SampleWorkspace(failingModel, failingRequest);
    if (failingWorkspace.success ||
        failingWorkspace.message.trimmed().isEmpty() ||
        failingWorkspace.message.contains(QStringLiteral("Legacy")))
    {
        return 4;
    }

    return 0;
}
