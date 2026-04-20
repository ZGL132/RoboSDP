#include "core/kinematics/SharedRobotKernelRegistry.h"
#include "core/logging/ConsoleLogger.h"
#include "core/repository/LocalJsonRepository.h"
#include "modules/kinematics/dto/KinematicModelDto.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/kinematics/service/KinematicsService.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <array>
#include <cmath>

namespace
{

bool WriteUtf8File(const QString& filePath, const QString& content)
{
    const QFileInfo fileInfo(filePath);
    QDir().mkpath(fileInfo.absolutePath());

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    {
        return false;
    }

    const QByteArray encoded = content.toUtf8();
    return file.write(encoded) == encoded.size();
}

bool IsNear(double left, double right, double tolerance = 1.0e-9)
{
    return std::abs(left - right) <= tolerance;
}

RoboSDP::Kinematics::Dto::KinematicModelDto MakePreviewModel(const QString& absoluteUrdfPath)
{
    RoboSDP::Kinematics::Dto::KinematicModelDto model;
    const QFileInfo urdfFileInfo(absoluteUrdfPath);
    model.meta.kinematic_id = QStringLiteral("urdf_geometry_%1").arg(urdfFileInfo.completeBaseName());
    model.meta.name = urdfFileInfo.completeBaseName();
    model.meta.source = QStringLiteral("urdf_geometry_smoke");
    model.modeling_mode = QStringLiteral("URDF");
    model.parameter_convention = QStringLiteral("URDF");
    model.model_source_mode = QStringLiteral("urdf_imported");
    model.backend_type = QStringLiteral("pinocchio_kinematic_backend");
    model.unified_robot_model_ref = QStringLiteral("urdf_geometry::%1").arg(absoluteUrdfPath);
    model.joint_order_signature = absoluteUrdfPath;
    model.urdf_source_path = absoluteUrdfPath;
    model.mesh_search_directories = {
        urdfFileInfo.absolutePath(),
        QDir(urdfFileInfo.absolutePath()).absoluteFilePath(QStringLiteral(".."))};
    model.joint_count = 0;
    model.links.clear();
    model.joint_limits.clear();
    model.flange_frame = {};
    model.tcp_frame.translation_m = {0.0, 0.0, 0.0};
    model.tcp_frame.rpy_deg = {0.0, 0.0, 0.0};
    return model;
}

const RoboSDP::Kinematics::Dto::GeometryObjectDto* FindGeometryByLink(
    const std::vector<RoboSDP::Kinematics::Dto::GeometryObjectDto>& geometries,
    const QString& linkName)
{
    for (const auto& geometry : geometries)
    {
        if (geometry.link_name == linkName)
        {
            return &geometry;
        }
    }

    return nullptr;
}

} // namespace

int main()
{
    using namespace RoboSDP::Core::Kinematics;
    using namespace RoboSDP::Kinematics::Persistence;
    using namespace RoboSDP::Kinematics::Service;
    using namespace RoboSDP::Repository;
    using namespace RoboSDP::Topology::Persistence;

    RoboSDP::Logging::ConsoleLogger logger;
    LocalJsonRepository repository;
    TopologyJsonStorage topologyStorage(repository);
    KinematicJsonStorage kinematicStorage(repository);
    KinematicsService service(kinematicStorage, topologyStorage, &logger);

    const QString tempRoot =
        QDir::current().absoluteFilePath(QStringLiteral("kinematics-urdf-geometry-stage20-project"));
    QDir(tempRoot).removeRecursively();
    QDir().mkpath(tempRoot);

    const QString relativeUrdfPath = QDir(tempRoot).absoluteFilePath(QStringLiteral("relative-mesh/robot.urdf"));
    const QString relativeMeshPath = QDir(tempRoot).absoluteFilePath(QStringLiteral("relative-mesh/meshes/tool.stl"));
    if (!WriteUtf8File(relativeMeshPath, QStringLiteral("solid tool\nendsolid tool\n")))
    {
        return 1;
    }

    const QString relativeUrdf = QStringLiteral(
        "<robot name=\"relative_mesh_robot\">"
        "<link name=\"base_link\"/>"
        "<joint name=\"joint_1\" type=\"revolute\">"
        "<parent link=\"base_link\"/><child link=\"tool_link\"/>"
        "<origin xyz=\"0 0 0.3\" rpy=\"0 0 0\"/>"
        "<limit lower=\"-3.14\" upper=\"3.14\" effort=\"1\" velocity=\"1\"/>"
        "</joint>"
        "<link name=\"tool_link\">"
        "<visual>"
        "<origin xyz=\"0.01 0.02 0.03\" rpy=\"0 0 0\"/>"
        "<geometry><mesh filename=\"meshes/tool.stl\" scale=\"1 2 3\"/></geometry>"
        "</visual>"
        "</link>"
        "</robot>");
    if (!WriteUtf8File(relativeUrdfPath, relativeUrdf))
    {
        return 2;
    }

    const auto relativeImportResult = service.ImportUrdfPreview(relativeUrdfPath);
    if (!relativeImportResult.IsSuccess())
    {
        return 3;
    }
    if (relativeImportResult.preview_scene.visual_geometries.size() != 1)
    {
        return 4;
    }

    const auto* relativeGeometry = FindGeometryByLink(
        relativeImportResult.preview_scene.visual_geometries,
        QStringLiteral("tool_link"));
    if (relativeGeometry == nullptr)
    {
        return 5;
    }
    if (relativeGeometry->geometry_type != QStringLiteral("mesh"))
    {
        return 6;
    }
    if (QDir::fromNativeSeparators(relativeGeometry->absolute_file_path) !=
        QDir::fromNativeSeparators(QFileInfo(relativeMeshPath).absoluteFilePath()))
    {
        return 7;
    }
    if (!relativeGeometry->resource_available)
    {
        return 8;
    }
    if (relativeImportResult.message.contains(QStringLiteral("尚未支持外部 Mesh")) ||
        relativeImportResult.message.contains(QStringLiteral("仅支持 STL visual mesh")))
    {
        return 25;
    }
    if (!IsNear(relativeGeometry->scale[0], 1.0) ||
        !IsNear(relativeGeometry->scale[1], 2.0) ||
        !IsNear(relativeGeometry->scale[2], 3.0))
    {
        return 9;
    }

    const auto poseUpdateResult =
        service.UpdatePreviewPoses(MakePreviewModel(relativeUrdfPath), std::vector<double>{30.0});
    if (!poseUpdateResult.IsSuccess())
    {
        return 22;
    }
    if (poseUpdateResult.link_world_poses.find(QStringLiteral("tool_link")) ==
        poseUpdateResult.link_world_poses.end())
    {
        return 23;
    }

    const auto badPoseUpdateResult =
        service.UpdatePreviewPoses(MakePreviewModel(relativeUrdfPath), std::vector<double>{30.0, 15.0});
    if (badPoseUpdateResult.IsSuccess())
    {
        return 24;
    }

    const QString packageRoot = QDir(tempRoot).absoluteFilePath(QStringLiteral("package-search"));
    const QString packageUrdfPath = QDir(packageRoot).absoluteFilePath(QStringLiteral("robot.urdf"));
    const QString packageMeshPath =
        QDir(packageRoot).absoluteFilePath(QStringLiteral("ws/src/demo_robot/meshes/link.stl"));
    if (!WriteUtf8File(packageMeshPath, QStringLiteral("solid link\nendsolid link\n")))
    {
        return 10;
    }

    const QString packageUrdf = QStringLiteral(
        "<robot name=\"package_mesh_robot\">"
        "<link name=\"base_link\"/>"
        "<joint name=\"joint_1\" type=\"revolute\">"
        "<parent link=\"base_link\"/><child link=\"tool_link\"/>"
        "<origin xyz=\"0 0 0.2\" rpy=\"0 0 0\"/>"
        "<limit lower=\"-3.14\" upper=\"3.14\" effort=\"1\" velocity=\"1\"/>"
        "</joint>"
        "<link name=\"tool_link\">"
        "<collision>"
        "<geometry><mesh filename=\"package://demo_robot/meshes/link.stl\"/></geometry>"
        "</collision>"
        "</link>"
        "</robot>");
    if (!WriteUtf8File(packageUrdfPath, packageUrdf))
    {
        return 11;
    }

    auto packageModel = MakePreviewModel(packageUrdfPath);
    packageModel.mesh_search_directories = {
        QDir(packageRoot).absoluteFilePath(QStringLiteral("not-found-root")),
        QDir(packageRoot).absoluteFilePath(QStringLiteral("ws/src"))};

    SharedRobotKernelRequest packageRequest;
    packageRequest.kinematic_model = &packageModel;
    packageRequest.unified_robot_model_ref = packageModel.unified_robot_model_ref;
    packageRequest.modeling_mode = packageModel.modeling_mode;
    packageRequest.joint_order_signature = packageModel.joint_order_signature;
    packageRequest.allow_structural_alias = false;

    const auto packageAcquireResult =
        SharedRobotKernelRegistry::Instance().GetOrBuildKernel(packageRequest);
    if (!packageAcquireResult.success)
    {
        return 12;
    }
    if (packageAcquireResult.metadata.collision_geometries.size() != 1)
    {
        return 13;
    }
    if (QDir::fromNativeSeparators(packageAcquireResult.metadata.collision_geometries.front().absolute_file_path) !=
        QDir::fromNativeSeparators(QFileInfo(packageMeshPath).absoluteFilePath()))
    {
        return 14;
    }
    if (!packageAcquireResult.metadata.collision_geometries.front().resource_available)
    {
        return 15;
    }

    const QString missingUrdfPath = QDir(tempRoot).absoluteFilePath(QStringLiteral("missing-mesh/robot.urdf"));
    const QString missingUrdf = QStringLiteral(
        "<robot name=\"missing_mesh_robot\">"
        "<link name=\"base_link\"/>"
        "<joint name=\"joint_1\" type=\"revolute\">"
        "<parent link=\"base_link\"/><child link=\"tool_link\"/>"
        "<origin xyz=\"0 0 0.25\" rpy=\"0 0 0\"/>"
        "<limit lower=\"-3.14\" upper=\"3.14\" effort=\"1\" velocity=\"1\"/>"
        "</joint>"
        "<link name=\"tool_link\">"
        "<visual>"
        "<geometry><mesh filename=\"meshes/missing_tool.stl\"/></geometry>"
        "</visual>"
        "</link>"
        "</robot>");
    if (!WriteUtf8File(missingUrdfPath, missingUrdf))
    {
        return 16;
    }

    const auto missingImportResult = service.ImportUrdfPreview(missingUrdfPath);
    if (!missingImportResult.IsSuccess())
    {
        return 17;
    }
    if (missingImportResult.preview_scene.visual_geometries.size() != 1)
    {
        return 18;
    }
    if (missingImportResult.preview_scene.nodes.empty() || missingImportResult.preview_scene.segments.empty())
    {
        return 19;
    }
    const auto& missingGeometry = missingImportResult.preview_scene.visual_geometries.front();
    if (missingGeometry.resource_available)
    {
        return 20;
    }
    if (missingGeometry.status_message.trimmed().isEmpty() &&
        !missingImportResult.message.contains(QStringLiteral("不存在")))
    {
        return 21;
    }

    return 0;
}
