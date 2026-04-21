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

const RoboSDP::Kinematics::Dto::UrdfPreviewNodeDto* FindNodeByLink(
    const std::vector<RoboSDP::Kinematics::Dto::UrdfPreviewNodeDto>& nodes,
    const QString& linkName)
{
    for (const auto& node : nodes)
    {
        if (node.link_name == linkName)
        {
            return &node;
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
    if (!IsNear(relativeGeometry->local_pose.position_m[0], 0.01) ||
        !IsNear(relativeGeometry->local_pose.position_m[1], 0.02) ||
        !IsNear(relativeGeometry->local_pose.position_m[2], 0.03))
    {
        return 26;
    }

    const auto* relativeToolNode = FindNodeByLink(
        relativeImportResult.preview_scene.nodes,
        QStringLiteral("tool_link"));
    if (relativeToolNode == nullptr)
    {
        return 27;
    }
    if (!IsNear(relativeToolNode->world_pose.position_m[0], 0.0) ||
        !IsNear(relativeToolNode->world_pose.position_m[1], 0.0) ||
        !IsNear(relativeToolNode->world_pose.position_m[2], 0.3))
    {
        return 28;
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

    const QString axisUrdfPath = QDir(tempRoot).absoluteFilePath(QStringLiteral("axis-x/robot.urdf"));
    const QString axisUrdf = QStringLiteral(
        "<robot name=\"axis_x_robot\">"
        "<link name=\"base_link\"/>"
        "<joint name=\"joint_1\" type=\"revolute\">"
        "<parent link=\"base_link\"/><child link=\"link_1\"/>"
        "<axis xyz=\"1 0 0\"/><origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
        "<limit lower=\"-3.14\" upper=\"3.14\" effort=\"1\" velocity=\"1\"/>"
        "</joint>"
        "<link name=\"link_1\"/>"
        "<joint name=\"fixed_tool\" type=\"fixed\">"
        "<parent link=\"link_1\"/><child link=\"tool_link\"/>"
        "<origin xyz=\"0 1 0\" rpy=\"0 0 0\"/>"
        "</joint>"
        "<link name=\"tool_link\"/>"
        "</robot>");
    if (!WriteUtf8File(axisUrdfPath, axisUrdf))
    {
        return 36;
    }

    const auto axisImportResult = service.ImportUrdfPreview(axisUrdfPath);
    if (!axisImportResult.IsSuccess())
    {
        return 37;
    }
    if (axisImportResult.preview_scene.segments.empty() ||
        !IsNear(axisImportResult.preview_scene.segments.front().joint_axis_xyz[0], 1.0) ||
        !IsNear(axisImportResult.preview_scene.segments.front().joint_axis_xyz[1], 0.0) ||
        !IsNear(axisImportResult.preview_scene.segments.front().joint_axis_xyz[2], 0.0))
    {
        return 38;
    }

    const auto axisPoseUpdateResult =
        service.UpdatePreviewPoses(MakePreviewModel(axisUrdfPath), std::vector<double>{90.0});
    if (!axisPoseUpdateResult.IsSuccess())
    {
        return 39;
    }
    const auto axisToolPoseIt = axisPoseUpdateResult.link_world_poses.find(QStringLiteral("tool_link"));
    if (axisToolPoseIt == axisPoseUpdateResult.link_world_poses.end())
    {
        return 40;
    }
    if (!IsNear(axisToolPoseIt->second.position_m[0], 0.0, 1.0e-6) ||
        !IsNear(axisToolPoseIt->second.position_m[1], 0.0, 1.0e-6) ||
        !IsNear(axisToolPoseIt->second.position_m[2], 1.0, 1.0e-6))
    {
        return 41;
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

    const QString siblingPackageRoot =
        QDir(tempRoot).absoluteFilePath(QStringLiteral("folder-not-named-link12"));
    const QString siblingPackageUrdfPath =
        QDir(siblingPackageRoot).absoluteFilePath(QStringLiteral("urdf/link12.urdf"));
    const QString siblingPackageMeshPath =
        QDir(siblingPackageRoot).absoluteFilePath(QStringLiteral("meshes/base_link.STL"));
    if (!WriteUtf8File(siblingPackageMeshPath, QStringLiteral("solid base_link\nendsolid base_link\n")))
    {
        return 30;
    }

    const QString siblingPackageUrdf = QStringLiteral(
        "<robot name=\"link12\">"
        "<link name=\"base_link\">"
        "<visual><geometry><mesh filename=\"package://link12/meshes/base_link.STL\"/></geometry></visual>"
        "</link>"
        "<joint name=\"joint_1\" type=\"revolute\">"
        "<parent link=\"base_link\"/><child link=\"link1\"/>"
        "<origin xyz=\"0 0 0.2\" rpy=\"0 0 0\"/>"
        "<limit lower=\"-3.14\" upper=\"3.14\" effort=\"1\" velocity=\"1\"/>"
        "</joint>"
        "<link name=\"link1\"/>"
        "</robot>");
    if (!WriteUtf8File(siblingPackageUrdfPath, siblingPackageUrdf))
    {
        return 31;
    }

    const auto siblingPackageImportResult = service.ImportUrdfPreview(siblingPackageUrdfPath);
    if (!siblingPackageImportResult.IsSuccess())
    {
        return 32;
    }
    if (siblingPackageImportResult.preview_scene.visual_geometries.size() != 1)
    {
        return 33;
    }
    const auto& siblingPackageGeometry =
        siblingPackageImportResult.preview_scene.visual_geometries.front();
    if (QDir::fromNativeSeparators(siblingPackageGeometry.absolute_file_path) !=
        QDir::fromNativeSeparators(QFileInfo(siblingPackageMeshPath).absoluteFilePath()))
    {
        return 34;
    }
    if (!siblingPackageGeometry.resource_available)
    {
        return 35;
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
