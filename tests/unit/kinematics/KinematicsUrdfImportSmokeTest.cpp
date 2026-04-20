#include "core/logging/ConsoleLogger.h"
#include "core/repository/LocalJsonRepository.h"
#include "modules/kinematics/adapter/PinocchioKinematicBackendAdapter.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/kinematics/service/KinematicsService.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <cmath>

namespace
{

bool IsNear(double left, double right, double tolerance)
{
    return std::abs(left - right) <= tolerance;
}

bool WriteUtf8File(const QString& filePath, const QString& content)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    {
        return false;
    }

    const QByteArray encoded = content.toUtf8();
    return file.write(encoded) == encoded.size();
}

RoboSDP::Kinematics::Dto::KinematicModelDto MakePreviewModel(const QString& absoluteUrdfPath)
{
    auto model = RoboSDP::Kinematics::Dto::KinematicModelDto::CreateDefault();
    const QFileInfo urdfFileInfo(absoluteUrdfPath);
    model.meta.kinematic_id = QStringLiteral("urdf_preview_%1").arg(urdfFileInfo.completeBaseName());
    model.meta.name = urdfFileInfo.completeBaseName();
    model.meta.source = QStringLiteral("urdf_preview");
    model.modeling_mode = QStringLiteral("URDF");
    model.parameter_convention = QStringLiteral("URDF");
    model.model_source_mode = QStringLiteral("urdf_imported");
    model.backend_type = QStringLiteral("pinocchio_kinematic_backend");
    model.unified_robot_model_ref = QStringLiteral("urdf_preview::%1").arg(absoluteUrdfPath);
    model.urdf_source_path = absoluteUrdfPath;
    model.flange_frame = {};
    model.tcp_frame.translation_m = {0.0, 0.0, 0.0};
    model.tcp_frame.rpy_deg = {0.0, 0.0, 0.0};
    return model;
}

const RoboSDP::Kinematics::Dto::UrdfPreviewNodeDto* FindNode(
    const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& scene,
    const QString& linkName)
{
    for (const auto& node : scene.nodes)
    {
        if (node.link_name == linkName)
        {
            return &node;
        }
    }
    return nullptr;
}

const RoboSDP::Kinematics::Dto::UrdfPreviewSegmentDto* FindSegment(
    const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& scene,
    const QString& jointName)
{
    for (const auto& segment : scene.segments)
    {
        if (segment.joint_name == jointName)
        {
            return &segment;
        }
    }
    return nullptr;
}

} // namespace

int main()
{
    using namespace RoboSDP::Kinematics::Adapter;
    using namespace RoboSDP::Kinematics::Persistence;
    using namespace RoboSDP::Kinematics::Service;
    using namespace RoboSDP::Repository;
    using namespace RoboSDP::Topology::Persistence;

    RoboSDP::Logging::ConsoleLogger logger;
    LocalJsonRepository repository;
    TopologyJsonStorage topologyStorage(repository);
    KinematicJsonStorage kinematicStorage(repository);
    KinematicsService service(kinematicStorage, topologyStorage, &logger);
    PinocchioKinematicBackendAdapter adapter(&logger);

    const QString sampleUrdfPath =
        QStringLiteral(ROBOSDP_TEST_PROJECT_SOURCE_DIR
                       "/resources/sample-projects/stage1-demo-project/kinematics/sample-6r-preview.urdf");

    const auto importResult = service.ImportUrdfPreview(sampleUrdfPath);
    if (!importResult.IsSuccess())
    {
        return 1;
    }

    const auto& scene = importResult.preview_scene;
    if (scene.model_name != QStringLiteral("sample_6r_preview"))
    {
        return 2;
    }

    if (scene.nodes.size() != 4 || scene.segments.size() != 3)
    {
        return 3;
    }

    const auto previewModel = MakePreviewModel(QFileInfo(sampleUrdfPath).absoluteFilePath());
    const auto zeroFk = adapter.EvaluateNativeFkDryRun(previewModel, std::vector<double>(2, 0.0));
    if (!zeroFk.success)
    {
        return 4;
    }

    const auto* baseNode = FindNode(scene, QStringLiteral("base_link"));
    const auto* toolNode = FindNode(scene, QStringLiteral("tool_link"));
    const auto* fixedSegment = FindSegment(scene, QStringLiteral("joint_3"));
    if (baseNode == nullptr || toolNode == nullptr || fixedSegment == nullptr)
    {
        return 5;
    }

    if (!IsNear(baseNode->position_m[0], zeroFk.base_pose.position_m[0], 1.0e-6) ||
        !IsNear(baseNode->position_m[1], zeroFk.base_pose.position_m[1], 1.0e-6) ||
        !IsNear(baseNode->position_m[2], zeroFk.base_pose.position_m[2], 1.0e-6))
    {
        return 6;
    }

    if (!IsNear(toolNode->position_m[0], zeroFk.tcp_pose.position_m[0], 1.0e-6) ||
        !IsNear(toolNode->position_m[1], zeroFk.tcp_pose.position_m[1], 1.0e-6) ||
        !IsNear(toolNode->position_m[2], zeroFk.tcp_pose.position_m[2], 1.0e-6))
    {
        return 7;
    }

    if (!IsNear(fixedSegment->end_position_m[0], toolNode->position_m[0], 1.0e-6) ||
        !IsNear(fixedSegment->end_position_m[1], toolNode->position_m[1], 1.0e-6) ||
        !IsNear(fixedSegment->end_position_m[2], toolNode->position_m[2], 1.0e-6))
    {
        return 8;
    }

    const QString projectRoot = QDir::current().absoluteFilePath(QStringLiteral("kinematics-urdf-preview-stage17-project"));
    QDir(projectRoot).removeRecursively();
    if (repository.OpenProject(projectRoot) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 9;
    }

    const QString fixedUrdfPath = QDir(projectRoot).absoluteFilePath(QStringLiteral("fixed-preview.urdf"));
    const QString fixedUrdf = QStringLiteral(
        "<robot name=\"fixed_preview_robot\">"
        "<link name=\"base_link\"/>"
        "<joint name=\"joint_1\" type=\"revolute\">"
        "<parent link=\"base_link\"/><child link=\"link_1\"/>"
        "<axis xyz=\"0 0 1\"/><origin xyz=\"0 0 0.4\" rpy=\"0 0 0\"/>"
        "<limit lower=\"-3.14\" upper=\"3.14\" effort=\"1\" velocity=\"1\"/>"
        "</joint>"
        "<link name=\"link_1\"/>"
        "<joint name=\"fixed_tool\" type=\"fixed\">"
        "<parent link=\"link_1\"/><child link=\"tool_link\"/>"
        "<origin xyz=\"0 0 0.35\" rpy=\"0 0 0\"/>"
        "</joint>"
        "<link name=\"tool_link\"/>"
        "</robot>");
    if (!WriteUtf8File(fixedUrdfPath, fixedUrdf))
    {
        return 10;
    }

    const auto fixedPreviewResult = service.ImportUrdfPreview(fixedUrdfPath);
    if (!fixedPreviewResult.IsSuccess())
    {
        return 11;
    }

    auto fixedPreviewDryRunModel = MakePreviewModel(fixedUrdfPath);
    fixedPreviewDryRunModel.joint_count = 1;
    fixedPreviewDryRunModel.links = {fixedPreviewDryRunModel.links.front()};
    fixedPreviewDryRunModel.joint_limits = {fixedPreviewDryRunModel.joint_limits.front()};
    fixedPreviewDryRunModel.joint_order_signature = QStringLiteral("joint_1");
    const auto fixedFk = adapter.EvaluateNativeFkDryRun(fixedPreviewDryRunModel, std::vector<double>(1, 0.0));
    if (!fixedFk.success)
    {
        return 12;
    }

    const auto* fixedToolNode = FindNode(fixedPreviewResult.preview_scene, QStringLiteral("tool_link"));
    const auto* fixedToolSegment = FindSegment(fixedPreviewResult.preview_scene, QStringLiteral("fixed_tool"));
    if (fixedToolNode == nullptr || fixedToolSegment == nullptr)
    {
        return 13;
    }

    if (!IsNear(fixedToolSegment->start_position_m[2], 0.4, 1.0e-6) ||
        !IsNear(fixedToolSegment->end_position_m[2], 0.75, 1.0e-6))
    {
        return 14;
    }

    if (!IsNear(fixedToolNode->position_m[2], fixedFk.tcp_pose.position_m[2], 1.0e-6) ||
        !IsNear(fixedToolNode->position_m[2], 0.75, 1.0e-6))
    {
        return 15;
    }

    return 0;
}
