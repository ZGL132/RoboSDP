#include "core/repository/LocalJsonRepository.h"
#include "modules/kinematics/adapter/PinocchioKinematicBackendAdapter.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/kinematics/service/KinematicsService.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>

#include <array>
#include <cmath>
#include <limits>

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

    PinocchioKinematicBackendAdapter adapter;

    const auto dhModel = MakeBaseModel();
    const auto dhResult = adapter.BuildNormalizedContext(dhModel);
    if (!dhResult.IsSuccess())
    {
        return 1;
    }
    if (dhResult.context.normalized_model.modeling_mode != QStringLiteral("DH") ||
        dhResult.context.normalized_model.parameter_convention != QStringLiteral("DH"))
    {
        return 2;
    }
    if (dhResult.context.normalized_model.normalized_backend_type != adapter.BackendId() ||
        dhResult.context.normalized_model.declared_backend_type != QStringLiteral("pinocchio_kinematic_backend"))
    {
        return 3;
    }
    if (!dhResult.context.frame_mapping.semantics_complete ||
        !dhResult.context.joint_order_mapping.mapping_complete)
    {
        return 4;
    }
    if ((dhResult.status.status_code != QStringLiteral("pinocchio_native_model_ready") &&
         dhResult.status.status_code != QStringLiteral("pinocchio_shared_model_ready")) ||
        !dhResult.status.shared_robot_kernel_ready ||
        !adapter.UsesSharedRobotKernel())
    {
        return 5;
    }

    auto mdhModel = MakeBaseModel();
    mdhModel.modeling_mode = QStringLiteral("MDH");
    mdhModel.parameter_convention = QStringLiteral("MDH");
    const auto mdhResult = adapter.BuildNormalizedContext(mdhModel);
    if (!mdhResult.IsSuccess() ||
        mdhResult.context.normalized_model.modeling_mode != QStringLiteral("MDH"))
    {
        return 6;
    }

    auto urdfModel = MakeBaseModel();
    urdfModel.modeling_mode = QStringLiteral("URDF");
    urdfModel.urdf_source_path = QDir(QStringLiteral(ROBOSDP_TEST_PROJECT_SOURCE_DIR))
        .absoluteFilePath(QStringLiteral("resources/sample-projects/stage1-demo-project/kinematics/sample-6r-preview.urdf"));
    const auto urdfResult = adapter.BuildNormalizedContext(urdfModel);
    if (!urdfResult.IsSuccess() ||
        urdfResult.context.normalized_model.parameter_convention != QStringLiteral("URDF") ||
        !urdfResult.status.shared_robot_kernel_ready)
    {
        return 7;
    }

    auto jointCountMismatchModel = MakeBaseModel();
    jointCountMismatchModel.joint_count = 5;
    const auto jointCountMismatchResult = adapter.ValidateBuildContext(jointCountMismatchModel);
    if (jointCountMismatchResult.IsSuccess() ||
        jointCountMismatchResult.status.status_code != QStringLiteral("joint_count_links_mismatch"))
    {
        return 8;
    }

    auto invalidBaseFrameModel = MakeBaseModel();
    invalidBaseFrameModel.base_frame.position_m[0] = std::numeric_limits<double>::infinity();
    const auto invalidBaseFrameResult = adapter.ValidateBuildContext(invalidBaseFrameModel);
    if (invalidBaseFrameResult.IsSuccess() ||
        invalidBaseFrameResult.status.status_code != QStringLiteral("base_frame_position_invalid"))
    {
        return 9;
    }

    auto invalidFlangeFrameModel = MakeBaseModel();
    invalidFlangeFrameModel.flange_frame.rpy_deg[1] = std::numeric_limits<double>::infinity();
    const auto invalidFlangeFrameResult = adapter.ValidateBuildContext(invalidFlangeFrameModel);
    if (invalidFlangeFrameResult.IsSuccess() ||
        invalidFlangeFrameResult.status.status_code != QStringLiteral("flange_frame_orientation_invalid"))
    {
        return 10;
    }

    auto invalidTcpFrameModel = MakeBaseModel();
    invalidTcpFrameModel.tcp_frame.translation_m[2] = std::numeric_limits<double>::infinity();
    const auto invalidTcpFrameResult = adapter.ValidateBuildContext(invalidTcpFrameModel);
    if (invalidTcpFrameResult.IsSuccess() ||
        invalidTcpFrameResult.status.status_code != QStringLiteral("tcp_frame_position_invalid"))
    {
        return 11;
    }

    auto invalidUrdfModel = MakeBaseModel();
    invalidUrdfModel.modeling_mode = QStringLiteral("URDF");
    invalidUrdfModel.model_source_mode = QStringLiteral("urdf_imported");
    invalidUrdfModel.unified_robot_model_ref = QStringLiteral("invalid_urdf_probe");
    invalidUrdfModel.urdf_source_path = QDir(QStringLiteral(ROBOSDP_TEST_PROJECT_SOURCE_DIR))
        .absoluteFilePath(QStringLiteral("resources/sample-projects/stage1-demo-project/kinematics/not-exist.urdf"));
    const auto invalidUrdfResult = adapter.BuildNormalizedContext(invalidUrdfModel);
    if (invalidUrdfResult.IsSuccess() ||
        invalidUrdfResult.status.shared_robot_kernel_ready ||
        invalidUrdfResult.status.status_code != QStringLiteral("shared_kernel_urdf_file_not_found"))
    {
        return 12;
    }

    if (dhResult.context.joint_order_mapping.signature != QStringLiteral("joint_1|joint_2|joint_3|joint_4|joint_5|joint_6") ||
        dhResult.context.joint_order_mapping.ordered_joint_ids.size() != 6)
    {
        return 13;
    }

    const auto jacobianZero = adapter.EvaluateNativeJacobianDryRun(dhModel, std::vector<double>(6, 0.0));
    if (!jacobianZero.success || jacobianZero.rows != 6 || jacobianZero.cols != 6 ||
        jacobianZero.jacobian_matrix.size() != 36)
    {
        return 14;
    }

    const auto dimensionFailure = adapter.EvaluateNativeFkDryRun(dhModel, std::vector<double>(5, 0.0));
    if (dimensionFailure.success || dimensionFailure.message.trimmed().isEmpty())
    {
        return 15;
    }

    LocalJsonRepository repository;
    KinematicJsonStorage kinematicStorage(repository);
    TopologyJsonStorage topologyStorage(repository);
    KinematicsService service(kinematicStorage, topologyStorage, nullptr);

    FkRequestDto zeroFkRequest;
    zeroFkRequest.joint_positions_deg = std::vector<double>(6, 0.0);
    const auto zeroServiceFk = service.SolveFk(dhModel, zeroFkRequest);
    const auto zeroDryRunFk = adapter.EvaluateNativeFkDryRun(dhModel, zeroFkRequest.joint_positions_deg);
    if (!zeroServiceFk.success || !zeroDryRunFk.success ||
        !PosePositionNear(zeroServiceFk.tcp_pose, zeroDryRunFk.tcp_pose, 1.0e-4))
    {
        return 16;
    }

    FkRequestDto nonZeroFkRequest;
    nonZeroFkRequest.joint_positions_deg = {5.0, -8.0, 12.0, 4.0, -6.0, 10.0};
    const auto nonZeroServiceFk = service.SolveFk(dhModel, nonZeroFkRequest);
    const auto nonZeroDryRunFk = adapter.EvaluateNativeFkDryRun(dhModel, nonZeroFkRequest.joint_positions_deg);
    if (!nonZeroServiceFk.success || !nonZeroDryRunFk.success ||
        !PosePositionNear(nonZeroServiceFk.tcp_pose, nonZeroDryRunFk.tcp_pose, 1.0e-4) ||
        !PoseRpyNear(nonZeroServiceFk.tcp_pose, nonZeroDryRunFk.tcp_pose, 1.0e-4))
    {
        return 17;
    }

    FkRequestDto mdhFkRequest;
    mdhFkRequest.joint_positions_deg = {15.0, -10.0, 18.0, -6.0, 9.0, 12.0};
    const auto mdhServiceFk = service.SolveFk(mdhModel, mdhFkRequest);
    const auto mdhDryRunFk = adapter.EvaluateNativeFkDryRun(mdhModel, mdhFkRequest.joint_positions_deg);
    if (!mdhServiceFk.success || !mdhDryRunFk.success ||
        !PosePositionNear(mdhServiceFk.tcp_pose, mdhDryRunFk.tcp_pose, 1.0e-4) ||
        !PoseRpyNear(mdhServiceFk.tcp_pose, mdhDryRunFk.tcp_pose, 1.0e-4))
    {
        return 18;
    }

    // 当前 sample-6r-preview.urdf 是最小骨架样例，只包含 2 个可动关节。
    const auto urdfDryRunFk = adapter.EvaluateNativeFkDryRun(urdfModel, std::vector<double>(2, 0.0));
    if (!urdfDryRunFk.success)
    {
        return 19;
    }

    const QString projectRoot = QDir::current().absoluteFilePath(QStringLiteral("kinematics-build-context-stage16-project"));
    QDir(projectRoot).removeRecursively();
    if (repository.OpenProject(projectRoot) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 20;
    }

    const QString fixedUrdfPath = QDir(projectRoot).absoluteFilePath(QStringLiteral("fixed-joint.urdf"));
    const QString fixedUrdf = QStringLiteral(
        "<robot name=\"fixed_joint_robot\">"
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
        return 21;
    }

    auto fixedJointModel = MakeBaseModel();
    fixedJointModel.modeling_mode = QStringLiteral("URDF");
    fixedJointModel.urdf_source_path = fixedUrdfPath;
    fixedJointModel.joint_count = 1;
    fixedJointModel.links = {fixedJointModel.links.front()};
    fixedJointModel.joint_limits = {fixedJointModel.joint_limits.front()};
    fixedJointModel.joint_order_signature = QStringLiteral("joint_1");
    fixedJointModel.tcp_frame.translation_m = {0.0, 0.0, 0.0};
    const auto fixedJointFk = adapter.EvaluateNativeFkDryRun(fixedJointModel, std::vector<double>(1, 0.0));
    if (!fixedJointFk.success || !IsNear(fixedJointFk.tcp_pose.position_m[2], 0.75, 1.0e-4))
    {
        return 22;
    }

    const QString meshUrdfPath = QDir(projectRoot).absoluteFilePath(QStringLiteral("mesh-warning.urdf"));
    const QString meshUrdf = QStringLiteral(
        "<robot name=\"mesh_warning_robot\">"
        "<link name=\"base_link\">"
        "<visual><geometry><mesh filename=\"package://robot/meshes/base.dae\"/></geometry></visual>"
        "</link>"
        "<joint name=\"joint_1\" type=\"revolute\">"
        "<parent link=\"base_link\"/><child link=\"tool_link\"/>"
        "<axis xyz=\"0 0 1\"/><origin xyz=\"0 0 0.2\" rpy=\"0 0 0\"/>"
        "<limit lower=\"-3.14\" upper=\"3.14\" effort=\"1\" velocity=\"1\"/>"
        "</joint>"
        "<link name=\"tool_link\"/>"
        "</robot>");
    if (!WriteUtf8File(meshUrdfPath, meshUrdf))
    {
        return 23;
    }

    auto meshWarningModel = MakeBaseModel();
    meshWarningModel.modeling_mode = QStringLiteral("URDF");
    meshWarningModel.unified_robot_model_ref = QStringLiteral("mesh_warning_probe");
    meshWarningModel.urdf_source_path = meshUrdfPath;
    meshWarningModel.joint_count = 1;
    meshWarningModel.links = {meshWarningModel.links.front()};
    meshWarningModel.joint_limits = {meshWarningModel.joint_limits.front()};
    meshWarningModel.joint_order_signature = QStringLiteral("joint_1");
    const auto meshWarningResult = adapter.BuildNormalizedContext(meshWarningModel);
    if (!meshWarningResult.IsSuccess() ||
        !meshWarningResult.status.status_message.contains(QStringLiteral("仅支持 STL visual mesh")))
    {
        return 24;
    }

    const auto failingFkResult = service.SolveFk(invalidUrdfModel, zeroFkRequest);
    if (failingFkResult.success || failingFkResult.message.trimmed().isEmpty() ||
        failingFkResult.message.contains(QStringLiteral("Legacy")))
    {
        return 25;
    }

    return 0;
}
