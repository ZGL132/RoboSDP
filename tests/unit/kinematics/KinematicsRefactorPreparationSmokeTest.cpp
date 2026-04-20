#include "core/repository/LocalJsonRepository.h"
#include "modules/kinematics/adapter/PinocchioKinematicBackendAdapter.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"

#include <QCoreApplication>
#include <QDir>
#include <QJsonArray>
#include <QJsonObject>

namespace
{

QJsonArray ToArray3(const std::array<double, 3>& values)
{
    QJsonArray array;
    for (double value : values)
    {
        array.append(value);
    }
    return array;
}

QJsonArray ToArray2(const std::array<double, 2>& values)
{
    QJsonArray array;
    for (double value : values)
    {
        array.append(value);
    }
    return array;
}

QJsonObject BuildPoseObject(const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    QJsonObject object;
    object.insert(QStringLiteral("position_m"), ToArray3(pose.position_m));
    object.insert(QStringLiteral("rpy_deg"), ToArray3(pose.rpy_deg));
    return object;
}

QJsonObject BuildLegacyModelDocument(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    QJsonObject rootObject;

    QJsonObject metaObject;
    metaObject.insert(QStringLiteral("kinematic_id"), model.meta.kinematic_id);
    metaObject.insert(QStringLiteral("name"), model.meta.name);
    metaObject.insert(QStringLiteral("version"), model.meta.version);
    metaObject.insert(QStringLiteral("source"), model.meta.source);
    metaObject.insert(QStringLiteral("status"), model.meta.status);
    metaObject.insert(QStringLiteral("topology_ref"), model.meta.topology_ref);
    metaObject.insert(QStringLiteral("requirement_ref"), model.meta.requirement_ref);
    rootObject.insert(QStringLiteral("meta"), metaObject);

    rootObject.insert(QStringLiteral("parameter_convention"), model.parameter_convention);
    rootObject.insert(QStringLiteral("joint_count"), model.joint_count);
    rootObject.insert(QStringLiteral("base_frame"), BuildPoseObject(model.base_frame));
    rootObject.insert(QStringLiteral("flange_frame"), BuildPoseObject(model.flange_frame));
    rootObject.insert(QStringLiteral("tool_frame"), QJsonValue(QJsonValue::Null));
    rootObject.insert(QStringLiteral("workpiece_frame"), QJsonValue(QJsonValue::Null));

    QJsonArray linksArray;
    for (const auto& link : model.links)
    {
        QJsonObject linkObject;
        linkObject.insert(QStringLiteral("link_id"), link.link_id);
        linkObject.insert(QStringLiteral("a"), link.a);
        linkObject.insert(QStringLiteral("alpha"), link.alpha);
        linkObject.insert(QStringLiteral("d"), link.d);
        linkObject.insert(QStringLiteral("theta_offset"), link.theta_offset);
        linksArray.append(linkObject);
    }
    rootObject.insert(QStringLiteral("links"), linksArray);

    QJsonArray jointLimitArray;
    for (const auto& jointLimit : model.joint_limits)
    {
        QJsonObject limitObject;
        limitObject.insert(QStringLiteral("joint_id"), jointLimit.joint_id);
        limitObject.insert(QStringLiteral("soft_limit"), ToArray2(jointLimit.soft_limit));
        limitObject.insert(QStringLiteral("hard_limit"), ToArray2(jointLimit.hard_limit));
        limitObject.insert(QStringLiteral("max_velocity"), jointLimit.max_velocity);
        limitObject.insert(QStringLiteral("max_acceleration"), jointLimit.max_acceleration);
        jointLimitArray.append(limitObject);
    }
    rootObject.insert(QStringLiteral("joint_limits"), jointLimitArray);

    QJsonObject tcpFrameObject;
    tcpFrameObject.insert(QStringLiteral("translation_m"), ToArray3(model.tcp_frame.translation_m));
    tcpFrameObject.insert(QStringLiteral("rpy_deg"), ToArray3(model.tcp_frame.rpy_deg));
    rootObject.insert(QStringLiteral("tcp_frame"), tcpFrameObject);

    QJsonObject ikConfigObject;
    ikConfigObject.insert(QStringLiteral("solver_type"), model.ik_solver_config.solver_type);
    ikConfigObject.insert(QStringLiteral("branch_policy"), model.ik_solver_config.branch_policy);
    ikConfigObject.insert(QStringLiteral("max_iterations"), model.ik_solver_config.max_iterations);
    ikConfigObject.insert(QStringLiteral("position_tolerance_mm"), model.ik_solver_config.position_tolerance_mm);
    ikConfigObject.insert(QStringLiteral("orientation_tolerance_deg"), model.ik_solver_config.orientation_tolerance_deg);
    ikConfigObject.insert(QStringLiteral("step_gain"), model.ik_solver_config.step_gain);
    rootObject.insert(QStringLiteral("ik_solver_config"), ikConfigObject);

    rootObject.insert(QStringLiteral("last_fk_result"), QJsonObject());
    rootObject.insert(QStringLiteral("last_ik_result"), QJsonObject());
    return rootObject;
}

RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto BuildFullSemanticState()
{
    using namespace RoboSDP::Kinematics::Dto;

    // 该状态用于验证“统一机器人模型”新增语义字段在 Save/Load 往返后不会丢失。
    KinematicsWorkspaceStateDto state = KinematicsWorkspaceStateDto::CreateDefault();
    state.current_model.modeling_mode = QStringLiteral("URDF");
    state.current_model.parameter_convention = QStringLiteral("URDF");
    state.current_model.unified_robot_model_ref = QStringLiteral("robot://stage18/persistence-roundtrip");
    state.current_model.model_source_mode = QStringLiteral("urdf_imported");
    state.current_model.backend_type = QStringLiteral("pinocchio_kinematic_backend");
    state.current_model.urdf_source_path =
        QStringLiteral("D:/10_Source_Repos/21_robot/RoboSDP_v1.0/resources/sample-projects/stage1-demo-project/kinematics/sample-6r-preview.urdf");
    state.current_model.pinocchio_model_ready = true;
    state.current_model.frame_semantics_version = 3;
    state.current_model.joint_order_signature = QStringLiteral("axis_a|axis_b|axis_c|axis_d|axis_e|axis_f");
    state.backend_summary.default_backend_type = QStringLiteral("pinocchio_kinematic_backend");
    state.backend_summary.active_backend_type = QStringLiteral("pinocchio_kinematic_backend");
    state.backend_summary.shared_kernel_stage = QStringLiteral("pinocchio_primary");
    state.backend_summary.shared_robot_kernel_ready = true;
    state.backend_summary.status_message = QStringLiteral("持久化专项测试：共享内核已完成建树。");
    state.current_model.meta.name = QStringLiteral("Stage18 Json RoundTrip");
    state.current_model.meta.topology_ref = QStringLiteral("topology://roundtrip");
    state.current_model.meta.requirement_ref = QStringLiteral("requirement://roundtrip");
    return state;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    using namespace RoboSDP::Errors;
    using namespace RoboSDP::Kinematics::Adapter;
    using namespace RoboSDP::Kinematics::Dto;
    using namespace RoboSDP::Kinematics::Persistence;
    using namespace RoboSDP::Repository;

    const KinematicModelDto defaultModel = KinematicModelDto::CreateDefault();
    if (defaultModel.modeling_mode != QStringLiteral("DH"))
    {
        return 1;
    }
    if (!defaultModel.unified_robot_model_ref.isEmpty())
    {
        return 2;
    }
    if (defaultModel.model_source_mode != QStringLiteral("manual_seed"))
    {
        return 3;
    }
    if (defaultModel.backend_type != QStringLiteral("pinocchio_kinematic_backend"))
    {
        return 4;
    }
    if (!defaultModel.urdf_source_path.isEmpty() || defaultModel.pinocchio_model_ready)
    {
        return 5;
    }
    if (defaultModel.frame_semantics_version != 1)
    {
        return 6;
    }
    if (defaultModel.joint_order_signature != QStringLiteral("joint_1|joint_2|joint_3|joint_4|joint_5|joint_6"))
    {
        return 7;
    }

    const QString projectRoot =
        QDir::current().absoluteFilePath(QStringLiteral("kinematics-refactor-prep-smoke-project"));
    QDir(projectRoot).removeRecursively();

    LocalJsonRepository repository;
    if (repository.OpenProject(projectRoot) != ErrorCode::Ok)
    {
        return 8;
    }

    const QJsonObject legacyDocument = BuildLegacyModelDocument(defaultModel);
    if (repository.WriteDocument(QStringLiteral("kinematics/kinematic-model.json"), legacyDocument) != ErrorCode::Ok)
    {
        return 9;
    }

    KinematicJsonStorage storage(repository);
    const KinematicsWorkspaceStateDto roundTripSourceState = BuildFullSemanticState();
    if (storage.SaveModel(projectRoot, roundTripSourceState) != ErrorCode::Ok)
    {
        return 10;
    }

    KinematicsWorkspaceStateDto roundTripLoadedState;
    if (storage.LoadModel(projectRoot, roundTripLoadedState) != ErrorCode::Ok)
    {
        return 11;
    }
    if (roundTripLoadedState.current_model.modeling_mode != roundTripSourceState.current_model.modeling_mode ||
        roundTripLoadedState.current_model.parameter_convention !=
            roundTripSourceState.current_model.parameter_convention ||
        roundTripLoadedState.current_model.unified_robot_model_ref !=
            roundTripSourceState.current_model.unified_robot_model_ref ||
        roundTripLoadedState.current_model.model_source_mode !=
            roundTripSourceState.current_model.model_source_mode ||
        roundTripLoadedState.current_model.backend_type != roundTripSourceState.current_model.backend_type ||
        roundTripLoadedState.current_model.urdf_source_path != roundTripSourceState.current_model.urdf_source_path ||
        roundTripLoadedState.current_model.frame_semantics_version !=
            roundTripSourceState.current_model.frame_semantics_version ||
        roundTripLoadedState.current_model.joint_order_signature !=
            roundTripSourceState.current_model.joint_order_signature)
    {
        return 12;
    }
    if (roundTripLoadedState.backend_summary.status_message !=
            roundTripSourceState.backend_summary.status_message ||
        !roundTripLoadedState.backend_summary.shared_robot_kernel_ready)
    {
        return 13;
    }

    if (repository.WriteDocument(QStringLiteral("kinematics/kinematic-model.json"), legacyDocument) != ErrorCode::Ok)
    {
        return 14;
    }

    KinematicsWorkspaceStateDto loadedState;
    if (storage.LoadModel(projectRoot, loadedState) != ErrorCode::Ok)
    {
        return 15;
    }
    if (loadedState.current_model.modeling_mode != QStringLiteral("DH"))
    {
        return 16;
    }
    if (loadedState.current_model.backend_type != QStringLiteral("pinocchio_kinematic_backend"))
    {
        return 17;
    }
    if (loadedState.current_model.model_source_mode != QStringLiteral("manual_seed"))
    {
        return 18;
    }
    if (loadedState.current_model.frame_semantics_version != 1)
    {
        return 19;
    }
    if (loadedState.current_model.joint_order_signature != defaultModel.joint_order_signature)
    {
        return 20;
    }
    if (loadedState.backend_summary.default_backend_type != QStringLiteral("pinocchio_kinematic_backend") ||
        !loadedState.backend_summary.status_message.contains(QStringLiteral("Pinocchio")))
    {
        return 21;
    }

    PinocchioKinematicBackendAdapter adapter;
    const auto legacyBuildContextResult = adapter.BuildNormalizedContext(loadedState.current_model);
    if (!legacyBuildContextResult.IsSuccess() || !legacyBuildContextResult.status.shared_robot_kernel_ready)
    {
        return 22;
    }

    if (storage.SaveModel(projectRoot, loadedState) != ErrorCode::Ok)
    {
        return 23;
    }

    QJsonObject upgradedDocument;
    if (repository.ReadDocument(QStringLiteral("kinematics/kinematic-model.json"), upgradedDocument) != ErrorCode::Ok)
    {
        return 24;
    }
    if (!upgradedDocument.contains(QStringLiteral("modeling_mode")) ||
        !upgradedDocument.contains(QStringLiteral("backend_type")) ||
        !upgradedDocument.contains(QStringLiteral("joint_order_signature")) ||
        !upgradedDocument.contains(QStringLiteral("backend_summary")) ||
        upgradedDocument.value(QStringLiteral("modeling_mode")).toString() != QStringLiteral("DH") ||
        upgradedDocument.value(QStringLiteral("parameter_convention")).toString() != QStringLiteral("DH") ||
        upgradedDocument.value(QStringLiteral("model_source_mode")).toString() != QStringLiteral("manual_seed") ||
        upgradedDocument.value(QStringLiteral("frame_semantics_version")).toInt() != 1)
    {
        return 25;
    }

    const auto currentBuildResult = adapter.BuildNormalizedContext(defaultModel);
    if (!currentBuildResult.IsSuccess() ||
        !currentBuildResult.status.shared_robot_kernel_ready)
    {
        return 26;
    }
    const QString currentStatusMessage = currentBuildResult.status.status_message;
    if (!currentStatusMessage.contains(QStringLiteral("Pinocchio Model/Data 已在内存中实例化")) &&
        !currentStatusMessage.contains(QStringLiteral("已命中缓存")) &&
        !currentStatusMessage.contains(QStringLiteral("均命中缓存")) &&
        !currentStatusMessage.contains(QStringLiteral("共享注册表 Model")) &&
        !currentStatusMessage.contains(QStringLiteral("共享注册表 Model 已接入")) &&
        !currentStatusMessage.contains(QStringLiteral("SharedRobotKernelRegistry")))
    {
        return 27;
    }
    if (!adapter.UsesSharedRobotKernel())
    {
        return 28;
    }

    FkRequestDto fkRequest;
    fkRequest.joint_positions_deg = {0.0, 10.0, 20.0, 30.0, 40.0, 50.0};
    const FkResultDto fkResult = adapter.SolveFk(defaultModel, fkRequest);
    if (!fkResult.success || !fkResult.message.contains(QStringLiteral("Pinocchio 原生内核")))
    {
        return 29;
    }

    WorkspaceRequestDto workspaceRequest;
    workspaceRequest.sample_count = 16;
    const WorkspaceResultDto workspaceResult = adapter.SampleWorkspace(defaultModel, workspaceRequest);
    if (!workspaceResult.success ||
        workspaceResult.requested_sample_count != 16 ||
        workspaceResult.reachable_sample_count != 16 ||
        workspaceResult.sampled_points.size() != 16 ||
        !workspaceResult.message.contains(QStringLiteral("Pinocchio 引擎工作空间采样完成")))
    {
        return 30;
    }

    return 0;
}
