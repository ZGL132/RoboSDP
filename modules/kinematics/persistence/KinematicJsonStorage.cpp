#include "modules/kinematics/persistence/KinematicJsonStorage.h"

#include <QDir>
#include <QJsonArray>
#include <QJsonObject>

#include <algorithm>

namespace RoboSDP::Kinematics::Persistence
{

namespace
{

QJsonArray ToJsonArray2(const std::array<double, 2>& values)
{
    QJsonArray array;
    for (double value : values)
    {
        array.append(value);
    }
    return array;
}

QJsonArray ToJsonArray3(const std::array<double, 3>& values)
{
    QJsonArray array;
    for (double value : values)
    {
        array.append(value);
    }
    return array;
}

QJsonArray ToJsonArray16(const std::array<double, 16>& values)
{
    QJsonArray array;
    for (double value : values)
    {
        array.append(value);
    }
    return array;
}

QJsonArray ToVectorArray(const std::vector<double>& values)
{
    QJsonArray array;
    for (double value : values)
    {
        array.append(value);
    }
    return array;
}

std::array<double, 2> ReadArray2(const QJsonArray& array)
{
    std::array<double, 2> values {0.0, 0.0};
    for (int index = 0; index < array.size() && index < 2; ++index)
    {
        values[static_cast<std::size_t>(index)] = array.at(index).toDouble();
    }
    return values;
}

std::array<double, 3> ReadArray3(const QJsonArray& array)
{
    std::array<double, 3> values {0.0, 0.0, 0.0};
    for (int index = 0; index < array.size() && index < 3; ++index)
    {
        values[static_cast<std::size_t>(index)] = array.at(index).toDouble();
    }
    return values;
}

std::array<double, 16> ReadArray16(const QJsonArray& array)
{
    std::array<double, 16> values {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0};
    for (int index = 0; index < array.size() && index < 16; ++index)
    {
        values[static_cast<std::size_t>(index)] = array.at(index).toDouble();
    }
    return values;
}

std::vector<double> ReadVectorArray(const QJsonArray& array)
{
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(array.size()));
    for (const QJsonValue& value : array)
    {
        values.push_back(value.toDouble());
    }
    return values;
}

QString ReadString(const QJsonObject& object, const QString& key, const QString& defaultValue = {})
{
    return object.contains(key) ? object.value(key).toString(defaultValue) : defaultValue;
}

int ReadInt(const QJsonObject& object, const QString& key, int defaultValue = 0)
{
    return object.contains(key) ? object.value(key).toInt(defaultValue) : defaultValue;
}

double ReadDouble(const QJsonObject& object, const QString& key, double defaultValue = 0.0)
{
    return object.contains(key) ? object.value(key).toDouble(defaultValue) : defaultValue;
}

bool ReadBool(const QJsonObject& object, const QString& key, bool defaultValue = false)
{
    return object.contains(key) ? object.value(key).toBool(defaultValue) : defaultValue;
}

/**
 * @brief 为缺失 joint_order_signature 的旧模型动态补齐关节顺序签名。
 * @details
 * 老版本 JSON 中没有该字段时，后续共享内核会失去稳定的关节顺序语义。
 * 这里优先使用 joint_limits 里的 joint_id 生成签名；若连限位也缺失，再按 joint_count 兜底。
 */
QString BuildJointOrderSignature(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    QString signature;
    for (std::size_t index = 0; index < model.joint_limits.size(); ++index)
    {
        if (index > 0)
        {
            signature.append(QLatin1Char('|'));
        }
        signature.append(model.joint_limits[index].joint_id);
    }

    if (!signature.isEmpty())
    {
        return signature;
    }

    for (int index = 0; index < model.joint_count; ++index)
    {
        if (index > 0)
        {
            signature.append(QLatin1Char('|'));
        }
        signature.append(QStringLiteral("joint_%1").arg(index + 1));
    }
    return signature;
}

/**
 * @brief 推断模型建模模式的兼容口径。
 * @details
 * 老 JSON 可能只保存了 parameter_convention，或者只留下了 urdf_source_path。
 * 该函数保证读取结果始终落在 DH / MDH / URDF 三种稳定语义之一。
 */
QString InferModelingMode(
    const QString& explicitModelingMode,
    const QString& parameterConvention,
    const QString& urdfSourcePath)
{
    if (!explicitModelingMode.trimmed().isEmpty())
    {
        return explicitModelingMode;
    }

    if (!urdfSourcePath.trimmed().isEmpty())
    {
        return QStringLiteral("URDF");
    }

    if (parameterConvention == QStringLiteral("MDH"))
    {
        return QStringLiteral("MDH");
    }

    return QStringLiteral("DH");
}

/**
 * @brief 兼容推断参数约定字段，避免旧 JSON 缺省时留下空字符串。
 */
QString InferParameterConvention(
    const QString& explicitParameterConvention,
    const QString& modelingMode,
    const QString& urdfSourcePath)
{
    if (!explicitParameterConvention.trimmed().isEmpty())
    {
        return explicitParameterConvention.trimmed();
    }

    if (modelingMode == QStringLiteral("URDF") || !urdfSourcePath.trimmed().isEmpty())
    {
        return QStringLiteral("URDF");
    }

    if (modelingMode == QStringLiteral("MDH"))
    {
        return QStringLiteral("MDH");
    }

    return QStringLiteral("DH");
}

/**
 * @brief 兼容推断模型来源模式，保证旧项目读取后有稳定来源语义。
 */
QString InferModelSourceMode(const QString& explicitSourceMode, const QString& urdfSourcePath)
{
    if (!explicitSourceMode.trimmed().isEmpty())
    {
        return explicitSourceMode.trimmed();
    }

    return urdfSourcePath.trimmed().isEmpty()
        ? QStringLiteral("manual_seed")
        : QStringLiteral("urdf_imported");
}

/**
 * @brief 兼容推断后端类型，确保旧 JSON 升级后仍默认指向当前共享内核后端。
 */
QString InferBackendType(const QString& explicitBackendType)
{
    return explicitBackendType.trimmed().isEmpty()
        ? QStringLiteral("pinocchio_kinematic_backend")
        : explicitBackendType.trimmed();
}

QJsonObject ToPoseObject(const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    QJsonObject object;
    object.insert(QStringLiteral("position_m"), ToJsonArray3(pose.position_m));
    object.insert(QStringLiteral("rpy_deg"), ToJsonArray3(pose.rpy_deg));
    return object;
}

RoboSDP::Kinematics::Dto::CartesianPoseDto FromPoseObject(const QJsonObject& object)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = ReadArray3(object.value(QStringLiteral("position_m")).toArray());
    pose.rpy_deg = ReadArray3(object.value(QStringLiteral("rpy_deg")).toArray());
    return pose;
}

QJsonObject ToFkResultObject(const RoboSDP::Kinematics::Dto::FkResultDto& result)
{
    QJsonObject object;
    object.insert(QStringLiteral("success"), result.success);
    object.insert(QStringLiteral("message"), result.message);
    object.insert(QStringLiteral("joint_positions_deg"), ToVectorArray(result.joint_positions_deg));
    object.insert(QStringLiteral("tcp_pose"), ToPoseObject(result.tcp_pose));
    object.insert(QStringLiteral("tcp_transform_matrix"), ToJsonArray16(result.tcp_transform_matrix));

    QJsonArray linkArray;
    for (const auto& linkPose : result.link_poses)
    {
        QJsonObject linkObject;
        linkObject.insert(QStringLiteral("link_id"), linkPose.link_id);
        linkObject.insert(QStringLiteral("pose"), ToPoseObject(linkPose.pose));
        linkArray.append(linkObject);
    }
    object.insert(QStringLiteral("link_poses"), linkArray);
    return object;
}

RoboSDP::Kinematics::Dto::FkResultDto FromFkResultObject(const QJsonObject& object)
{
    RoboSDP::Kinematics::Dto::FkResultDto result;
    result.success = ReadBool(object, QStringLiteral("success"));
    result.message = ReadString(object, QStringLiteral("message"));
    result.joint_positions_deg = ReadVectorArray(object.value(QStringLiteral("joint_positions_deg")).toArray());
    result.tcp_pose = FromPoseObject(object.value(QStringLiteral("tcp_pose")).toObject());
    result.tcp_transform_matrix = ReadArray16(object.value(QStringLiteral("tcp_transform_matrix")).toArray());

    const QJsonArray linkArray = object.value(QStringLiteral("link_poses")).toArray();
    for (const QJsonValue& value : linkArray)
    {
        const QJsonObject linkObject = value.toObject();
        result.link_poses.push_back({
            ReadString(linkObject, QStringLiteral("link_id")),
            FromPoseObject(linkObject.value(QStringLiteral("pose")).toObject())});
    }
    return result;
}

QJsonObject ToIkResultObject(const RoboSDP::Kinematics::Dto::IkResultDto& result)
{
    QJsonObject object;
    object.insert(QStringLiteral("success"), result.success);
    object.insert(QStringLiteral("message"), result.message);
    object.insert(QStringLiteral("joint_positions_deg"), ToVectorArray(result.joint_positions_deg));
    object.insert(QStringLiteral("position_error_mm"), result.position_error_mm);
    object.insert(QStringLiteral("orientation_error_deg"), result.orientation_error_deg);
    object.insert(QStringLiteral("iteration_count"), result.iteration_count);
    return object;
}

RoboSDP::Kinematics::Dto::IkResultDto FromIkResultObject(const QJsonObject& object)
{
    RoboSDP::Kinematics::Dto::IkResultDto result;
    result.success = ReadBool(object, QStringLiteral("success"));
    result.message = ReadString(object, QStringLiteral("message"));
    result.joint_positions_deg = ReadVectorArray(object.value(QStringLiteral("joint_positions_deg")).toArray());
    result.position_error_mm = ReadDouble(object, QStringLiteral("position_error_mm"));
    result.orientation_error_deg = ReadDouble(object, QStringLiteral("orientation_error_deg"));
    result.iteration_count = ReadInt(object, QStringLiteral("iteration_count"));
    return result;
}

QJsonObject ToBackendSummaryObject(const RoboSDP::Kinematics::Dto::KinematicBackendStateSummaryDto& summary)
{
    QJsonObject object;
    object.insert(QStringLiteral("default_backend_type"), summary.default_backend_type);
    object.insert(QStringLiteral("active_backend_type"), summary.active_backend_type);
    object.insert(QStringLiteral("shared_kernel_stage"), summary.shared_kernel_stage);
    object.insert(QStringLiteral("shared_robot_kernel_ready"), summary.shared_robot_kernel_ready);
    object.insert(QStringLiteral("status_message"), summary.status_message);
    return object;
}

RoboSDP::Kinematics::Dto::KinematicBackendStateSummaryDto FromBackendSummaryObject(
    const QJsonObject& object,
    const RoboSDP::Kinematics::Dto::KinematicBackendStateSummaryDto& defaultSummary,
    const QString& fallbackBackendType)
{
    RoboSDP::Kinematics::Dto::KinematicBackendStateSummaryDto summary = defaultSummary;
    summary.default_backend_type =
        ReadString(object, QStringLiteral("default_backend_type"), fallbackBackendType);
    summary.active_backend_type =
        ReadString(object, QStringLiteral("active_backend_type"), summary.default_backend_type);
    summary.shared_kernel_stage =
        ReadString(object, QStringLiteral("shared_kernel_stage"), summary.shared_kernel_stage);
    summary.shared_robot_kernel_ready =
        ReadBool(object, QStringLiteral("shared_robot_kernel_ready"), summary.shared_robot_kernel_ready);
    summary.status_message =
        ReadString(object, QStringLiteral("status_message"), summary.status_message);
    return summary;
}

} // namespace

KinematicJsonStorage::KinematicJsonStorage(RoboSDP::Repository::IJsonRepository& repository)
    : m_repository(repository)
{
}

RoboSDP::Errors::ErrorCode KinematicJsonStorage::SaveModel(
    const QString& projectRootPath,
    const RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& state) const
{
    const RoboSDP::Errors::ErrorCode openProjectError = m_repository.OpenProject(projectRootPath);
    if (openProjectError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return openProjectError;
    }

    try
    {
        return m_repository.WriteDocument(RelativeModelFilePath(), ToModelJsonObject(state));
    }
    catch (const std::exception&)
    {
        return RoboSDP::Errors::ErrorCode::JsonFormatInvalid;
    }
    catch (...)
    {
        return RoboSDP::Errors::ErrorCode::UnknownError;
    }
}

RoboSDP::Errors::ErrorCode KinematicJsonStorage::LoadModel(
    const QString& projectRootPath,
    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& state) const
{
    const RoboSDP::Errors::ErrorCode openProjectError = m_repository.OpenProject(projectRootPath);
    if (openProjectError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return openProjectError;
    }

    QJsonObject jsonObject;
    const RoboSDP::Errors::ErrorCode readError = m_repository.ReadDocument(RelativeModelFilePath(), jsonObject);
    if (readError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return readError;
    }

    try
    {
        state = FromModelJsonObject(jsonObject);
        return RoboSDP::Errors::ErrorCode::Ok;
    }
    catch (const std::exception&)
    {
        return RoboSDP::Errors::ErrorCode::JsonFormatInvalid;
    }
    catch (...)
    {
        return RoboSDP::Errors::ErrorCode::UnknownError;
    }
}

RoboSDP::Errors::ErrorCode KinematicJsonStorage::SaveWorkspaceCache(
    const QString& projectRootPath,
    const RoboSDP::Kinematics::Dto::WorkspaceResultDto& result) const
{
    const RoboSDP::Errors::ErrorCode openProjectError = m_repository.OpenProject(projectRootPath);
    if (openProjectError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return openProjectError;
    }

    try
    {
        return m_repository.WriteDocument(RelativeWorkspaceFilePath(), ToWorkspaceJsonObject(result));
    }
    catch (const std::exception&)
    {
        return RoboSDP::Errors::ErrorCode::JsonFormatInvalid;
    }
    catch (...)
    {
        return RoboSDP::Errors::ErrorCode::UnknownError;
    }
}

RoboSDP::Errors::ErrorCode KinematicJsonStorage::LoadWorkspaceCache(
    const QString& projectRootPath,
    RoboSDP::Kinematics::Dto::WorkspaceResultDto& result) const
{
    const RoboSDP::Errors::ErrorCode openProjectError = m_repository.OpenProject(projectRootPath);
    if (openProjectError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return openProjectError;
    }

    QJsonObject jsonObject;
    const RoboSDP::Errors::ErrorCode readError =
        m_repository.ReadDocument(RelativeWorkspaceFilePath(), jsonObject);
    if (readError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return readError;
    }

    try
    {
        result = FromWorkspaceJsonObject(jsonObject);
        return RoboSDP::Errors::ErrorCode::Ok;
    }
    catch (const std::exception&)
    {
        return RoboSDP::Errors::ErrorCode::JsonFormatInvalid;
    }
    catch (...)
    {
        return RoboSDP::Errors::ErrorCode::UnknownError;
    }
}

QString KinematicJsonStorage::RelativeModelFilePath() const
{
    return QStringLiteral("kinematics/kinematic-model.json");
}

QString KinematicJsonStorage::RelativeWorkspaceFilePath() const
{
    return QStringLiteral("kinematics/workspace-cache.json");
}

QString KinematicJsonStorage::BuildAbsoluteModelFilePath(const QString& projectRootPath) const
{
    return QDir(projectRootPath).filePath(RelativeModelFilePath());
}

QString KinematicJsonStorage::BuildAbsoluteWorkspaceFilePath(const QString& projectRootPath) const
{
    return QDir(projectRootPath).filePath(RelativeWorkspaceFilePath());
}

QJsonObject KinematicJsonStorage::ToModelJsonObject(
    const RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& state) const
{
    QJsonObject rootObject;
    const QString normalizedParameterConvention = InferParameterConvention(
        state.current_model.parameter_convention,
        state.current_model.modeling_mode,
        state.current_model.urdf_source_path);
    const QString normalizedModelingMode = InferModelingMode(
        state.current_model.modeling_mode,
        normalizedParameterConvention,
        state.current_model.urdf_source_path);
    const QString normalizedModelSourceMode = InferModelSourceMode(
        state.current_model.model_source_mode,
        state.current_model.urdf_source_path);
    const QString normalizedBackendType = InferBackendType(state.current_model.backend_type);
    const QString normalizedJointOrderSignature = state.current_model.joint_order_signature.trimmed().isEmpty()
        ? BuildJointOrderSignature(state.current_model)
        : state.current_model.joint_order_signature.trimmed();
    const int normalizedFrameSemanticsVersion =
        state.current_model.frame_semantics_version <= 0 ? 1 : state.current_model.frame_semantics_version;

    QJsonObject metaObject;
    metaObject.insert(QStringLiteral("kinematic_id"), state.current_model.meta.kinematic_id);
    metaObject.insert(QStringLiteral("name"), state.current_model.meta.name);
    metaObject.insert(QStringLiteral("version"), state.current_model.meta.version);
    metaObject.insert(QStringLiteral("source"), state.current_model.meta.source);
    metaObject.insert(QStringLiteral("status"), state.current_model.meta.status);
    metaObject.insert(QStringLiteral("topology_ref"), state.current_model.meta.topology_ref);
    metaObject.insert(QStringLiteral("requirement_ref"), state.current_model.meta.requirement_ref);
    rootObject.insert(QStringLiteral("meta"), metaObject);

    rootObject.insert(QStringLiteral("modeling_mode"), normalizedModelingMode);
    rootObject.insert(QStringLiteral("unified_robot_model_ref"), state.current_model.unified_robot_model_ref);
    rootObject.insert(QStringLiteral("model_source_mode"), normalizedModelSourceMode);
    rootObject.insert(QStringLiteral("backend_type"), normalizedBackendType);
    rootObject.insert(QStringLiteral("parameter_convention"), normalizedParameterConvention);
    rootObject.insert(QStringLiteral("joint_count"), state.current_model.joint_count);
    rootObject.insert(QStringLiteral("urdf_source_path"), state.current_model.urdf_source_path);
    rootObject.insert(QStringLiteral("pinocchio_model_ready"), state.current_model.pinocchio_model_ready);
    rootObject.insert(QStringLiteral("frame_semantics_version"), normalizedFrameSemanticsVersion);
    rootObject.insert(QStringLiteral("joint_order_signature"), normalizedJointOrderSignature);
    rootObject.insert(QStringLiteral("base_frame"), ToPoseObject(state.current_model.base_frame));
    rootObject.insert(QStringLiteral("flange_frame"), ToPoseObject(state.current_model.flange_frame));
    rootObject.insert(
        QStringLiteral("tool_frame"),
        state.current_model.tool_frame.has_value()
            ? QJsonValue(ToPoseObject(state.current_model.tool_frame.value()))
            : QJsonValue(QJsonValue::Null));
    rootObject.insert(
        QStringLiteral("workpiece_frame"),
        state.current_model.workpiece_frame.has_value()
            ? QJsonValue(ToPoseObject(state.current_model.workpiece_frame.value()))
            : QJsonValue(QJsonValue::Null));

    QJsonArray linksArray;
    for (const auto& link : state.current_model.links)
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

    QJsonArray limitArray;
    for (const auto& limit : state.current_model.joint_limits)
    {
        QJsonObject limitObject;
        limitObject.insert(QStringLiteral("joint_id"), limit.joint_id);
        limitObject.insert(QStringLiteral("soft_limit"), ToJsonArray2(limit.soft_limit));
        limitObject.insert(QStringLiteral("hard_limit"), ToJsonArray2(limit.hard_limit));
        limitObject.insert(QStringLiteral("max_velocity"), limit.max_velocity);
        limitObject.insert(QStringLiteral("max_acceleration"), limit.max_acceleration);
        limitArray.append(limitObject);
    }
    rootObject.insert(QStringLiteral("joint_limits"), limitArray);

    QJsonObject tcpObject;
    tcpObject.insert(QStringLiteral("translation_m"), ToJsonArray3(state.current_model.tcp_frame.translation_m));
    tcpObject.insert(QStringLiteral("rpy_deg"), ToJsonArray3(state.current_model.tcp_frame.rpy_deg));
    rootObject.insert(QStringLiteral("tcp_frame"), tcpObject);

    QJsonObject ikConfigObject;
    ikConfigObject.insert(QStringLiteral("solver_type"), state.current_model.ik_solver_config.solver_type);
    ikConfigObject.insert(QStringLiteral("branch_policy"), state.current_model.ik_solver_config.branch_policy);
    ikConfigObject.insert(QStringLiteral("max_iterations"), state.current_model.ik_solver_config.max_iterations);
    ikConfigObject.insert(
        QStringLiteral("position_tolerance_mm"),
        state.current_model.ik_solver_config.position_tolerance_mm);
    ikConfigObject.insert(
        QStringLiteral("orientation_tolerance_deg"),
        state.current_model.ik_solver_config.orientation_tolerance_deg);
    ikConfigObject.insert(QStringLiteral("step_gain"), state.current_model.ik_solver_config.step_gain);
    rootObject.insert(QStringLiteral("ik_solver_config"), ikConfigObject);

    rootObject.insert(QStringLiteral("last_fk_result"), ToFkResultObject(state.last_fk_result));
    rootObject.insert(QStringLiteral("last_ik_result"), ToIkResultObject(state.last_ik_result));
    rootObject.insert(QStringLiteral("backend_summary"), ToBackendSummaryObject(state.backend_summary));

    return rootObject;
}

RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto KinematicJsonStorage::FromModelJsonObject(
    const QJsonObject& jsonObject) const
{
    using namespace RoboSDP::Kinematics::Dto;

    KinematicsWorkspaceStateDto state = KinematicsWorkspaceStateDto::CreateDefault();

    const QJsonObject metaObject = jsonObject.value(QStringLiteral("meta")).toObject();
    state.current_model.meta.kinematic_id =
        ReadString(metaObject, QStringLiteral("kinematic_id"), state.current_model.meta.kinematic_id);
    state.current_model.meta.name =
        ReadString(metaObject, QStringLiteral("name"), state.current_model.meta.name);
    state.current_model.meta.version =
        ReadInt(metaObject, QStringLiteral("version"), state.current_model.meta.version);
    state.current_model.meta.source =
        ReadString(metaObject, QStringLiteral("source"), state.current_model.meta.source);
    state.current_model.meta.status =
        ReadString(metaObject, QStringLiteral("status"), state.current_model.meta.status);
    state.current_model.meta.topology_ref = ReadString(metaObject, QStringLiteral("topology_ref"));
    state.current_model.meta.requirement_ref = ReadString(metaObject, QStringLiteral("requirement_ref"));

    // 老版本 JSON 可能只保存了部分统一语义字段，这里先把“来源路径”和“显示声明”读出来，
    // 后续再按统一规则补齐 parameter_convention / modeling_mode / backend_type 等关键元数据。
    const QString explicitModelingMode = ReadString(jsonObject, QStringLiteral("modeling_mode"));
    state.current_model.urdf_source_path =
        ReadString(jsonObject, QStringLiteral("urdf_source_path"), state.current_model.urdf_source_path);
    state.current_model.parameter_convention = InferParameterConvention(
        ReadString(jsonObject, QStringLiteral("parameter_convention")),
        explicitModelingMode,
        state.current_model.urdf_source_path);
    state.current_model.joint_count =
        ReadInt(jsonObject, QStringLiteral("joint_count"), state.current_model.joint_count);
    state.current_model.unified_robot_model_ref =
        ReadString(jsonObject, QStringLiteral("unified_robot_model_ref"), state.current_model.unified_robot_model_ref);
    state.current_model.model_source_mode = InferModelSourceMode(
        ReadString(jsonObject, QStringLiteral("model_source_mode")),
        state.current_model.urdf_source_path);
    state.current_model.backend_type =
        InferBackendType(ReadString(jsonObject, QStringLiteral("backend_type")));
    state.current_model.pinocchio_model_ready =
        ReadBool(jsonObject, QStringLiteral("pinocchio_model_ready"), state.current_model.pinocchio_model_ready);
    state.current_model.frame_semantics_version = std::max(
        1,
        ReadInt(jsonObject, QStringLiteral("frame_semantics_version"), state.current_model.frame_semantics_version));
    state.current_model.modeling_mode = InferModelingMode(
        explicitModelingMode,
        state.current_model.parameter_convention,
        state.current_model.urdf_source_path);
    state.current_model.base_frame =
        FromPoseObject(jsonObject.value(QStringLiteral("base_frame")).toObject());
    state.current_model.flange_frame =
        FromPoseObject(jsonObject.value(QStringLiteral("flange_frame")).toObject());
    state.current_model.tool_frame = jsonObject.value(QStringLiteral("tool_frame")).isObject()
        ? std::optional<RoboSDP::Kinematics::Dto::CartesianPoseDto>(
              FromPoseObject(jsonObject.value(QStringLiteral("tool_frame")).toObject()))
        : std::nullopt;
    state.current_model.workpiece_frame = jsonObject.value(QStringLiteral("workpiece_frame")).isObject()
        ? std::optional<RoboSDP::Kinematics::Dto::CartesianPoseDto>(
              FromPoseObject(jsonObject.value(QStringLiteral("workpiece_frame")).toObject()))
        : std::nullopt;

    state.current_model.links.clear();
    for (const QJsonValue& value : jsonObject.value(QStringLiteral("links")).toArray())
    {
        const QJsonObject linkObject = value.toObject();
        state.current_model.links.push_back({
            ReadString(linkObject, QStringLiteral("link_id")),
            ReadDouble(linkObject, QStringLiteral("a")),
            ReadDouble(linkObject, QStringLiteral("alpha")),
            ReadDouble(linkObject, QStringLiteral("d")),
            ReadDouble(linkObject, QStringLiteral("theta_offset"))});
    }

    state.current_model.joint_limits.clear();
    for (const QJsonValue& value : jsonObject.value(QStringLiteral("joint_limits")).toArray())
    {
        const QJsonObject limitObject = value.toObject();
        state.current_model.joint_limits.push_back({
            ReadString(limitObject, QStringLiteral("joint_id")),
            ReadArray2(limitObject.value(QStringLiteral("soft_limit")).toArray()),
            ReadArray2(limitObject.value(QStringLiteral("hard_limit")).toArray()),
            ReadDouble(limitObject, QStringLiteral("max_velocity"), 180.0),
            ReadDouble(limitObject, QStringLiteral("max_acceleration"), 360.0)});
    }

    const QJsonObject tcpObject = jsonObject.value(QStringLiteral("tcp_frame")).toObject();
    state.current_model.tcp_frame.translation_m =
        ReadArray3(tcpObject.value(QStringLiteral("translation_m")).toArray());
    state.current_model.tcp_frame.rpy_deg =
        ReadArray3(tcpObject.value(QStringLiteral("rpy_deg")).toArray());

    const QJsonObject ikConfigObject = jsonObject.value(QStringLiteral("ik_solver_config")).toObject();
    state.current_model.ik_solver_config.solver_type =
        ReadString(ikConfigObject, QStringLiteral("solver_type"), state.current_model.ik_solver_config.solver_type);
    state.current_model.ik_solver_config.branch_policy =
        ReadString(ikConfigObject, QStringLiteral("branch_policy"), state.current_model.ik_solver_config.branch_policy);
    state.current_model.ik_solver_config.max_iterations =
        ReadInt(ikConfigObject, QStringLiteral("max_iterations"), state.current_model.ik_solver_config.max_iterations);
    state.current_model.ik_solver_config.position_tolerance_mm =
        ReadDouble(
            ikConfigObject,
            QStringLiteral("position_tolerance_mm"),
            state.current_model.ik_solver_config.position_tolerance_mm);
    state.current_model.ik_solver_config.orientation_tolerance_deg =
        ReadDouble(
            ikConfigObject,
            QStringLiteral("orientation_tolerance_deg"),
            state.current_model.ik_solver_config.orientation_tolerance_deg);
    state.current_model.ik_solver_config.step_gain =
        ReadDouble(ikConfigObject, QStringLiteral("step_gain"), state.current_model.ik_solver_config.step_gain);

    state.current_model.joint_order_signature = ReadString(
        jsonObject,
        QStringLiteral("joint_order_signature"),
        BuildJointOrderSignature(state.current_model));
    if (state.current_model.joint_order_signature.trimmed().isEmpty())
    {
        // 某些老项目既没有 joint_order_signature，也可能把字段写成空串；
        // 这里再次兜底生成，避免后续共享内核把合法老项目误判为顺序签名缺失。
        state.current_model.joint_order_signature = BuildJointOrderSignature(state.current_model);
    }
    state.last_fk_result = FromFkResultObject(jsonObject.value(QStringLiteral("last_fk_result")).toObject());
    state.last_ik_result = FromIkResultObject(jsonObject.value(QStringLiteral("last_ik_result")).toObject());
    state.backend_summary = FromBackendSummaryObject(
        jsonObject.value(QStringLiteral("backend_summary")).toObject(),
        state.backend_summary,
        state.current_model.backend_type);
    return state;
}

QJsonObject KinematicJsonStorage::ToWorkspaceJsonObject(
    const RoboSDP::Kinematics::Dto::WorkspaceResultDto& result) const
{
    QJsonObject object;
    object.insert(QStringLiteral("success"), result.success);
    object.insert(QStringLiteral("message"), result.message);
    object.insert(QStringLiteral("requested_sample_count"), result.requested_sample_count);
    object.insert(QStringLiteral("reachable_sample_count"), result.reachable_sample_count);
    object.insert(QStringLiteral("max_radius_m"), result.max_radius_m);
    object.insert(QStringLiteral("min_position_m"), ToJsonArray3(result.min_position_m));
    object.insert(QStringLiteral("max_position_m"), ToJsonArray3(result.max_position_m));

    QJsonArray pointArray;
    for (const auto& point : result.sampled_points)
    {
        QJsonObject pointObject;
        pointObject.insert(QStringLiteral("joint_positions_deg"), ToVectorArray(point.joint_positions_deg));
        pointObject.insert(QStringLiteral("tcp_pose"), ToPoseObject(point.tcp_pose));
        pointArray.append(pointObject);
    }
    object.insert(QStringLiteral("sampled_points"), pointArray);
    return object;
}

RoboSDP::Kinematics::Dto::WorkspaceResultDto KinematicJsonStorage::FromWorkspaceJsonObject(
    const QJsonObject& jsonObject) const
{
    RoboSDP::Kinematics::Dto::WorkspaceResultDto result;
    result.success = ReadBool(jsonObject, QStringLiteral("success"));
    result.message = ReadString(jsonObject, QStringLiteral("message"));
    result.requested_sample_count = ReadInt(jsonObject, QStringLiteral("requested_sample_count"));
    result.reachable_sample_count = ReadInt(jsonObject, QStringLiteral("reachable_sample_count"));
    result.max_radius_m = ReadDouble(jsonObject, QStringLiteral("max_radius_m"));
    result.min_position_m = ReadArray3(jsonObject.value(QStringLiteral("min_position_m")).toArray());
    result.max_position_m = ReadArray3(jsonObject.value(QStringLiteral("max_position_m")).toArray());

    for (const QJsonValue& value : jsonObject.value(QStringLiteral("sampled_points")).toArray())
    {
        const QJsonObject pointObject = value.toObject();
        result.sampled_points.push_back({
            ReadVectorArray(pointObject.value(QStringLiteral("joint_positions_deg")).toArray()),
            FromPoseObject(pointObject.value(QStringLiteral("tcp_pose")).toObject())});
    }
    return result;
}

} // namespace RoboSDP::Kinematics::Persistence
