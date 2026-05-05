#pragma once

/**
 * @file KinematicsServiceInternal.h
 * @brief KinematicsService 各拆分编译单元共享的内部工具函数。
 *
 * 本文件中的函数原属于 KinematicsService.cpp 的匿名命名空间，
 * 拆分后通过 inline / namespace 机制暴露给多个 .cpp 编译单元共享。
 * 这些函数不属于 KinematicsService 的公有 API。
 */

#include "modules/kinematics/dto/KinematicModelDto.h"
#include "modules/kinematics/dto/KinematicSolverResultDto.h"
#include "modules/kinematics/dto/UnifiedRobotModelSnapshotDto.h"

#include <QDateTime>
#include <QFileInfo>
#include <QRegularExpression>
#include <QSet>
#include <QString>

#include <array>
#include <cmath>
#include <cstdlib>

namespace RoboSDP::Kinematics::Service::Internal
{

constexpr double kPi = 3.14159265358979323846;

/// 轻量级 4x4 齐次变换矩阵结构，不依赖 Eigen 或 Pinocchio。
struct Matrix4x4 {
    std::array<double, 16> data;
    Matrix4x4() {
        data.fill(0.0);
        data[0] = data[5] = data[10] = data[15] = 1.0;
    }
    Matrix4x4 operator*(const Matrix4x4& other) const {
        Matrix4x4 result;
        result.data.fill(0.0);
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                for (int k = 0; k < 4; ++k)
                    result.data[i * 4 + j] += data[i * 4 + k] * other.data[k * 4 + j];
        return result;
    }
    std::array<double, 3> GetTranslation() const { return {data[3], data[7], data[11]}; }
    std::array<double, 3> GetZAxis() const { return {data[2], data[6], data[10]}; }
};

inline Matrix4x4 ComputeDHMatrix(double a, double alpha_deg, double d, double theta_deg)
{
    Matrix4x4 mat;
    double alpha = alpha_deg * kPi / 180.0;
    double theta = theta_deg * kPi / 180.0;
    double ct = std::cos(theta), st = std::sin(theta);
    double ca = std::cos(alpha), sa = std::sin(alpha);
    mat.data[0] = ct;   mat.data[1] = -st * ca; mat.data[2] = st * sa;  mat.data[3] = a * ct;
    mat.data[4] = st;   mat.data[5] = ct * ca;  mat.data[6] = -ct * sa; mat.data[7] = a * st;
    mat.data[8] = 0.0;  mat.data[9] = sa;       mat.data[10] = ca;      mat.data[11] = d;
    mat.data[12] = 0.0; mat.data[13] = 0.0;     mat.data[14] = 0.0;     mat.data[15] = 1.0;
    return mat;
}

inline std::array<double, 3> MatrixToRPY(const Matrix4x4& m)
{
    double r31 = m.data[8], r32 = m.data[9], r33 = m.data[10];
    double r21 = m.data[4], r11 = m.data[0];
    double pitch = std::atan2(-r31, std::sqrt(r11 * r11 + r21 * r21));
    double roll, yaw;
    if (std::abs(std::cos(pitch)) > 1e-6) {
        roll = std::atan2(r32, r33);
        yaw = std::atan2(r21, r11);
    } else {
        roll = 0.0;
        yaw = std::atan2(-m.data[1], m.data[5]);
    }
    return {roll * 180.0 / kPi, pitch * 180.0 / kPi, yaw * 180.0 / kPi};
}

inline Matrix4x4 PoseToMatrix(const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    Matrix4x4 mat;
    double r = pose.rpy_deg[0] * kPi / 180.0;
    double p = pose.rpy_deg[1] * kPi / 180.0;
    double y = pose.rpy_deg[2] * kPi / 180.0;
    double cr = std::cos(r), sr = std::sin(r);
    double cp = std::cos(p), sp = std::sin(p);
    double cy = std::cos(y), sy = std::sin(y);
    mat.data[0] = cp * cy;  mat.data[1] = sr * sp * cy - cr * sy; mat.data[2] = cr * sp * cy + sr * sy;
    mat.data[4] = cp * sy;  mat.data[5] = sr * sp * sy + cr * cy; mat.data[6] = cr * sp * sy - sr * cy;
    mat.data[8] = -sp;      mat.data[9] = sr * cp;                mat.data[10] = cr * cp;
    mat.data[3] = pose.position_m[0];
    mat.data[7] = pose.position_m[1];
    mat.data[11] = pose.position_m[2];
    return mat;
}

inline double PreferredSoftLimit(double hardLimit, bool isMinimum)
{
    return hardLimit + (isMinimum ? 5.0 : -5.0);
}

struct ModelValidationResult
{
    bool success = true;
    QString message;
};

inline ModelValidationResult MakeValidationFailure(const QString& message)
{
    return {false, message};
}

inline bool IsFiniteValue(double value) { return std::isfinite(value); }

inline QString NormalizeArtifactNumber(double value, int decimals = 8)
{
    const double normalized = std::abs(value) < 1.0e-12 ? 0.0 : value;
    return QString::number(normalized, 'f', decimals);
}

inline QString FormatArtifactTriple(double x, double y, double z, int decimals = 8)
{
    return QStringLiteral("%1 %2 %3")
        .arg(NormalizeArtifactNumber(x, decimals))
        .arg(NormalizeArtifactNumber(y, decimals))
        .arg(NormalizeArtifactNumber(z, decimals));
}

inline void AppendWarningMessage(QString& target, const QString& warning)
{
    const QString normalizedWarning = warning.trimmed();
    if (normalizedWarning.isEmpty()) return;
    if (target.trimmed().isEmpty()) { target = normalizedWarning; return; }
    if (!target.contains(normalizedWarning))
        target.append(QStringLiteral("；")).append(normalizedWarning);
}

inline QString BuildPreviewJointOrderSignature(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    QString signature;
    for (std::size_t index = 0; index < model.joint_limits.size(); ++index)
    {
        if (index > 0) signature.append(QLatin1Char('|'));
        signature.append(model.joint_limits[index].joint_id);
    }
    return signature;
}

inline QString DefaultConversionDiagnostics(const QString& masterModelType)
{
    return masterModelType == QStringLiteral("urdf")
        ? QStringLiteral("当前预览由导入 URDF 驱动，DH/MDH 参数仅作为后续派生入口。")
        : QStringLiteral("当前草稿以 DH/MDH 参数为主模型，中央三维显示派生骨架预览。");
}

/// 将角度从度转为弧度。与 Pinocchio 无关的纯数学函数。
inline double PreviewDegToRad(double degrees)
{
    return degrees * kPi / 180.0;
}

// ---- 模型校验函数 ----

inline ModelValidationResult ValidatePoseFrame(
    const QString& frameName,
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    constexpr double kPositionLimitM = 10.0;
    constexpr double kAngleLimitDeg = 360.0;
    for (std::size_t index = 0; index < pose.position_m.size(); ++index)
    {
        if (!IsFiniteValue(pose.position_m[index]))
            return MakeValidationFailure(QStringLiteral("%1 的位置分量存在非数值内容。").arg(frameName));
        if (std::abs(pose.position_m[index]) > kPositionLimitM)
            return MakeValidationFailure(QStringLiteral("%1 的位置分量超出允许范围 ±%2 m。").arg(frameName).arg(kPositionLimitM, 0, 'f', 1));
    }
    for (std::size_t index = 0; index < pose.rpy_deg.size(); ++index)
    {
        if (!IsFiniteValue(pose.rpy_deg[index]))
            return MakeValidationFailure(QStringLiteral("%1 的姿态分量存在非数值内容。").arg(frameName));
        if (std::abs(pose.rpy_deg[index]) > kAngleLimitDeg)
            return MakeValidationFailure(QStringLiteral("%1 的姿态分量超出允许范围 ±%2 deg。").arg(frameName).arg(kAngleLimitDeg, 0, 'f', 0));
    }
    return {};
}

inline ModelValidationResult ValidateTcpFrame(const RoboSDP::Kinematics::Dto::TcpFrameDto& tcpFrame)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = tcpFrame.translation_m;
    pose.rpy_deg = tcpFrame.rpy_deg;
    return ValidatePoseFrame(QStringLiteral("TCP Frame"), pose);
}

inline ModelValidationResult ValidateJointLimits(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    if (model.links.empty())
        return MakeValidationFailure(QStringLiteral("links 为空，无法校验 joint_limits。"));
    if (model.joint_count != static_cast<int>(model.links.size()))
        return MakeValidationFailure(QStringLiteral("joint_count 必须与 links 数量一致。"));
    if (model.joint_limits.size() != model.links.size())
        return MakeValidationFailure(QStringLiteral("joint_limits 数量必须与 links 数量一致。"));

    QSet<QString> jointIds;
    for (std::size_t index = 0; index < model.joint_limits.size(); ++index)
    {
        const auto& limit = model.joint_limits[index];
        const QString displayIndex = QString::number(static_cast<int>(index) + 1);
        if (limit.joint_id.trimmed().isEmpty())
            return MakeValidationFailure(QStringLiteral("第 %1 个 joint_limits 条目缺少 joint_id。").arg(displayIndex));
        if (jointIds.contains(limit.joint_id))
            return MakeValidationFailure(QStringLiteral("joint_limits 中存在重复的 joint_id：%1。").arg(limit.joint_id));
        jointIds.insert(limit.joint_id);

        for (double value : limit.soft_limit)
            if (!IsFiniteValue(value)) return MakeValidationFailure(QStringLiteral("关节 %1 的 soft_limit 存在非数值内容。").arg(limit.joint_id));
        for (double value : limit.hard_limit)
            if (!IsFiniteValue(value)) return MakeValidationFailure(QStringLiteral("关节 %1 的 hard_limit 存在非数值内容。").arg(limit.joint_id));

        if (!IsFiniteValue(limit.max_velocity) || limit.max_velocity <= 0.0)
            return MakeValidationFailure(QStringLiteral("关节 %1 的 max_velocity 必须大于 0。").arg(limit.joint_id));
        if (!IsFiniteValue(limit.max_acceleration) || limit.max_acceleration <= 0.0)
            return MakeValidationFailure(QStringLiteral("关节 %1 的 max_acceleration 必须大于 0。").arg(limit.joint_id));

        if (limit.soft_limit[0] > limit.soft_limit[1])
            return MakeValidationFailure(QStringLiteral("关节 %1 的 soft_limit 最小值不能大于最大值。").arg(limit.joint_id));
        if (limit.hard_limit[0] > limit.hard_limit[1])
            return MakeValidationFailure(QStringLiteral("关节 %1 的 hard_limit 最小值不能大于最大值。").arg(limit.joint_id));
        if (limit.soft_limit[0] < limit.hard_limit[0] || limit.soft_limit[1] > limit.hard_limit[1])
            return MakeValidationFailure(QStringLiteral("关节 %1 的 soft_limit 必须落在 hard_limit 范围内。").arg(limit.joint_id));
    }
    return {};
}

inline ModelValidationResult ValidateModel(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    if (model.parameter_convention != QStringLiteral("DH") && model.parameter_convention != QStringLiteral("MDH"))
        return MakeValidationFailure(QStringLiteral("parameter_convention 仅支持 DH 或 MDH。"));

    auto validation = ValidatePoseFrame(QStringLiteral("Base Frame"), model.base_frame);
    if (!validation.success) return validation;
    validation = ValidatePoseFrame(QStringLiteral("Flange Frame"), model.flange_frame);
    if (!validation.success) return validation;

    if (model.tool_frame.has_value())
    {
        validation = ValidatePoseFrame(QStringLiteral("Tool Frame"), model.tool_frame.value());
        if (!validation.success) return validation;
    }
    if (model.workpiece_frame.has_value())
    {
        validation = ValidatePoseFrame(QStringLiteral("Workpiece Frame"), model.workpiece_frame.value());
        if (!validation.success) return validation;
    }
    validation = ValidateTcpFrame(model.tcp_frame);
    if (!validation.success) return validation;
    return ValidateJointLimits(model);
}

inline RoboSDP::Kinematics::Dto::UnifiedRobotModelSnapshotDto BuildUnifiedRobotSnapshot(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    RoboSDP::Kinematics::Dto::UnifiedRobotModelSnapshotDto snapshot;
    snapshot.snapshot_version = 3;
    snapshot.unified_robot_model_ref = model.unified_robot_model_ref;
    snapshot.source_kinematic_id = model.meta.kinematic_id;
    snapshot.master_model_type = model.master_model_type;
    snapshot.modeling_mode = model.modeling_mode;
    snapshot.parameter_convention = model.parameter_convention;
    snapshot.backend_type = model.backend_type;
    snapshot.joint_order_signature = model.joint_order_signature;
    snapshot.pinocchio_model_ready = model.pinocchio_model_ready;
    snapshot.frame_semantics_version = model.frame_semantics_version;
    snapshot.model_source_mode = model.model_source_mode;
    snapshot.conversion_diagnostics = model.conversion_diagnostics;

    const QString normalizedKinematicId = model.meta.kinematic_id.trimmed().isEmpty()
        ? QStringLiteral("default") : model.meta.kinematic_id.trimmed();
    const QString generatedAtUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);

    if (!model.urdf_source_path.trimmed().isEmpty())
    {
        const QFileInfo urdfFileInfo(model.urdf_source_path);
        snapshot.derived_artifact_relative_path.clear();
        snapshot.derived_artifact_version = urdfFileInfo.exists()
            ? urdfFileInfo.lastModified().toUTC().toString(Qt::ISODateWithMs)
            : QStringLiteral("external_missing");
        snapshot.derived_artifact_generated_at_utc = urdfFileInfo.exists()
            ? urdfFileInfo.lastModified().toUTC().toString(Qt::ISODateWithMs) : QString();
        snapshot.derived_artifact_state_code = urdfFileInfo.exists()
            ? QStringLiteral("external_master") : QStringLiteral("external_missing");
        snapshot.derived_artifact_exists = urdfFileInfo.exists();
        snapshot.derived_artifact_fresh = urdfFileInfo.exists() && snapshot.pinocchio_model_ready;
        return snapshot;
    }

    snapshot.derived_artifact_relative_path =
        QStringLiteral("kinematics/derived/%1.urdf").arg(normalizedKinematicId);
    snapshot.derived_artifact_version = QStringLiteral("logical_v%1").arg(snapshot.snapshot_version);
    snapshot.derived_artifact_generated_at_utc = generatedAtUtc;
    snapshot.derived_artifact_state_code = snapshot.pinocchio_model_ready
        ? QStringLiteral("logical_only") : QStringLiteral("logical_not_ready");
    snapshot.derived_artifact_exists = false;
    snapshot.derived_artifact_fresh = false;
    return snapshot;
}

inline QString BuildDhUnifiedChainDiagnostics(
    const RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto& backendContext)
{
    if (backendContext.IsSuccess() && backendContext.status.shared_robot_kernel_ready)
        return QStringLiteral("当前草稿以 DH/MDH 参数为主模型，统一工程主链已同步到共享 Pinocchio 内核。");
    const QString reason = backendContext.status.status_message.trimmed();
    if (!reason.isEmpty())
        return QStringLiteral("当前草稿以 DH/MDH 参数为主模型，但统一工程主链尚未就绪：%1").arg(reason);
    return QStringLiteral("当前草稿以 DH/MDH 参数为主模型，但统一工程主链尚未就绪。");
}

/// @brief 生成最小 link XML 片段。
inline QString BuildUrdfLinkBlock(const QString& linkName)
{
    return QStringLiteral("  <link name=\"%1\"/>\n").arg(linkName);
}

/// @brief 生成 origin 属性的 xyz/rpy XML 片段。
inline QString BuildUrdfOriginAttributes(const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    return QStringLiteral("xyz=\"%1\" rpy=\"%2\"")
        .arg(FormatArtifactTriple(pose.position_m[0], pose.position_m[1], pose.position_m[2]))
        .arg(FormatArtifactTriple(
            PreviewDegToRad(pose.rpy_deg[0]),
            PreviewDegToRad(pose.rpy_deg[1]),
            PreviewDegToRad(pose.rpy_deg[2])));
}

// ============================================================================
// URDF 草案提取相关数据结构与工具函数（无 Pinocchio 依赖）
// ============================================================================

struct UrdfDraftJointInput
{
    QString joint_id, joint_type, parent_link_name, child_link_name;
    std::array<double, 3> xyz{0,0,0}, rpy_deg{0,0,0}, axis_xyz{1,0,0};
    bool has_limit = false;
    double lower_deg = -180.0, upper_deg = 180.0, velocity_deg = 180.0;
};

struct UrdfDraftLinkInput
{
    QString link_id;
    int parent_joint_index = -1;
    std::vector<int> child_joint_indices;
};

struct UrdfDraftModelInput
{
    QString robot_name;
    std::vector<UrdfDraftLinkInput> links;
    std::vector<UrdfDraftJointInput> joints;
    QStringList root_link_names;
    QString warning_message;
};

struct UrdfDraftTrunkInput
{
    std::vector<int> ordered_joint_indices;
    int movable_joint_count = 0;
    QString warning_message;
};

inline QString NormalizeLowerToken(const QString& value)
{
    return value.trimmed().toLower();
}

inline std::array<double, 3> ParseUrdfDraftTriple(const QString& value, const QString& fieldName)
{
    std::array<double, 3> result{0,0,0};
    const QStringList tokens = value.trimmed().split(QRegularExpression(QStringLiteral("[\\s,]+")),
        Qt::SkipEmptyParts);
    for (int i = 0; i < 3 && i < tokens.size(); ++i)
    {
        bool ok = false;
        double v = tokens[i].toDouble(&ok);
        result[i] = ok ? v : 0.0;
    }
    return result;
}

inline bool IsNearZero(double value, double epsilon = 1.0e-6)
{
    return std::abs(value) <= epsilon;
}

inline bool IsAxisNearPositiveZ(const std::array<double, 3>& axis)
{
    return IsNearZero(axis[0], 1.0e-3) && IsNearZero(axis[1], 1.0e-3) && std::abs(axis[2] - 1.0) <= 1.0e-3;
}

inline RoboSDP::Kinematics::Dto::CartesianPoseDto BuildDraftPose(
    const std::array<double, 3>& xyz,
    const std::array<double, 3>& rpyDeg)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = xyz;
    pose.rpy_deg = rpyDeg;
    return pose;
}

inline const UrdfDraftJointInput* FindJointByName(const UrdfDraftModelInput& model, const QString& jointName)
{
    const QString normalizedJointName = jointName.trimmed();
    for (const auto& joint : model.joints)
    {
        if (joint.joint_id == normalizedJointName)
            return &joint;
    }
    return nullptr;
}

inline QString BuildDraftJointSignature(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    QString signature;
    for (std::size_t index = 0; index < model.joint_limits.size(); ++index)
    {
        if (index > 0) signature.append(QLatin1Char('|'));
        signature.append(model.joint_limits[index].joint_id.trimmed());
    }
    return signature;
}

} // namespace RoboSDP::Kinematics::Service::Internal
