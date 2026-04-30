#include "modules/kinematics/service/KinematicsService.h"

#include "core/kinematics/SharedRobotKernelRegistry.h"
#include "modules/kinematics/adapter/PinocchioIkSolverAdapter.h"
#include "modules/kinematics/adapter/PinocchioKinematicBackendAdapter.h"
#include "modules/topology/dto/TopologyRecommendationDto.h"

#include <QDir>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QSaveFile>
#include <QSet>
#include <QStringList>
#include <QTextStream>
#include <QXmlStreamReader>

#include <cmath>

// 条件编译：仅在 CMake 中探测到并开启了 Pinocchio 库时，引入其头文件。
// 这是为了保证在没有安装复杂物理引擎的精简环境下依然能够编译。
#if defined(ROBOSDP_HAVE_PINOCCHIO)
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#endif

namespace RoboSDP::Kinematics::Service
{

// 匿名命名空间，用于限制内部辅助函数的可见性，避免符号污染其他编译单元。
// 这些函数只在当前文件内部生效。
namespace
{
    constexpr double kPi = 3.14159265358979323846;
// --- 新增：轻量级 4x4 齐次变换矩阵结构与 D-H 算法 ---
    // 保留在匿名空间，确保不污染外部，且能在无 Eigen 依赖下快速运行
    struct Matrix4x4 {
        std::array<double, 16> data;
        Matrix4x4() {
            data.fill(0.0);
            data[0] = data[5] = data[10] = data[15] = 1.0;
        }
        Matrix4x4 operator*(const Matrix4x4& other) const {
            Matrix4x4 result;
            result.data.fill(0.0);
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    for (int k = 0; k < 4; ++k) {
                        result.data[i * 4 + j] += data[i * 4 + k] * other.data[k * 4 + j];
                    }
                }
            }
            return result;
        }
        std::array<double, 3> GetTranslation() const { return {data[3], data[7], data[11]}; }
        std::array<double, 3> GetZAxis() const { return {data[2], data[6], data[10]}; }
    };

    Matrix4x4 ComputeDHMatrix(double a, double alpha_deg, double d, double theta_deg) {
        Matrix4x4 mat;
        double alpha = alpha_deg * 3.14159265358979323846 / 180.0;
        double theta = theta_deg * 3.14159265358979323846 / 180.0;
        double ct = std::cos(theta), st = std::sin(theta);
        double ca = std::cos(alpha), sa = std::sin(alpha);
        mat.data[0] = ct;   mat.data[1] = -st * ca; mat.data[2] = st * sa;  mat.data[3] = a * ct;
        mat.data[4] = st;   mat.data[5] = ct * ca;  mat.data[6] = -ct * sa; mat.data[7] = a * st;
        mat.data[8] = 0.0;  mat.data[9] = sa;       mat.data[10] = ca;      mat.data[11] = d;
        mat.data[12] = 0.0; mat.data[13] = 0.0;     mat.data[14] = 0.0;     mat.data[15] = 1.0;
        return mat;
    }
    /**
     * @brief 从旋转矩阵提取 RPY 欧拉角 (单位: 度)
     * 采用固定轴 XYZ 约定 (Roll-Pitch-Yaw)，等价于内部轴 ZYX。
     */
    std::array<double, 3> MatrixToRPY(const Matrix4x4& m) {
        double r11 = m.data[0], r21 = m.data[4], r31 = m.data[8];
        double r32 = m.data[9], r33 = m.data[13]; // 修正：data[13] 是矩阵第4行第2列？不，data[10]是r33

        // 严格按照 Matrix4x4 data[16] 的行主序索引：
        // 0:r11, 1:r12, 2:r13, 3:tx
        // 4:r21, 5:r22, 6:r23, 7:ty
        // 8:r31, 9:r32, 10:r33, 11:tz
        r31 = m.data[8]; r32 = m.data[9]; r33 = m.data[10];
        r21 = m.data[4]; r11 = m.data[0];

        double pitch = std::atan2(-r31, std::sqrt(r11 * r11 + r21 * r21));
        double roll, yaw;

        if (std::abs(std::cos(pitch)) > 1e-6) {
            roll = std::atan2(r32, r33);
            yaw = std::atan2(r21, r11);
        } else {
            // 奇异位姿处理 (Gimbal Lock)
            roll = 0.0;
            yaw = std::atan2(-m.data[1], m.data[5]); // 使用 r12 和 r22
        }

        return {roll * 180.0 / kPi, pitch * 180.0 / kPi, yaw * 180.0 / kPi};
    }
// 将 CartesianPoseDto (平移 + RPY) 转换为齐次变换矩阵
    Matrix4x4 PoseToMatrix(const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose) {
        Matrix4x4 mat;
        double r = pose.rpy_deg[0] * kPi / 180.0;
        double p = pose.rpy_deg[1] * kPi / 180.0;
        double y = pose.rpy_deg[2] * kPi / 180.0;

        double cr = std::cos(r), sr = std::sin(r);
        double cp = std::cos(p), sp = std::sin(p);
        double cy = std::cos(y), sy = std::sin(y);

        // 旋转部分 (RPY 顺序: 固定轴 XYZ)
        mat.data[0] = cp * cy;
        mat.data[1] = sr * sp * cy - cr * sy;
        mat.data[2] = cr * sp * cy + sr * sy;
        mat.data[4] = cp * sy;
        mat.data[5] = sr * sp * sy + cr * cy;
        mat.data[6] = cr * sp * sy - sr * cy;
        mat.data[8] = -sp;
        mat.data[9] = sr * cp;
        mat.data[10] = cr * cp;

        // 平移部分
        mat.data[3] = pose.position_m[0];
        mat.data[7] = pose.position_m[1];
        mat.data[11] = pose.position_m[2];

        return mat;
    }

/**
 * @brief 根据硬限位计算推荐的软限位值
 * @param hardLimit 物理硬限位值（角度或位置）
 * @param isMinimum true 表示当前计算的是下限（最小值），false 表示上限（最大值）
 * @return 软限位值
 * @details 设计意图：软限位是控制系统软件层面的保护边界，必须比机械硬限位更保守。
 * 这里采用向内收缩 5 度的方式，为电机减速停机留出安全制动余量。
 */
double PreferredSoftLimit(double hardLimit, bool isMinimum)
{
    // 如果是最小值，向正方向收缩 +5；如果是最大值，向负方向收缩 -5
    return hardLimit + (isMinimum ? 5.0 : -5.0);
}

/**
 * @brief 模型参数合法性校验结果结构体，用于在验证流程中传递状态和错误信息
 */
struct ModelValidationResult
{
    bool success = true; // 默认校验通过
    QString message;     // 失败时的错误提示
};

/**
 * @brief 构造一个表示“校验失败”的结果对象的快捷工厂函数
 * @param message 错误信息描述
 * @return 初始化为失败状态的 ModelValidationResult
 */
ModelValidationResult MakeValidationFailure(const QString& message)
{
    return {false, message};
}

/**
 * @brief 检查浮点数是否为安全的有限数值
 * @param value 待检查的浮点数
 * @return 如果是有效数字返回 true，如果是 NaN (非数字) 或 Inf (无穷大) 返回 false
 * @details 在将数据传递给底层矩阵运算前，防止这些非法值导致程序崩溃
 */
bool IsFiniteValue(double value)
{
    return std::isfinite(value);
}

QString NormalizeArtifactNumber(double value, int decimals = 8)
{
    const double normalized = std::abs(value) < 1.0e-12 ? 0.0 : value;
    return QString::number(normalized, 'f', decimals);
}

QString FormatArtifactTriple(double x, double y, double z, int decimals = 8)
{
    return QStringLiteral("%1 %2 %3")
        .arg(NormalizeArtifactNumber(x, decimals))
        .arg(NormalizeArtifactNumber(y, decimals))
        .arg(NormalizeArtifactNumber(z, decimals));
}

void AppendWarningMessage(QString& target, const QString& warning)
{
    const QString normalizedWarning = warning.trimmed();
    if (normalizedWarning.isEmpty())
    {
        return;
    }

    if (target.trimmed().isEmpty())
    {
        target = normalizedWarning;
        return;
    }

    if (!target.contains(normalizedWarning))
    {
        target.append(QStringLiteral("；")).append(normalizedWarning);
    }
}

/**
 * @brief 为预览链路构造稳定的关节顺序签名，避免旧草稿遗漏该字段时共享内核无法正确匹配关节顺序。
 */
QString BuildPreviewJointOrderSignature(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
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
    return signature;
}

/**
 * @brief 将主模型类型翻译为预览链路使用的固定中文摘要，避免 UI 与日志到处散落硬编码字面量。
 */
QString DefaultConversionDiagnostics(const QString& masterModelType)
{
    return masterModelType == QStringLiteral("urdf")
        ? QStringLiteral("当前预览由导入 URDF 驱动，DH/MDH 参数仅作为后续派生入口。")
        : QStringLiteral("当前草稿以 DH/MDH 参数为主模型，中央三维显示派生骨架预览。");
}

/**
 * @brief 将 build-context 诊断结果翻译成 DH 主模型阶段可读的同步摘要。
 * @details
 * 第二阶段先不真正派生高保真 URDF 文件，但会把“统一工程主链是否已经进入共享 Pinocchio 内核”
 * 这件事清晰写回模型状态，避免后续 Dynamics / Planning 还要重新猜当前主链是否 ready。
 */
QString BuildDhUnifiedChainDiagnostics(
    const RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto& backendContext)
{
    if (backendContext.IsSuccess() && backendContext.status.shared_robot_kernel_ready)
    {
        return QStringLiteral("当前草稿以 DH/MDH 参数为主模型，统一工程主链已同步到共享 Pinocchio 内核。");
    }

    const QString reason = backendContext.status.status_message.trimmed();
    if (!reason.isEmpty())
    {
        return QStringLiteral("当前草稿以 DH/MDH 参数为主模型，但统一工程主链尚未就绪：%1").arg(reason);
    }

    return QStringLiteral("当前草稿以 DH/MDH 参数为主模型，但统一工程主链尚未就绪。");
}

/**
 * @brief 根据当前 KinematicModel 构造一份统一工程主链快照摘要。
 * @details
 * 第二阶段先不导出真正的派生 URDF 文件，而是先沉淀一份可持久化、可跨模块透传的
 * 统一主链摘要对象，作为 Dynamics / Planning 判断上游主链状态的稳定入口。
 */
RoboSDP::Kinematics::Dto::UnifiedRobotModelSnapshotDto BuildUnifiedRobotSnapshot(
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
        ? QStringLiteral("default")
        : model.meta.kinematic_id.trimmed();
    const QString generatedAtUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);

    if (!model.urdf_source_path.trimmed().isEmpty())
    {
        const QFileInfo urdfFileInfo(model.urdf_source_path);
        snapshot.derived_artifact_relative_path.clear();
        snapshot.derived_artifact_version = urdfFileInfo.exists()
            ? urdfFileInfo.lastModified().toUTC().toString(Qt::ISODateWithMs)
            : QStringLiteral("external_missing");
        snapshot.derived_artifact_generated_at_utc = urdfFileInfo.exists()
            ? urdfFileInfo.lastModified().toUTC().toString(Qt::ISODateWithMs)
            : QString();
        snapshot.derived_artifact_state_code = urdfFileInfo.exists()
            ? QStringLiteral("external_master")
            : QStringLiteral("external_missing");
        snapshot.derived_artifact_exists = urdfFileInfo.exists();
        snapshot.derived_artifact_fresh = urdfFileInfo.exists() && snapshot.pinocchio_model_ready;
        return snapshot;
    }

    snapshot.derived_artifact_relative_path =
        QStringLiteral("kinematics/derived/%1.urdf").arg(normalizedKinematicId);
    snapshot.derived_artifact_version = QStringLiteral("logical_v%1").arg(snapshot.snapshot_version);
    snapshot.derived_artifact_generated_at_utc = generatedAtUtc;
    snapshot.derived_artifact_state_code = snapshot.pinocchio_model_ready
        ? QStringLiteral("logical_only")
        : QStringLiteral("logical_not_ready");
    snapshot.derived_artifact_exists = false;
    snapshot.derived_artifact_fresh = false;
    return snapshot;
}

/**
 * @brief 校验单个笛卡尔坐标系位姿数据（位置和姿态）的合法性
 * @param frameName 坐标系名称，用于在报错信息中提示用户哪个坐标系出错
 * @param pose 待检查的位姿数据 DTO
 * @return 校验结果
 * @details 包含安全边界检查：
 * 1. 确保所有分量都是合法数字。
 * 2. 位置限制在 ±10m 内（基于常规工业机器人的合理物理尺寸）。
 * 3. 欧拉角限制在 ±360° 内。
 */
ModelValidationResult ValidatePoseFrame(
    const QString& frameName,
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    constexpr double kPositionLimitM = 10.0;  // 位置阈值 10米
    constexpr double kAngleLimitDeg = 360.0;  // 角度阈值 360度

    // 1. 校验 XYZ 平移分量
    for (std::size_t index = 0; index < pose.position_m.size(); ++index)
    {
        if (!IsFiniteValue(pose.position_m[index]))
        {
            return MakeValidationFailure(QStringLiteral("%1 的位置分量存在非数值内容。").arg(frameName));
        }
        if (std::abs(pose.position_m[index]) > kPositionLimitM)
        {
            return MakeValidationFailure(QStringLiteral("%1 的位置分量超出允许范围 ±%2 m。")
                                             .arg(frameName)
                                             .arg(kPositionLimitM, 0, 'f', 1));
        }
    }

    // 2. 校验 RPY (Roll, Pitch, Yaw) 旋转分量
    for (std::size_t index = 0; index < pose.rpy_deg.size(); ++index)
    {
        if (!IsFiniteValue(pose.rpy_deg[index]))
        {
            return MakeValidationFailure(QStringLiteral("%1 的姿态分量存在非数值内容。").arg(frameName));
        }
        if (std::abs(pose.rpy_deg[index]) > kAngleLimitDeg)
        {
            return MakeValidationFailure(QStringLiteral("%1 的姿态分量超出允许范围 ±%2 deg。")
                                             .arg(frameName)
                                             .arg(kAngleLimitDeg, 0, 'f', 0));
        }
    }

    return {}; // 返回默认成功状态
}

/**
 * @brief 校验工具中心点 (TCP) 坐标系的合法性
 * @param tcpFrame 待校验的 TCP 结构
 * @return 校验结果
 * @note 由于 TCP 的 DTO 结构定义不同（平移属性名为 translation_m），
 * 我们在此将其转换为标准的 CartesianPoseDto，以复用 ValidatePoseFrame 的校验逻辑。
 */
ModelValidationResult ValidateTcpFrame(const RoboSDP::Kinematics::Dto::TcpFrameDto& tcpFrame)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = tcpFrame.translation_m;
    pose.rpy_deg = tcpFrame.rpy_deg;
    return ValidatePoseFrame(QStringLiteral("TCP Frame"), pose);
}

/**
 * @brief 校验模型中所有关节的运动限位与物理约束配置
 * @param model 待校验的运动学模型
 * @return 校验结果
 * @details 
 * 该函数执行一系列严格的逻辑一致性检查，防呆防错：
 * 1. 数量对齐检查（限位数组大小 == 连杆数量 == 关节数）。
 * 2. 标识符排重。
 * 3. 动力学约束（速度、加速度必须 > 0）。
 * 4. 边界合法性（软硬限位中 Min 必须 <= Max）。
 * 5. 安全包裹（软限位区间必须严格在硬限位区间内部）。
 */
ModelValidationResult ValidateJointLimits(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    if (model.links.empty())
    {
        return MakeValidationFailure(QStringLiteral("links 为空，无法校验 joint_limits。"));
    }

    if (model.joint_count != static_cast<int>(model.links.size()))
    {
        return MakeValidationFailure(QStringLiteral("joint_count 必须与 links 数量一致。"));
    }

    if (model.joint_limits.size() != model.links.size())
    {
        return MakeValidationFailure(QStringLiteral("joint_limits 数量必须与 links 数量一致。"));
    }

    QSet<QString> jointIds; // 借助 QSet 用于查重 joint_id
    for (std::size_t index = 0; index < model.joint_limits.size(); ++index)
    {
        const auto& limit = model.joint_limits[index];
        const QString displayIndex = QString::number(static_cast<int>(index) + 1);

        // 检查 ID 是否为空
        if (limit.joint_id.trimmed().isEmpty())
        {
            return MakeValidationFailure(QStringLiteral("第 %1 个 joint_limits 条目缺少 joint_id。").arg(displayIndex));
        }

        // 检查 ID 是否重复
        if (jointIds.contains(limit.joint_id))
        {
            return MakeValidationFailure(QStringLiteral("joint_limits 中存在重复的 joint_id：%1。").arg(limit.joint_id));
        }
        jointIds.insert(limit.joint_id);

        // 检查限制数组中的值是否是有效数字
        for (double value : limit.soft_limit)
        {
            if (!IsFiniteValue(value)) return MakeValidationFailure(QStringLiteral("关节 %1 的 soft_limit 存在非数值内容。").arg(limit.joint_id));
        }
        for (double value : limit.hard_limit)
        {
            if (!IsFiniteValue(value)) return MakeValidationFailure(QStringLiteral("关节 %1 的 hard_limit 存在非数值内容。").arg(limit.joint_id));
        }

        // 动力学约束检查：速度、加速度不能为负数或0
        if (!IsFiniteValue(limit.max_velocity) || limit.max_velocity <= 0.0)
        {
            return MakeValidationFailure(QStringLiteral("关节 %1 的 max_velocity 必须大于 0。").arg(limit.joint_id));
        }
        if (!IsFiniteValue(limit.max_acceleration) || limit.max_acceleration <= 0.0)
        {
            return MakeValidationFailure(QStringLiteral("关节 %1 的 max_acceleration 必须大于 0。").arg(limit.joint_id));
        }

        // 范围合法性检查：区间的最小值 (index 0) 必须 <= 最大值 (index 1)
        if (limit.soft_limit[0] > limit.soft_limit[1])
        {
            return MakeValidationFailure(QStringLiteral("关节 %1 的 soft_limit 最小值不能大于最大值。").arg(limit.joint_id));
        }
        if (limit.hard_limit[0] > limit.hard_limit[1])
        {
            return MakeValidationFailure(QStringLiteral("关节 %1 的 hard_limit 最小值不能大于最大值。").arg(limit.joint_id));
        }

        // 包含关系检查：软限位区间必须是硬限位区间的子集
        if (limit.soft_limit[0] < limit.hard_limit[0] || limit.soft_limit[1] > limit.hard_limit[1])
        {
            return MakeValidationFailure(QStringLiteral("关节 %1 的 soft_limit 必须落在 hard_limit 范围内。").arg(limit.joint_id));
        }
    }

    return {};
}

/**
 * @brief 对整个运动学模型执行完整深度校验的总入口
 * @param model 待校验的运动学模型
 * @return 校验结果
 * @details 按照坐标系顺序，依次校验基础系、法兰系、工具系、工件系、TCP，以及所有关节限制。
 * 只有通过此校验的模型才能被送入底层的算法求解器 (FK/IK) 或保存至磁盘。
 */
ModelValidationResult ValidateModel(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    // 校验参数约定类型
    if (model.parameter_convention != QStringLiteral("DH") &&
        model.parameter_convention != QStringLiteral("MDH"))
    {
        return MakeValidationFailure(QStringLiteral("parameter_convention 仅支持 DH 或 MDH。"));
    }

    // 级联校验所有的关键坐标系
    auto validation = ValidatePoseFrame(QStringLiteral("Base Frame"), model.base_frame);
    if (!validation.success) return validation;

    validation = ValidatePoseFrame(QStringLiteral("Flange Frame"), model.flange_frame);
    if (!validation.success) return validation;

    // 工具和工件系是可选的，只有配置了才去校验
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

    // 最后校验关节限制的业务逻辑
    return ValidateJointLimits(model);
}

#if defined(ROBOSDP_HAVE_PINOCCHIO)
/**
 * @brief 将 Pinocchio 的三维平移向量转换为预览 DTO 使用的标准数组。
 */
std::array<double, 3> ToPreviewPosition(const Eigen::Vector3d& translation)
{
    return {translation[0], translation[1], translation[2]};
}

/**
 * @brief 将业务层角度单位 deg 转换为 Pinocchio 使用的 rad。
 */
double PreviewDegToRad(double degrees)
{
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    return degrees * kDegToRad;
}

/**
 * @brief 将旋转矩阵安全转换为预览链路使用的 RPY 欧拉角（单位：度）。
 * @details
 * 这里沿用项目中 pose -> SE3 的 yaw * pitch * roll 约定，
 * 因此反解时按 ZYX 顺序提取欧拉角，再回填为 roll/pitch/yaw。
 */
std::array<double, 3> ToPreviewRpyDeg(const Eigen::Matrix3d& rotation)
{
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    const Eigen::Vector3d yawPitchRoll = rotation.eulerAngles(2, 1, 0);
    return {
        yawPitchRoll[2] * kRadToDeg,
        yawPitchRoll[1] * kRadToDeg,
        yawPitchRoll[0] * kRadToDeg};
}

/**
 * @brief 将 Pinocchio 的 frame 位姿转换为预览节点使用的轻量全局位姿 DTO。
 * @details 该 DTO 后续会被 VTK Mesh 渲染链路复用，保证“骨架”和“皮肤”读取同一份零位事实。
 */
RoboSDP::Kinematics::Dto::CartesianPoseDto ToPreviewPose(const pinocchio::SE3& placement)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = ToPreviewPosition(placement.translation());
    pose.rpy_deg = ToPreviewRpyDeg(placement.rotation());
    return pose;
}

/**
 * @brief 将通用 CartesianPose DTO 转换为 Pinocchio SE3，供派生 URDF 文件构造固定关节 origin 使用。
 */
pinocchio::SE3 BuildArtifactPoseSe3(const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    const Eigen::AngleAxisd roll(PreviewDegToRad(pose.rpy_deg[0]), Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(PreviewDegToRad(pose.rpy_deg[1]), Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(PreviewDegToRad(pose.rpy_deg[2]), Eigen::Vector3d::UnitZ());
    const Eigen::Matrix3d rotation = (yaw * pitch * roll).toRotationMatrix();
    const Eigen::Vector3d translation(pose.position_m[0], pose.position_m[1], pose.position_m[2]);
    return pinocchio::SE3(rotation, translation);
}

/**
 * @brief 将 TCP DTO 转换为 Pinocchio SE3，语义上表示 flange -> tcp 的固定安装关系。
 */
pinocchio::SE3 BuildArtifactTcpSe3(const RoboSDP::Kinematics::Dto::TcpFrameDto& tcpFrame)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = tcpFrame.translation_m;
    pose.rpy_deg = tcpFrame.rpy_deg;
    return BuildArtifactPoseSe3(pose);
}

/**
 * @brief 将 Pinocchio SE3 重新转换为通用位姿 DTO，供派生 URDF origin 字段格式化复用。
 */
RoboSDP::Kinematics::Dto::CartesianPoseDto BuildArtifactPoseFromSe3(const pinocchio::SE3& transform)
{
    return ToPreviewPose(transform);
}

/**
 * @brief 标准 DH 派生 URDF 的关节后固定段：Rz(theta_offset) * Tz(d) * Tx(a) * Rx(alpha)。
 * @details 变量关节角 q 由前一段 revolute joint 提供，因此该固定段中显式保留 theta_offset。
 */
pinocchio::SE3 BuildStandardDhArtifactPostPlacement(
    const RoboSDP::Kinematics::Dto::KinematicLinkParameterDto& link)
{
    const double theta = PreviewDegToRad(link.theta_offset);
    const double alpha = PreviewDegToRad(link.alpha);
    const Eigen::Matrix3d rotation =
        (Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const Eigen::Vector3d translation(link.a * std::cos(theta), link.a * std::sin(theta), link.d);
    return pinocchio::SE3(rotation, translation);
}

/**
 * @brief 改进 DH 派生 URDF 的关节前固定段：Tx(a) * Rx(alpha)。
 */
pinocchio::SE3 BuildModifiedDhArtifactPrePlacement(
    const RoboSDP::Kinematics::Dto::KinematicLinkParameterDto& link)
{
    const double alpha = PreviewDegToRad(link.alpha);
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Vector3d translation(link.a, 0.0, 0.0);
    return pinocchio::SE3(rotation, translation);
}

/**
 * @brief 改进 DH 派生 URDF 的关节后固定段：Rz(theta_offset) * Tz(d)。
 */
pinocchio::SE3 BuildModifiedDhArtifactPostPlacement(
    const RoboSDP::Kinematics::Dto::KinematicLinkParameterDto& link)
{
    const double theta = PreviewDegToRad(link.theta_offset);
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d translation(0.0, 0.0, link.d);
    return pinocchio::SE3(rotation, translation);
}

/**
 * @brief 将固定安装位姿格式化为 URDF origin 属性字符串。
 */
QString BuildUrdfOriginAttributes(const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    return QStringLiteral("xyz=\"%1\" rpy=\"%2\"")
        .arg(FormatArtifactTriple(pose.position_m[0], pose.position_m[1], pose.position_m[2]))
        .arg(FormatArtifactTriple(
            PreviewDegToRad(pose.rpy_deg[0]),
            PreviewDegToRad(pose.rpy_deg[1]),
            PreviewDegToRad(pose.rpy_deg[2])));
}

/**
 * @brief 生成一个最小 link 片段；当前阶段仅承载运动学主链，不写入虚假的质量或 Mesh 数据。
 */
QString BuildUrdfLinkBlock(const QString& linkName)
{
    return QStringLiteral("  <link name=\"%1\"/>\n").arg(linkName);
}

RoboSDP::Kinematics::Dto::KinematicModelDto BuildUrdfPreviewModel(const QString& absoluteUrdfPath);

/// @brief URDF -> DH 草案提取使用的最小 joint 语义输入。
struct UrdfDraftJointInput
{
    QString joint_id;
    QString joint_type;
    QString parent_link_name;
    QString child_link_name;
    std::array<double, 3> xyz {0.0, 0.0, 0.0};
    std::array<double, 3> rpy_deg {0.0, 0.0, 0.0};
    std::array<double, 3> axis_xyz {1.0, 0.0, 0.0};
    bool has_limit = false;
    double lower_deg = -180.0;
    double upper_deg = 180.0;
    double velocity_deg = 180.0;
};

/// @brief URDF -> DH 草案提取使用的最小 link 拓扑输入。
struct UrdfDraftLinkInput
{
    QString link_id;
    int parent_joint_index = -1;
    std::vector<int> child_joint_indices;
};

/// @brief 最小 URDF 语义模型，只保留 DH 草案提取所需信息。
struct UrdfDraftModelInput
{
    QString robot_name;
    std::vector<UrdfDraftLinkInput> links;
    std::vector<UrdfDraftJointInput> joints;
    QStringList root_link_names;
    QString warning_message;
};

/// @brief 从复杂 URDF 树中挑出的主干结果；只用于草案展示，不作为主模型切换依据。
struct UrdfDraftTrunkInput
{
    std::vector<int> ordered_joint_indices;
    int movable_joint_count = 0;
    QString warning_message;
};

QString NormalizeLowerToken(const QString& value)
{
    return value.trimmed().toLower();
}

std::array<double, 3> ParseUrdfDraftTriple(const QString& value, const QString& fieldName)
{
    std::array<double, 3> result {0.0, 0.0, 0.0};
    if (value.trimmed().isEmpty())
    {
        return result;
    }

    const QStringList tokens = value.split(QLatin1Char(' '), Qt::SkipEmptyParts);
    if (tokens.size() != 3)
    {
        throw std::runtime_error(
            QStringLiteral("URDF 字段 %1 必须包含 3 个数值。").arg(fieldName).toUtf8().constData());
    }

    for (int index = 0; index < 3; ++index)
    {
        bool ok = false;
        result[static_cast<std::size_t>(index)] = tokens[index].toDouble(&ok);
        if (!ok || !IsFiniteValue(result[static_cast<std::size_t>(index)]))
        {
            throw std::runtime_error(
                QStringLiteral("URDF 字段 %1 的第 %2 个分量不是有效数值。")
                    .arg(fieldName)
                    .arg(index + 1)
                    .toUtf8()
                    .constData());
        }
    }
    return result;
}

bool IsNearZero(double value, double epsilon = 1.0e-6)
{
    return std::abs(value) <= epsilon;
}

bool IsAxisNearPositiveZ(const std::array<double, 3>& axis)
{
    return IsNearZero(axis[0], 1.0e-3) && IsNearZero(axis[1], 1.0e-3) && std::abs(axis[2] - 1.0) <= 1.0e-3;
}

RoboSDP::Kinematics::Dto::CartesianPoseDto BuildDraftPose(
    const std::array<double, 3>& xyz,
    const std::array<double, 3>& rpyDeg)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = xyz;
    pose.rpy_deg = rpyDeg;
    return pose;
}

UrdfDraftModelInput ReadUrdfDraftModel(const QString& urdfPath)
{
    QFile file(urdfPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        throw std::runtime_error(
            QStringLiteral("URDF 解析失败：无法打开文件 %1。").arg(urdfPath).toUtf8().constData());
    }

    UrdfDraftModelInput model;
    QXmlStreamReader reader(&file);
    bool insideJoint = false;
    int visualDepth = 0;
    int collisionDepth = 0;
    UrdfDraftJointInput currentJoint;
    QHash<QString, int> linkIndexByName;

    auto ensureLinkIndex = [&model, &linkIndexByName](const QString& linkName) -> int {
        const QString normalizedLinkName = linkName.trimmed();
        if (normalizedLinkName.isEmpty())
        {
            throw std::runtime_error("URDF 解析失败：存在未命名 link。");
        }

        const auto existing = linkIndexByName.constFind(normalizedLinkName);
        if (existing != linkIndexByName.constEnd())
        {
            return existing.value();
        }

        UrdfDraftLinkInput link;
        link.link_id = normalizedLinkName;
        const int newIndex = static_cast<int>(model.links.size());
        model.links.push_back(link);
        linkIndexByName.insert(normalizedLinkName, newIndex);
        return newIndex;
    };

    while (!reader.atEnd())
    {
        reader.readNext();
        if (reader.isStartElement())
        {
            const QString elementName = reader.name().toString();
            if (elementName == QStringLiteral("robot"))
            {
                model.robot_name = reader.attributes().value(QStringLiteral("name")).toString().trimmed();
            }
            else if (elementName == QStringLiteral("link"))
            {
                ensureLinkIndex(reader.attributes().value(QStringLiteral("name")).toString());
            }
            else if (elementName == QStringLiteral("joint"))
            {
                insideJoint = true;
                currentJoint = {};
                currentJoint.joint_id = reader.attributes().value(QStringLiteral("name")).toString().trimmed();
                currentJoint.joint_type =
                    NormalizeLowerToken(reader.attributes().value(QStringLiteral("type")).toString());
                if (currentJoint.joint_id.isEmpty())
                {
                    throw std::runtime_error("URDF 解析失败：存在未命名 joint。");
                }
            }
            else if (insideJoint && elementName == QStringLiteral("parent"))
            {
                currentJoint.parent_link_name =
                    reader.attributes().value(QStringLiteral("link")).toString().trimmed();
            }
            else if (insideJoint && elementName == QStringLiteral("child"))
            {
                currentJoint.child_link_name =
                    reader.attributes().value(QStringLiteral("link")).toString().trimmed();
            }
            else if (insideJoint && elementName == QStringLiteral("origin"))
            {
                currentJoint.xyz = ParseUrdfDraftTriple(
                    reader.attributes().value(QStringLiteral("xyz")).toString(),
                    QStringLiteral("origin.xyz"));
                const auto rpyRad = ParseUrdfDraftTriple(
                    reader.attributes().value(QStringLiteral("rpy")).toString(),
                    QStringLiteral("origin.rpy"));
                currentJoint.rpy_deg = {
                    rpyRad[0] * 180.0 / 3.14159265358979323846,
                    rpyRad[1] * 180.0 / 3.14159265358979323846,
                    rpyRad[2] * 180.0 / 3.14159265358979323846};
            }
            else if (insideJoint && elementName == QStringLiteral("axis"))
            {
                currentJoint.axis_xyz = ParseUrdfDraftTriple(
                    reader.attributes().value(QStringLiteral("xyz")).toString(),
                    QStringLiteral("axis.xyz"));
            }
            else if (insideJoint && elementName == QStringLiteral("limit"))
            {
                const QString lowerText = reader.attributes().value(QStringLiteral("lower")).toString().trimmed();
                const QString upperText = reader.attributes().value(QStringLiteral("upper")).toString().trimmed();
                const QString velocityText = reader.attributes().value(QStringLiteral("velocity")).toString().trimmed();
                if (!lowerText.isEmpty() || !upperText.isEmpty() || !velocityText.isEmpty())
                {
                    currentJoint.has_limit = true;
                }

                bool lowerOk = false;
                const double lowerRad = lowerText.toDouble(&lowerOk);
                bool upperOk = false;
                const double upperRad = upperText.toDouble(&upperOk);
                bool velocityOk = false;
                const double velocityRad = velocityText.toDouble(&velocityOk);
                if (lowerOk)
                {
                    currentJoint.lower_deg = lowerRad * 180.0 / 3.14159265358979323846;
                }
                if (upperOk)
                {
                    currentJoint.upper_deg = upperRad * 180.0 / 3.14159265358979323846;
                }
                if (velocityOk)
                {
                    currentJoint.velocity_deg = velocityRad * 180.0 / 3.14159265358979323846;
                }
            }
            else if (elementName == QStringLiteral("visual"))
            {
                ++visualDepth;
            }
            else if (elementName == QStringLiteral("collision"))
            {
                ++collisionDepth;
            }
            else if ((visualDepth > 0 || collisionDepth > 0) && elementName == QStringLiteral("mesh"))
            {
                AppendWarningMessage(
                    model.warning_message,
                    QStringLiteral("当前 DH 草案仅提取运动学主链，不尝试映射 visual/collision Mesh。"));
            }
        }
        else if (reader.isEndElement())
        {
            const QString elementName = reader.name().toString();
            if (elementName == QStringLiteral("joint"))
            {
                if (currentJoint.parent_link_name.trimmed().isEmpty() ||
                    currentJoint.child_link_name.trimmed().isEmpty())
                {
                    throw std::runtime_error(
                        QStringLiteral("URDF joint=%1 缺少 parent/child link 定义。")
                            .arg(currentJoint.joint_id)
                            .toUtf8()
                            .constData());
                }

                const int parentIndex = ensureLinkIndex(currentJoint.parent_link_name);
                const int childIndex = ensureLinkIndex(currentJoint.child_link_name);
                const int jointIndex = static_cast<int>(model.joints.size());
                model.joints.push_back(currentJoint);
                model.links[static_cast<std::size_t>(childIndex)].parent_joint_index = jointIndex;
                model.links[static_cast<std::size_t>(parentIndex)].child_joint_indices.push_back(jointIndex);
                insideJoint = false;
            }
            else if (elementName == QStringLiteral("visual") && visualDepth > 0)
            {
                --visualDepth;
            }
            else if (elementName == QStringLiteral("collision") && collisionDepth > 0)
            {
                --collisionDepth;
            }
        }
    }

    if (reader.hasError())
    {
        throw std::runtime_error(
            QStringLiteral("URDF XML 解析失败：%1。").arg(reader.errorString()).toUtf8().constData());
    }

    if (model.links.empty())
    {
        throw std::runtime_error("URDF 解析失败：未找到任何 link。");
    }

    for (const auto& link : model.links)
    {
        if (link.parent_joint_index < 0)
        {
            model.root_link_names.push_back(link.link_id);
        }
    }

    return model;
}

UrdfDraftTrunkInput ExtractUrdfDraftTrunk(const UrdfDraftModelInput& model)
{
    struct PathCandidate
    {
        std::vector<int> joint_indices;
        int movable_joint_count = 0;
        int total_joint_count = 0;
        QString signature;
    };

    auto buildSignature = [&model](const std::vector<int>& jointIndices) {
        QString signature;
        for (std::size_t index = 0; index < jointIndices.size(); ++index)
        {
            if (index > 0)
            {
                signature.append(QLatin1Char('|'));
            }
            signature.append(model.joints[static_cast<std::size_t>(jointIndices[index])].joint_id);
        }
        return signature;
    };

    auto chooseBetterCandidate = [](const PathCandidate& left, const PathCandidate& right) {
        if (right.movable_joint_count != left.movable_joint_count)
        {
            return right.movable_joint_count > left.movable_joint_count;
        }
        if (right.total_joint_count != left.total_joint_count)
        {
            return right.total_joint_count > left.total_joint_count;
        }
        return right.signature < left.signature;
    };

    // 中文说明：这里不继续复用递归候选搜索，改为在各 root 上执行一条简单 DFS，
    // 只保留“可动关节最多”的主干，满足当前只读草案展示需求。
    std::function<PathCandidate(int)> collectBest = [&](int linkIndex) -> PathCandidate {
        PathCandidate best;
        const auto& link = model.links[static_cast<std::size_t>(linkIndex)];
        for (int jointIndex : link.child_joint_indices)
        {
            const auto& joint = model.joints[static_cast<std::size_t>(jointIndex)];
            const int childLinkIndex = [&model, &joint]() {
                for (std::size_t i = 0; i < model.links.size(); ++i)
                {
                    if (model.links[i].link_id == joint.child_link_name)
                    {
                        return static_cast<int>(i);
                    }
                }
                return -1;
            }();

            PathCandidate candidate;
            if (childLinkIndex >= 0)
            {
                candidate = collectBest(childLinkIndex);
            }
            candidate.joint_indices.insert(candidate.joint_indices.begin(), jointIndex);
            candidate.total_joint_count += 1;
            if (joint.joint_type == QStringLiteral("revolute") || joint.joint_type == QStringLiteral("continuous"))
            {
                candidate.movable_joint_count += 1;
            }
            candidate.signature = buildSignature(candidate.joint_indices);
            if (best.signature.isEmpty() || chooseBetterCandidate(best, candidate))
            {
                best = candidate;
            }
        }
        return best;
    };

    PathCandidate bestOverall;
    for (const QString& rootLinkName : model.root_link_names)
    {
        const int rootIndex = [&model, &rootLinkName]() {
            for (std::size_t index = 0; index < model.links.size(); ++index)
            {
                if (model.links[index].link_id == rootLinkName)
                {
                    return static_cast<int>(index);
                }
            }
            return -1;
        }();
        if (rootIndex < 0)
        {
            continue;
        }

        const auto candidate = collectBest(rootIndex);
        if (bestOverall.signature.isEmpty() || chooseBetterCandidate(bestOverall, candidate))
        {
            bestOverall = candidate;
        }
    }

    if (bestOverall.movable_joint_count <= 0)
    {
        throw std::runtime_error("URDF 解析失败：未找到可提取的 revolute/continuous 主干关节。");
    }

    UrdfDraftTrunkInput trunk;
    trunk.ordered_joint_indices = std::move(bestOverall.joint_indices);
    trunk.movable_joint_count = bestOverall.movable_joint_count;
    if (model.root_link_names.size() > 1)
    {
        AppendWarningMessage(
            trunk.warning_message,
            QStringLiteral("检测到多个根 link，当前 DH 草案仅提取可动关节最多的主干。"));
    }
    for (const auto& link : model.links)
    {
        if (link.child_joint_indices.size() > 1)
        {
            AppendWarningMessage(
                trunk.warning_message,
                QStringLiteral("检测到多分支 URDF 树结构，当前 DH 草案仅用于主干诊断展示。"));
            break;
        }
    }
    return trunk;
}

const UrdfDraftJointInput* FindJointByName(const UrdfDraftModelInput& model, const QString& jointName)
{
    const QString normalizedJointName = jointName.trimmed();
    for (const auto& joint : model.joints)
    {
        if (joint.joint_id == normalizedJointName)
        {
            return &joint;
        }
    }
    return nullptr;
}

QString BuildDraftJointSignature(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    QString signature;
    for (std::size_t index = 0; index < model.joint_limits.size(); ++index)
    {
        if (index > 0)
        {
            signature.append(QLatin1Char('|'));
        }
        signature.append(model.joint_limits[index].joint_id.trimmed());
    }
    return signature;
}

DhDraftExtractionResult ExtractDhDraftFromUrdfModel(
    const QString& absoluteUrdfPath,
    const UrdfDraftModelInput& minimalModel,
    const UrdfDraftTrunkInput& trunk)
{
    DhDraftExtractionResult result;
    auto draftModel = BuildUrdfPreviewModel(absoluteUrdfPath);
    const QFileInfo urdfFileInfo(absoluteUrdfPath);
    draftModel.meta.name = minimalModel.robot_name.trimmed().isEmpty()
        ? urdfFileInfo.completeBaseName()
        : minimalModel.robot_name.trimmed();
    draftModel.meta.source = QStringLiteral("urdf_dh_draft");
    draftModel.model_source_mode = QStringLiteral("urdf_imported");
    draftModel.modeling_mode = QStringLiteral("URDF");
    draftModel.master_model_type = QStringLiteral("urdf");
    draftModel.derived_model_state = QStringLiteral("fresh");
    draftModel.dh_editable = false;
    draftModel.urdf_editable = true;

    int exactJointCount = 0;
    int diagnosticJointCount = 0;
    int mdhExactJointCount = 0;
    QStringList diagnostics;

    const auto baseMountJoint = FindJointByName(minimalModel, QStringLiteral("base_mount_joint"));
    if (baseMountJoint != nullptr)
    {
        draftModel.base_frame = BuildDraftPose(baseMountJoint->xyz, baseMountJoint->rpy_deg);
    }
    const auto flangeFixedJoint = FindJointByName(minimalModel, QStringLiteral("flange_fixed_joint"));
    if (flangeFixedJoint != nullptr)
    {
        draftModel.flange_frame = BuildDraftPose(flangeFixedJoint->xyz, flangeFixedJoint->rpy_deg);
    }
    const auto tcpFixedJoint = FindJointByName(minimalModel, QStringLiteral("tcp_fixed_joint"));
    if (tcpFixedJoint != nullptr)
    {
        draftModel.tcp_frame.translation_m = tcpFixedJoint->xyz;
        draftModel.tcp_frame.rpy_deg = tcpFixedJoint->rpy_deg;
    }

    for (int jointIndex : trunk.ordered_joint_indices)
    {
        const auto& joint = minimalModel.joints[static_cast<std::size_t>(jointIndex)];
        if (joint.joint_type != QStringLiteral("revolute") &&
            joint.joint_type != QStringLiteral("continuous"))
        {
            continue;
        }

        RoboSDP::Kinematics::Dto::KinematicLinkParameterDto link;
        RoboSDP::Kinematics::Dto::KinematicJointLimitDto limit;
        link.link_id = joint.child_link_name.trimmed().isEmpty()
            ? QStringLiteral("link_%1").arg(draftModel.links.size() + 1)
            : joint.child_link_name.trimmed();
        limit.joint_id = joint.joint_id.trimmed().isEmpty()
            ? QStringLiteral("joint_%1").arg(draftModel.joint_limits.size() + 1)
            : joint.joint_id.trimmed();

        const QString preFixedJointName = QStringLiteral("%1_pre_fixed").arg(limit.joint_id);
        const QString postFixedJointName = QStringLiteral("%1_post_fixed").arg(limit.joint_id);
        const auto preFixedJoint = FindJointByName(minimalModel, preFixedJointName);
        const auto postFixedJoint = FindJointByName(minimalModel, postFixedJointName);

        bool extractedExactly = false;
        bool extractedAsMdh = false;
        if (postFixedJoint != nullptr && IsAxisNearPositiveZ(joint.axis_xyz))
        {
            if (preFixedJoint != nullptr)
            {
                link.a = preFixedJoint->xyz[0];
                link.alpha = preFixedJoint->rpy_deg[0];
                link.d = postFixedJoint->xyz[2];
                link.theta_offset = postFixedJoint->rpy_deg[2];
                link.link_id = postFixedJoint->child_link_name.trimmed().isEmpty()
                    ? link.link_id
                    : postFixedJoint->child_link_name.trimmed();
                extractedExactly = true;
                extractedAsMdh = true;
                ++mdhExactJointCount;
            }
            else
            {
                link.a = std::sqrt(
                    postFixedJoint->xyz[0] * postFixedJoint->xyz[0] +
                    postFixedJoint->xyz[1] * postFixedJoint->xyz[1]);
                link.alpha = postFixedJoint->rpy_deg[0];
                link.d = postFixedJoint->xyz[2];
                link.theta_offset = postFixedJoint->rpy_deg[2];
                link.link_id = postFixedJoint->child_link_name.trimmed().isEmpty()
                    ? link.link_id
                    : postFixedJoint->child_link_name.trimmed();
                extractedExactly = true;
            }
        }

        if (!extractedExactly)
        {
            link.a = std::sqrt(joint.xyz[0] * joint.xyz[0] + joint.xyz[1] * joint.xyz[1]);
            link.alpha = joint.rpy_deg[0];
            link.d = joint.xyz[2];
            link.theta_offset = joint.rpy_deg[2];
            ++diagnosticJointCount;

            if (!IsAxisNearPositiveZ(joint.axis_xyz))
            {
                diagnostics.push_back(
                    QStringLiteral("关节 %1 的 axis 不是标准 Z 轴，当前 DH 草案仅作诊断参考。")
                        .arg(limit.joint_id));
            }
            if (!IsNearZero(joint.xyz[1], 1.0e-5))
            {
                diagnostics.push_back(
                    QStringLiteral("关节 %1 的 origin.y 不为 0，当前 DH 草案为近似映射。")
                        .arg(limit.joint_id));
            }
            if (!IsNearZero(joint.rpy_deg[1], 1.0e-4))
            {
                diagnostics.push_back(
                    QStringLiteral("关节 %1 的 origin.pitch 不为 0，当前 DH 草案为近似映射。")
                        .arg(limit.joint_id));
            }
        }
        else
        {
            ++exactJointCount;
        }

        if (joint.has_limit)
        {
            limit.hard_limit = {joint.lower_deg, joint.upper_deg};
            limit.soft_limit = {
                PreferredSoftLimit(joint.lower_deg, true),
                PreferredSoftLimit(joint.upper_deg, false)};
            limit.max_velocity = std::max(1.0, joint.velocity_deg);
        }

        draftModel.links.push_back(link);
        draftModel.joint_limits.push_back(limit);
    }

    draftModel.joint_count = static_cast<int>(draftModel.links.size());
    draftModel.joint_order_signature = BuildDraftJointSignature(draftModel);
    draftModel.parameter_convention =
        (exactJointCount > 0 && exactJointCount == draftModel.joint_count && mdhExactJointCount == exactJointCount)
            ? QStringLiteral("MDH")
            : QStringLiteral("DH");

    if (!trunk.warning_message.trimmed().isEmpty())
    {
        diagnostics.push_back(trunk.warning_message.trimmed());
    }
    if (!minimalModel.warning_message.trimmed().isEmpty())
    {
        diagnostics.push_back(minimalModel.warning_message.trimmed());
    }

    if (draftModel.links.empty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("URDF 导入成功，但未提取到可展示的 DH/MDH 草案。");
        result.extraction_level = QStringLiteral("diagnostic_only");
        result.draft_model = draftModel;
        return result;
    }

    const bool fullExactDraft =
        diagnosticJointCount == 0 && exactJointCount == draftModel.joint_count;
    const bool completeApproximateDraft =
        !draftModel.links.empty() &&
        draftModel.links.size() == draftModel.joint_limits.size() &&
        draftModel.joint_count == static_cast<int>(draftModel.links.size());
    result.extraction_level = fullExactDraft
        ? QStringLiteral("full")
        : (completeApproximateDraft ? QStringLiteral("partial") : QStringLiteral("diagnostic_only"));

    QStringList summaryParts;
    summaryParts.push_back(
        result.extraction_level == QStringLiteral("full")
            ? QStringLiteral("当前草稿由 URDF 主模型完整提取，可作为 DH/MDH 诊断基线。")
            : (result.extraction_level == QStringLiteral("partial")
                   ? QStringLiteral("当前草稿由 URDF 主模型部分提取，存在近似映射段，可作为 DH/MDH 参数化起点，但需重点校核。")
                   : QStringLiteral("当前草稿由 URDF 主模型近似提取，仅用于诊断展示，不建议反写设计参数。")));
    for (const QString& diagnostic : diagnostics)
    {
        if (!diagnostic.trimmed().isEmpty() && !summaryParts.contains(diagnostic.trimmed()))
        {
            summaryParts.push_back(diagnostic.trimmed());
        }
    }
    draftModel.conversion_diagnostics = summaryParts.join(QStringLiteral("；"));
    draftModel.dh_draft_extraction_level = result.extraction_level;
    draftModel.dh_draft_readonly_reason =
        QStringLiteral("当前 DH/MDH 参数表由 URDF 主模型提取，仅用于诊断展示。若需继续参数化设计，请显式切换为 DH/MDH 主模型模式。");
    draftModel.unified_robot_snapshot = BuildUnifiedRobotSnapshot(draftModel);

    result.message = QStringLiteral("URDF 导入成功，已提取 %1 条 DH/MDH 草案，级别=%2。")
        .arg(draftModel.links.size())
        .arg(result.extraction_level);
    if (!diagnostics.isEmpty())
    {
        result.message.append(QStringLiteral(" 提示：%1").arg(diagnostics.join(QStringLiteral("；"))));
    }
    result.draft_model = draftModel;
    return result;
}

/**
 * @brief 为 URDF 预览链路生成最小统一机器人模型 DTO。
 * @details
 * 预览链路只关心共享内核建树所需的最小语义：
 * modeling_mode / source_path / 缓存键。
 * 这里显式将 flange 与 TCP 置零，保证最后一个 URDF link 节点与 Dry-Run TCP 位姿保持一致。
 */
RoboSDP::Kinematics::Dto::KinematicModelDto BuildUrdfPreviewModel(const QString& absoluteUrdfPath)
{
    RoboSDP::Kinematics::Dto::KinematicModelDto model;
    const QFileInfo urdfFileInfo(absoluteUrdfPath);
    model.meta.kinematic_id = QStringLiteral("urdf_preview_%1").arg(urdfFileInfo.completeBaseName());
    model.meta.name = urdfFileInfo.completeBaseName();
    model.meta.source = QStringLiteral("urdf_preview");
    model.master_model_type = QStringLiteral("urdf");
    model.derived_model_state = QStringLiteral("fresh");
    model.dh_editable = false;
    model.urdf_editable = true;
    model.conversion_diagnostics = DefaultConversionDiagnostics(model.master_model_type);
    model.dh_draft_extraction_level = QStringLiteral("diagnostic_only");
    model.dh_draft_readonly_reason =
        QStringLiteral("当前 DH/MDH 参数表由 URDF 主模型提取，仅用于诊断展示。若需继续参数化设计，请显式切换为 DH/MDH 主模型模式。");
    model.modeling_mode = QStringLiteral("URDF");
    model.parameter_convention = QStringLiteral("URDF");
    model.model_source_mode = QStringLiteral("urdf_imported");
    model.backend_type = QStringLiteral("pinocchio_kinematic_backend");
    model.unified_robot_model_ref = QStringLiteral("urdf_preview::%1").arg(absoluteUrdfPath);
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
    model.unified_robot_snapshot = BuildUnifiedRobotSnapshot(model);
    return model;
}

/**
 * @brief 从共享物理内核直接生成 URDF 骨架预览。
 * @details
 * 这是中央 3D 火柴人视图的唯一合法数据来源：
 * 1. 从 SharedRobotKernelRegistry 获取统一 Pinocchio Model；
 * 2. 在 neutral(q) 零位上执行 FK；
 * 3. 按 preview_nodes / preview_segments 元数据组装 DTO。
 */
UrdfImportResult ImportUrdfPreviewWithSharedKernel(
    const QString& urdfFilePath,
    RoboSDP::Logging::ILogger* logger)
{
    UrdfImportResult result;
    result.urdf_file_path = QFileInfo(urdfFilePath).absoluteFilePath();
    result.preview_scene.urdf_file_path = result.urdf_file_path;

    if (urdfFilePath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("URDF 文件路径不能为空。");
        return result;
    }

    const QFileInfo urdfFileInfo(urdfFilePath);
    if (!urdfFileInfo.exists() || !urdfFileInfo.isFile())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::IoError;
        result.message = QStringLiteral("URDF 文件不存在：%1").arg(urdfFileInfo.absoluteFilePath());
        return result;
    }

    auto previewModel = BuildUrdfPreviewModel(urdfFileInfo.absoluteFilePath());
    RoboSDP::Core::Kinematics::SharedRobotKernelRequest request;
    request.kinematic_model = &previewModel;
    request.unified_robot_model_ref = previewModel.unified_robot_model_ref;
    request.modeling_mode = previewModel.modeling_mode;
    request.joint_order_signature = previewModel.joint_order_signature;
    request.allow_structural_alias = false;

    const auto acquireResult =
        RoboSDP::Core::Kinematics::SharedRobotKernelRegistry::Instance().GetOrBuildKernel(request);
    if (!acquireResult.success || acquireResult.model == nullptr)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = acquireResult.metadata.status_message.trimmed().isEmpty()
            ? QStringLiteral("共享 Pinocchio 内核未能生成 URDF 预览模型。")
            : acquireResult.metadata.status_message;
        return result;
    }

    try
    {
        RoboSDP::Core::Kinematics::SharedPinocchioData data(*acquireResult.model);
        // 当前共享内核只承载 Revolute-Z 关节族；对这组关节来说，全零 q 与 neutral(q) 数学等价。
        // 这里显式构造零位配置，避免 MinGW + 自定义 JointCollection 下 neutral 模板实例化失败。
        const Eigen::VectorXd zeroConfiguration =
            Eigen::VectorXd::Zero(static_cast<Eigen::Index>(acquireResult.model->nq));
        pinocchio::forwardKinematics(*acquireResult.model, data, zeroConfiguration);
        pinocchio::updateFramePlacements(*acquireResult.model, data);

        result.preview_scene.model_name = acquireResult.metadata.model_name.trimmed().isEmpty()
            ? QString::fromStdString(acquireResult.model->name)
            : acquireResult.metadata.model_name.trimmed();
        result.preview_scene.visual_geometries = acquireResult.metadata.visual_geometries;
        result.preview_scene.collision_geometries = acquireResult.metadata.collision_geometries;
        // 中文说明：把本次导入对应的统一模型 DTO 一并返回给 UI，
        // 后续拖动关节只需调用 UpdatePreviewPoses，不再重新解析 URDF 或 Mesh。
        previewModel.joint_count = static_cast<int>(acquireResult.model->nq);
        previewModel.pinocchio_model_ready = true;
        previewModel.conversion_diagnostics =
            QStringLiteral("当前预览由导入 URDF 驱动，统一工程主链已同步到共享 Pinocchio 内核。");
        previewModel.unified_robot_snapshot = BuildUnifiedRobotSnapshot(previewModel);

        try
        {
            const auto minimalUrdfModel = ReadUrdfDraftModel(urdfFileInfo.absoluteFilePath());
            const auto trunk = ExtractUrdfDraftTrunk(minimalUrdfModel);
            const auto dhDraftResult =
                ExtractDhDraftFromUrdfModel(urdfFileInfo.absoluteFilePath(), minimalUrdfModel, trunk);
            if (dhDraftResult.IsSuccess())
            {
                result.preview_model = dhDraftResult.draft_model;
                result.preview_model.pinocchio_model_ready = previewModel.pinocchio_model_ready;
                result.preview_model.unified_robot_model_ref = previewModel.unified_robot_model_ref;
                // 🔽 🔽 🔽 【新增这极其关键的一行】 🔽 🔽 🔽
                // 必须保留原 URDF 的签名，否则后续去内核拿缓存时，会因为签名不对被拒绝！
                result.preview_model.joint_order_signature = previewModel.joint_order_signature;
                // 🔼 🔼 🔼 
                result.preview_model.unified_robot_snapshot = BuildUnifiedRobotSnapshot(result.preview_model);
            }
            else
            {
                result.preview_model = previewModel;
                AppendWarningMessage(
                    result.preview_model.conversion_diagnostics,
                    QStringLiteral("DH 草案提取失败：%1").arg(dhDraftResult.message));
            }
        }
        catch (const std::exception& exception)
        {
            result.preview_model = previewModel;
            AppendWarningMessage(
                result.preview_model.conversion_diagnostics,
                QStringLiteral("DH 草案提取异常：%1").arg(QString::fromUtf8(exception.what())));
        }
        catch (...)
        {
            result.preview_model = previewModel;
            AppendWarningMessage(
                result.preview_model.conversion_diagnostics,
                QStringLiteral("DH 草案提取时出现未知异常。"));
        }
        if (result.preview_scene.model_name.trimmed().isEmpty())
        {
            result.preview_scene.model_name = urdfFileInfo.completeBaseName();
        }

        for (const auto& previewNode : acquireResult.metadata.preview_nodes)
        {
            if (previewNode.frame_id < 0 ||
                previewNode.frame_id >= static_cast<int>(acquireResult.model->nframes))
            {
                result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
                result.message = QStringLiteral("共享 Pinocchio 内核返回了非法的预览节点 frame_id=%1。")
                    .arg(previewNode.frame_id);
                return result;
            }

            RoboSDP::Kinematics::Dto::UrdfPreviewNodeDto node;
            node.link_name = previewNode.link_name;
            node.world_pose = ToPreviewPose(data.oMf[static_cast<std::size_t>(previewNode.frame_id)]);
            node.position_m = node.world_pose.position_m;
            result.preview_scene.nodes.push_back(node);
        }

        for (const auto& previewSegment : acquireResult.metadata.preview_segments)
        {
            if (previewSegment.start_frame_id < 0 ||
                previewSegment.start_frame_id >= static_cast<int>(acquireResult.model->nframes) ||
                previewSegment.end_frame_id < 0 ||
                previewSegment.end_frame_id >= static_cast<int>(acquireResult.model->nframes))
            {
                result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
                result.message = QStringLiteral(
                    "共享 Pinocchio 内核返回了非法的预览线段 frame_id=%1 -> %2。")
                    .arg(previewSegment.start_frame_id)
                    .arg(previewSegment.end_frame_id);
                return result;
            }

            RoboSDP::Kinematics::Dto::UrdfPreviewSegmentDto segment;
            segment.joint_name = previewSegment.joint_name;
            segment.joint_type = previewSegment.joint_type;
            segment.parent_link_name = previewSegment.parent_link_name;
            segment.child_link_name = previewSegment.child_link_name;
            segment.joint_axis_xyz = previewSegment.joint_axis_xyz;
            segment.start_position_m =
                ToPreviewPosition(data.oMf[static_cast<std::size_t>(previewSegment.start_frame_id)].translation());
            segment.end_position_m =
                ToPreviewPosition(data.oMf[static_cast<std::size_t>(previewSegment.end_frame_id)].translation());
            result.preview_scene.segments.push_back(segment);
        }

        if (result.preview_scene.IsEmpty())
        {
            result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
            result.message = QStringLiteral("共享 Pinocchio 内核已解析 URDF，但未提取到可预览的骨架节点。");
            return result;
        }

        result.message = QStringLiteral("Pinocchio 共享内核 URDF 导入成功：模型 %1，节点 %2，连杆段 %3。")
            .arg(result.preview_scene.model_name)
            .arg(result.preview_scene.nodes.size())
            .arg(result.preview_scene.segments.size());
        if (!result.preview_model.conversion_diagnostics.trimmed().isEmpty())
        {
            result.message.append(QStringLiteral(" DH 草案：%1").arg(result.preview_model.conversion_diagnostics));
        }
        if (!acquireResult.metadata.warning_message.trimmed().isEmpty())
        {
            result.message.append(QStringLiteral(" 提示：%1").arg(acquireResult.metadata.warning_message.trimmed()));
            if (logger != nullptr)
            {
                logger->Log(
                    RoboSDP::Logging::LogLevel::Warning,
                    QStringLiteral("[Kinematics] URDF 预览存在兼容性提示：%1")
                        .arg(acquireResult.metadata.warning_message.trimmed()));
            }
        }
        return result;
    }
    catch (const std::exception& exception)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
        result.message = QStringLiteral("共享 Pinocchio 内核生成 URDF 预览异常：%1")
            .arg(QString::fromUtf8(exception.what()));
        return result;
    }
    catch (...)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
        result.message = QStringLiteral("共享 Pinocchio 内核生成 URDF 预览时出现未知异常。");
        return result;
    }
}
#endif

} // namespace

// =================================================================================
// KinematicsService 成员方法实现区域
// =================================================================================

/**
 * @brief 普通构造函数（委托调用）
 * @details 当调用者没有提供具体的 Adapter 适配器时，通过委托构造函数（Delegating Constructor）
 * 将纯 Pinocchio 的默认实现装配进入依赖注入路径。
 * Stage 16 之后 FK / IK / Workspace 都统一走共享内核，不再保留 Stage1Legacy 回退链。
 */
KinematicsService::KinematicsService(
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage& storage,
    RoboSDP::Topology::Persistence::TopologyJsonStorage& topologyStorage,
    RoboSDP::Logging::ILogger* logger)
    : KinematicsService( 
          storage,
          topologyStorage,
          std::make_unique<RoboSDP::Kinematics::Adapter::PinocchioKinematicBackendAdapter>(logger), 
          std::make_unique<RoboSDP::Kinematics::Adapter::PinocchioIkSolverAdapter>(logger),         
          logger)
{
}

/**
 * @brief 完整参数的依赖注入构造函数
 * @details 负责装载各类依赖项到成员变量。
 */
KinematicsService::KinematicsService(
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage& storage,
    RoboSDP::Topology::Persistence::TopologyJsonStorage& topologyStorage,
    std::unique_ptr<RoboSDP::Kinematics::Adapter::IKinematicBackendAdapter> backendAdapter,
    std::unique_ptr<RoboSDP::Kinematics::Adapter::IIkSolverAdapter> ikSolverAdapter,
    RoboSDP::Logging::ILogger* logger)
    : m_storage(storage)
    , m_topology_storage(topologyStorage)
    , m_logger(logger)
    , m_backend_adapter(std::move(backendAdapter))
    , m_ik_solver_adapter(std::move(ikSolverAdapter))
{
    // 如果由于某种原因（比如误操作）外部注入了 nullptr，我们在此兜住默认装配，
    // 保证当前业务主链始终由 Pinocchio 后端负责。
    if (m_backend_adapter == nullptr)
    {
        m_backend_adapter = std::make_unique<RoboSDP::Kinematics::Adapter::PinocchioKinematicBackendAdapter>(logger);
    }

    if (m_ik_solver_adapter == nullptr)
    {
        // IK 默认也统一装配 Pinocchio 数值求解器，避免出现历史后端回流。
        m_ik_solver_adapter = std::make_unique<RoboSDP::Kinematics::Adapter::PinocchioIkSolverAdapter>(logger);
    }
}

/**
 * @brief 代理生成空模型 DTO
 */
RoboSDP::Kinematics::Dto::KinematicModelDto KinematicsService::CreateDefaultModel() const
{
    return RoboSDP::Kinematics::Dto::KinematicModelDto::CreateDefault();
}

/**
 * @brief 代理生成空状态 DTO
 */
RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto KinematicsService::CreateDefaultState() const
{
    return RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto::CreateDefault();
}

/**
 * @brief 基于指定的项目目录，加载之前的拓扑(Topology)数据来构建运动学初稿
 * @param projectRootPath 项目根目录路径
 * @return 构造成功的相关状态包裹体
 */
KinematicBuildResult KinematicsService::BuildFromTopology(const QString& projectRootPath) const
{
    KinematicBuildResult result;
    result.state = CreateDefaultState();
    result.topology_file_path = m_topology_storage.BuildAbsoluteFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法从 Topology 构建 Kinematics。");
        return result;
    }

    // 1. 读取前置模块 (Topology) 生成的草稿状态
    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto topologyState;
    result.error_code = m_topology_storage.Load(projectRootPath, topologyState);
    if (result.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.message = QStringLiteral("读取 Topology 草稿失败，请先保存 Topology：%1").arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        return result;
    }

    // 2. 检查模型的约束条件（当前业务逻辑只支持特定的 6R 串联机器人）
    if (topologyState.current_model.robot_definition.robot_type != QStringLiteral("6R_serial") ||
        topologyState.current_model.robot_definition.joint_count != 6)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("当前 Kinematics 最小实现仅支持模板化 6R 串联构型。");
        return result;
    }

    // 3. 执行核心业务映射，构建具体的模型 DTO
    result.state.current_model = BuildModelFromTopology(topologyState.current_model);
    result.message = QStringLiteral("已基于 Topology 生成最小 KinematicModel。");
    return result;
}

/**
 * @brief 执行正运动学计算服务
 * @param model 目标机器人模型参数
 * @param request 计算请求参数（需包含待求角度）
 * @return 计算结果
 */
RoboSDP::Kinematics::Dto::FkResultDto KinematicsService::SolveFk(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const RoboSDP::Kinematics::Dto::FkRequestDto& request) const
{
    // 在下发到底层 C++ 算法库或物理引擎前，执行严格的数据防呆校验
    const auto validation = ValidateModel(model);
    if (!validation.success)
    {
        RoboSDP::Kinematics::Dto::FkResultDto result;
        result.joint_positions_deg = request.joint_positions_deg; // 保持原样返回以匹配请求
        result.message = validation.message; // 报告不通过的原因
        return result; // 校验失败直接提前返回，切断下发途径保护底层安全
    }

    // 校验通过，安全地代理调用适配器的对应计算方法
    return m_backend_adapter->SolveFk(model, request);
}

/**
 * @brief 执行逆运动学计算服务
 * @param model 目标机器人模型参数
 * @param request 计算请求参数（需包含待求的末端笛卡尔位姿）
 * @return 计算结果
 */
RoboSDP::Kinematics::Dto::IkResultDto KinematicsService::SolveIk(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const RoboSDP::Kinematics::Dto::IkRequestDto& request) const
{
    // 同样执行严格模型拦截
    const auto validation = ValidateModel(model);
    if (!validation.success)
    {
        RoboSDP::Kinematics::Dto::IkResultDto result;
        result.message = validation.message;
        return result;
    }

    // 交给独立的 IK Solver 适配器去解决（因为逆解算法往往不同于前向的普通框架）
    return m_ik_solver_adapter->SolveIk(model, request);
}

/**
 * @brief 触发工作空间采样云的生成服务
 * @param model 目标模型
 * @param request 采样请求
 * @return 带有大量 3D 点坐标的结果包
 */
RoboSDP::Kinematics::Dto::WorkspaceResultDto KinematicsService::SampleWorkspace(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const RoboSDP::Kinematics::Dto::WorkspaceRequestDto& request) const
{
    const auto validation = ValidateModel(model);
    if (!validation.success)
    {
        RoboSDP::Kinematics::Dto::WorkspaceResultDto result;
        result.message = validation.message;
        result.requested_sample_count = request.sample_count;
        return result;
    }

    return m_backend_adapter->SampleWorkspace(model, request);
}

/**
 * @brief 检测并返回底层构建的真实运动学语义状态
 * @details 用于验证上层 UI 参数在底层数学库中是否被正确解析
 */
RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto
KinematicsService::InspectBackendBuildContext(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const
{
    RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto result;
    // 获取正在生效的底层提供商信息
    result.backend_id = m_backend_adapter != nullptr ? m_backend_adapter->BackendId() : QStringLiteral("backend_unavailable");
    result.backend_description = m_backend_adapter != nullptr ? m_backend_adapter->BackendDescription() : QStringLiteral("当前未注入运动学 backend。");

    if (m_backend_adapter == nullptr)
    {
        result.status.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
        result.status.status_code = QStringLiteral("backend_missing");
        result.status.status_message = QStringLiteral("当前未注入运动学 backend，无法执行 build-context 校验。");
        return result;
    }

    // C++ RTTI: 尝试将基类指针向下安全转型为具备诊断能力的特定接口
    // 并不是所有的引擎后端都有诊断功能，如果无法转型 (得到 nullptr)，说明不支持。
    const auto* diagnosticsAdapter = dynamic_cast<const RoboSDP::Kinematics::Adapter::IKinematicBackendDiagnosticsAdapter*>(m_backend_adapter.get());
    if (diagnosticsAdapter == nullptr)
    {
        result.status.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
        result.status.status_code = QStringLiteral("backend_diagnostics_unavailable");
        result.status.status_message = QStringLiteral("当前 backend=%1 未实现 build-context 诊断接口，无法输出统一机器人模型语义映射结果。").arg(result.backend_id);
        return result;
    }

    // 支持诊断，返回底层映射结果
    return diagnosticsAdapter->BuildNormalizedContext(model);
}

/**
 * @brief 透传 Pinocchio 原生 FK 干跑诊断
 * @details 该入口不调用 ValidateModel，也不参与 SolveFk 主链选择；
 * 它只确认诊断型 backend 是否能安全执行底层 FK 数学调用。
 */
RoboSDP::Kinematics::Dto::NativeFkDryRunResultDto
KinematicsService::InspectNativeFkDryRun(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const std::vector<double>& joint_positions_deg) const
{
    RoboSDP::Kinematics::Dto::NativeFkDryRunResultDto result;
    result.joint_positions_deg = joint_positions_deg;

    if (m_backend_adapter == nullptr)
    {
        result.message = QStringLiteral("当前未注入运动学 backend，无法执行 Pinocchio FK 干跑。");
        return result;
    }

    const auto* diagnosticsAdapter =
        dynamic_cast<const RoboSDP::Kinematics::Adapter::IKinematicBackendDiagnosticsAdapter*>(m_backend_adapter.get());
    if (diagnosticsAdapter == nullptr)
    {
        result.message = QStringLiteral("当前 backend=%1 未实现 FK 干跑诊断接口。")
            .arg(m_backend_adapter->BackendId());
        return result;
    }

    return diagnosticsAdapter->EvaluateNativeFkDryRun(model, joint_positions_deg);
}

/**
 * @brief 透传 Pinocchio 原生 Jacobian 干跑诊断
 * @details 该入口只服务于测试和诊断；用于查看当前 Pinocchio 共享内核的 build-context 健康度。
 */
RoboSDP::Kinematics::Dto::NativeJacobianDryRunResultDto
KinematicsService::InspectNativeJacobianDryRun(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const std::vector<double>& joint_positions_deg) const
{
    RoboSDP::Kinematics::Dto::NativeJacobianDryRunResultDto result;
    result.joint_positions_deg = joint_positions_deg;

    if (m_backend_adapter == nullptr)
    {
        result.message = QStringLiteral("当前未注入运动学 backend，无法执行 Pinocchio Jacobian 干跑。");
        return result;
    }

    const auto* diagnosticsAdapter =
        dynamic_cast<const RoboSDP::Kinematics::Adapter::IKinematicBackendDiagnosticsAdapter*>(m_backend_adapter.get());
    if (diagnosticsAdapter == nullptr)
    {
        result.message = QStringLiteral("当前 backend=%1 未实现 Jacobian 干跑诊断接口。")
            .arg(m_backend_adapter->BackendId());
        return result;
    }

    return diagnosticsAdapter->EvaluateNativeJacobianDryRun(model, joint_positions_deg);
}

/**
 * @brief 将服务状态进行持久化（存盘到 JSON）
 * @param projectRootPath 项目目录
 * @param state 当前包含所有内容的内存状态
 * @return 存盘结果
 */
KinematicSaveResult KinematicsService::SaveDraft(
    const QString& projectRootPath,
    const RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& state) const
{
    using namespace RoboSDP::Kinematics::Dto;

    KinematicSaveResult result;
    result.model_file_path = m_storage.BuildAbsoluteModelFilePath(projectRootPath);
    result.workspace_file_path = m_storage.BuildAbsoluteWorkspaceFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法保存 Kinematics 草稿。");
        return result;
    }

    // 保存前极其重要的拦截：拒绝将错误的、非法的脏数据写入磁盘污染项目
    const auto validation = ValidateModel(state.current_model);
    if (!validation.success)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = validation.message;
        return result;
    }

    KinematicsWorkspaceStateDto persistedState = state;
    if (persistedState.current_model.master_model_type != QStringLiteral("urdf"))
    {
        QString artifactDiagnosticMessage;
        auto snapshot = persistedState.current_model.unified_robot_snapshot;
        const RoboSDP::Errors::ErrorCode artifactError = WriteDerivedUrdfArtifact(
            projectRootPath,
            persistedState.current_model,
            snapshot,
            artifactDiagnosticMessage);
        persistedState.current_model.unified_robot_snapshot = snapshot;
        persistedState.current_model.conversion_diagnostics = artifactDiagnosticMessage;

        if (artifactError != RoboSDP::Errors::ErrorCode::Ok)
        {
            result.error_code = artifactError;
            result.message = artifactDiagnosticMessage;
            return result;
        }
    }

    // 先保存核心的模型信息
    result.error_code = m_storage.SaveModel(projectRootPath, persistedState);
    if (result.error_code == RoboSDP::Errors::ErrorCode::Ok)
    {
        // 只有核心模型确保安全落盘后，才去保存庞大的工作区点云采样缓存
        result.error_code = m_storage.SaveWorkspaceCache(projectRootPath, persistedState.last_workspace_result);
    }

    // 格式化友好的反馈信息
    result.message = result.IsSuccess()
        ? QStringLiteral("Kinematics 草稿已保存到：%1").arg(result.model_file_path)
        : QStringLiteral("Kinematics 草稿保存失败：%1").arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
    return result;
}

/**
 * @brief 从项目目录下读取历史保存的 JSON 恢复工作状态
 * @param projectRootPath 项目目录
 * @return 读取恢复的结果包
 */
KinematicLoadResult KinematicsService::LoadDraft(const QString& projectRootPath) const
{
    KinematicLoadResult result;
    result.model_file_path = m_storage.BuildAbsoluteModelFilePath(projectRootPath);
    result.workspace_file_path = m_storage.BuildAbsoluteWorkspaceFilePath(projectRootPath);
    result.state = CreateDefaultState();

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法加载 Kinematics 草稿。");
        return result;
    }

    // 调用持久化层读取文件内容反序列化进 state
    result.error_code = m_storage.LoadModel(projectRootPath, result.state);
    if (result.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.message = QStringLiteral("Kinematics 加载失败：%1").arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        return result;
    }

    // 安全防御点：即使从本地读上来的模型也必须校验，
    // 因为外部可能存在手动使用文本编辑器篡改 JSON 注入恶意/非法参数的行为。
    const auto validation = ValidateModel(result.state.current_model);
    if (!validation.success)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = validation.message;
        return result;
    }

    // 尝试加载可能存在的采样点云缓存
    RoboSDP::Kinematics::Dto::WorkspaceResultDto workspaceResult;
    const RoboSDP::Errors::ErrorCode workspaceLoadError = m_storage.LoadWorkspaceCache(projectRootPath, workspaceResult);
    if (workspaceLoadError == RoboSDP::Errors::ErrorCode::Ok)
    {
        result.state.last_workspace_result = workspaceResult;
        result.message = QStringLiteral("Kinematics 已从 JSON 重新加载。");
    }
    else
    {
        // 容错处理：采样缓存较大或可丢弃，它的缺失不应该阻断核心模型的正常加载
        result.message = QStringLiteral("Kinematics 模型已加载，工作空间缓存未找到，已保留默认空结果。");
    }

    return result;
}

/**
 * @brief 根据文件路径智能导入 URDF 并生成预览数据
 * @details 预览数据统一来自 SharedRobotKernelRegistry，不再保留 Qt XML 手工解析分支。
 * @param urdfFilePath 待读取文件
 * @return 解析结果
 */
UrdfImportResult KinematicsService::ImportUrdfPreview(const QString& urdfFilePath) const
{
#if defined(ROBOSDP_HAVE_PINOCCHIO)
    return ImportUrdfPreviewWithSharedKernel(urdfFilePath, m_logger);
#else
    UrdfImportResult result;
    result.urdf_file_path = QFileInfo(urdfFilePath).absoluteFilePath();
    result.preview_scene.urdf_file_path = result.urdf_file_path;
    result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
    result.message = QStringLiteral("当前构建未启用 Pinocchio，共享物理内核不可用，无法导入 URDF 预览。");
    return result;
#endif
}

DhDraftExtractionResult KinematicsService::ExtractDhDraftFromUrdf(const QString& urdfFilePath) const
{
#if defined(ROBOSDP_HAVE_PINOCCHIO)
    DhDraftExtractionResult result;
    const QFileInfo urdfFileInfo(urdfFilePath);
    if (urdfFilePath.trimmed().isEmpty() || !urdfFileInfo.exists() || !urdfFileInfo.isFile())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::IoError;
        result.message = QStringLiteral("URDF 文件不存在，无法提取 DH/MDH 草案：%1").arg(urdfFilePath);
        return result;
    }

    try
    {
        const auto minimalUrdfModel = ReadUrdfDraftModel(urdfFileInfo.absoluteFilePath());
        const auto trunk = ExtractUrdfDraftTrunk(minimalUrdfModel);
        return ExtractDhDraftFromUrdfModel(urdfFileInfo.absoluteFilePath(), minimalUrdfModel, trunk);
    }
    catch (const std::exception& exception)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("URDF -> DH 草案提取失败：%1").arg(QString::fromUtf8(exception.what()));
        return result;
    }
    catch (...)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
        result.message = QStringLiteral("URDF -> DH 草案提取时出现未知异常。");
        return result;
    }
#else
    DhDraftExtractionResult result;
    result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
    result.message = QStringLiteral("当前构建未启用 Pinocchio，无法提取 URDF 对应的 DH/MDH 草案。");
    return result;
#endif
}
/**
 * @brief 高频刷新 URDF 预览 link 全局位姿。
 * @details
 * 该方法只复用 SharedRobotKernelRegistry 中的共享 Pinocchio Model，
 * 执行 FK 并提取 preview_nodes 对应 frame 的绝对位姿。
 * 整个流程不读取 Mesh/URDF 文件，避免拖动关节时发生磁盘 I/O。
 */
PreviewPoseUpdateResult KinematicsService::UpdatePreviewPoses(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const std::vector<double>& joint_positions_deg) const
{
    // 初始化返回结果对象，默认状态是成功的
    PreviewPoseUpdateResult result;

// 【条件编译】：只有当 CMake 探测到系统中安装了 Pinocchio 物理引擎库时，才编译这段核心逻辑
#if defined(ROBOSDP_HAVE_PINOCCHIO)
    
    // ========================================================================
    // 第一步：向“共享内核注册表”申请复用已存在的物理引擎实例
    // ========================================================================
    RoboSDP::Core::Kinematics::SharedRobotKernelRequest request;
    request.kinematic_model = &model;
    request.unified_robot_model_ref = model.unified_robot_model_ref; // 模型的唯一身份标识
    request.modeling_mode = model.modeling_mode;
    request.joint_order_signature = model.joint_order_signature;     // 关节顺序签名（防呆）
    
    // 【核心性能优化点】：allow_structural_alias = true
    // 这句话告诉内核管理器：“请直接把内存里现成的引擎实例借我用一下，千万不要重新分配内存建树！”
    // 这是保证滑块拖动不卡顿的关键。
    request.allow_structural_alias = true;

    // 获取底层物理内核（Model）
    const auto acquireResult =
        RoboSDP::Core::Kinematics::SharedRobotKernelRegistry::Instance().GetOrBuildKernel(request);
        
    // 异常拦截：如果内核不存在（比如刚打开软件还没初始化），直接报错返回
    if (!acquireResult.success || acquireResult.model == nullptr)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = acquireResult.metadata.status_message.trimmed().isEmpty()
            ? QStringLiteral("预览姿态刷新失败：共享 Pinocchio 内核不可用。")
            : acquireResult.metadata.status_message;
        return result;
    }

    // ========================================================================
    // 第二步：输入数据的严格安全校验 (防 Crash)
    // ========================================================================
    // acquireResult.model->nq 是 Pinocchio 引擎中该机器人的自由度（通常是6）
    const auto expectedJointCount = static_cast<std::size_t>(acquireResult.model->nq);
    
    // // 检查 UI 传进来的角度数组长度，是否和物理引擎期望的自由度一致
    // if (joint_positions_deg.size() != expectedJointCount)
    // {
    //     result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
    //     result.message = QStringLiteral("预览姿态刷新失败：输入关节角数量=%1，但共享内核 nq=%2。")
    //         .arg(joint_positions_deg.size())
    //         .arg(expectedJointCount);
    //     return result;
    // }

    // 检查模型元数据里的“零位偏移数组”长度是否合法
    // if (acquireResult.metadata.native_position_offsets_deg.size() != expectedJointCount)
    // {
    //     result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
    //     result.message = QStringLiteral("预览姿态刷新失败：内部零位偏移数量=%1，但共享内核 nq=%2。")
    //         .arg(acquireResult.metadata.native_position_offsets_deg.size())
    //         .arg(expectedJointCount);
    //     return result;
    // }

    // ========================================================================
    // 第三步：执行核心的正运动学 (FK) 计算
    // ========================================================================
    try
    {
        // 实例化 Pinocchio 的 Data 对象，它就像是算草纸，专门用来存放计算过程中的中间矩阵
        RoboSDP::Core::Kinematics::SharedPinocchioData data(*acquireResult.model);
        
        // 准备一个 Eigen 向量，用于存放引擎所需的弧度制关节角 q
        Eigen::VectorXd q = Eigen::VectorXd::Zero(acquireResult.model->nq);
        
        // 遍历 UI 传进来的角度
        const std::size_t loopCount = std::min(joint_positions_deg.size(), expectedJointCount);
        for (std::size_t index = 0; index < loopCount; ++index)
        {
            if (!IsFiniteValue(joint_positions_deg[index])) continue;

            // 中文说明：DTO 输入为 deg，Pinocchio q 使用 rad；DH/MDH 零位偏移也在这里统一叠加。
            // 物理引擎的零位（0度）和 UI 画面的零位可能不一致，这里叠加补偿值
            // 【修复】：URDF 模型可能根本没有 native_position_offsets_deg，必须允许其为空
            double offset = 0.0;
            if (index < acquireResult.metadata.native_position_offsets_deg.size()) {
                offset = acquireResult.metadata.native_position_offsets_deg[index];
            }
                
            // 将角度转为弧度，存入 q 向量
            const double nativeDegrees = joint_positions_deg[index] + offset;
            q[static_cast<Eigen::Index>(index)] = PreviewDegToRad(nativeDegrees);
        }

        // 【算力核心 1】：执行正解。引擎会遍历运动学树，算出每个关节的局部变换
        pinocchio::forwardKinematics(*acquireResult.model, data, q);
        // 【算力核心 2】：更新所有坐标系。基于刚才的关节变换，算出空间中所有挂载点（Frame）的绝对位姿
        pinocchio::updateFramePlacements(*acquireResult.model, data);


    // ========================================================================
    // 第四步：提取计算结果，包装成 UI 认识的格式
    // ========================================================================
        // 遍历元数据中标记为“需要给 3D 渲染器看”的关键节点 (preview_nodes)
        for (const auto& previewNode : acquireResult.metadata.preview_nodes)
        {
            // 安全校验：防止 frame_id 越界导致访问非法内存
            if (previewNode.frame_id < 0 ||
                previewNode.frame_id >= static_cast<int>(acquireResult.model->nframes))
            {
                result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
                result.message = QStringLiteral("预览姿态刷新失败：非法 frame_id=%1。").arg(previewNode.frame_id);
                return result;
            }

            // data.oMf 是一个数组，存放了所有 Frame 在世界坐标系(o)下的绝对变换矩阵(M)
            // ToPreviewPose 函数会将复杂的 Eigen::SE3 矩阵转化为简单的 DTO (x,y,z, rx,ry,rz)
            // 将其存入 map，键为连杆的名字（如 "link_1", "tcp_frame"）
            result.link_world_poses[previewNode.link_name] =
                ToPreviewPose(data.oMf[static_cast<std::size_t>(previewNode.frame_id)]);
        }

        result.message = QStringLiteral("预览姿态刷新完成：已更新 %1 个 link 全局位姿。")
            .arg(result.link_world_poses.size());
        return result;
    }
    // ========================================================================
    // 第五步：C++ 异常兜底 (Exception Handling)
    // ========================================================================
    // 物理引擎内部如果发生矩阵奇异或其他严重错误可能会 throw exception。
    // 在这里必须 catch 住，将其转化为友好的错误码返回，绝对不能让 UI 主程序闪退。
    catch (const std::exception& exception)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
        result.message = QStringLiteral("预览姿态刷新异常：%1").arg(QString::fromUtf8(exception.what()));
        return result;
    }
    catch (...)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
        result.message = QStringLiteral("预览姿态刷新出现未知异常，已拦截以避免崩溃。");
        return result;
    }
#else
// 如果编译时没有物理引擎，直接返回降级报错
    Q_UNUSED(model);
    Q_UNUSED(joint_positions_deg);
    result.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
    result.message = QStringLiteral("当前构建未启用 Pinocchio，无法刷新 URDF 预览姿态。");
    return result;
#endif
}

PreviewSceneBuildResult KinematicsService::BuildDhPreviewScene(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const std::vector<double>& joint_positions_deg,
    const QString& projectRootPath) const
{
    using namespace RoboSDP::Kinematics::Dto;

    PreviewSceneBuildResult result;

    // =========================================================================
    // 第一步：强制状态机重置与元数据初始化
    // =========================================================================
    KinematicModelDto previewModel = model;
    
    // 强制宣告主权：将当前模型切换为纯粹的 DH/MDH 参数驱动模式
    previewModel.master_model_type = QStringLiteral("dh_mdh");
    previewModel.derived_model_state = QStringLiteral("fresh"); // 标记派生状态为“新鲜”（未过期）
    previewModel.dh_editable = true;   // 开放 DH 表格编辑权限
    previewModel.urdf_editable = false; // 封锁 URDF 编辑权限
    
    // 清除由于从 URDF 提取而带来的“只读草案”等历史包袱标记
    previewModel.conversion_diagnostics = DefaultConversionDiagnostics(previewModel.master_model_type);
    previewModel.dh_draft_extraction_level.clear();
    previewModel.dh_draft_readonly_reason.clear();
    
    // 统一建模模式，确保底层引擎知道按照 DH 还是 MDH 来解析矩阵
    previewModel.modeling_mode = previewModel.parameter_convention;
    previewModel.model_source_mode = previewModel.model_source_mode.trimmed().isEmpty()
        ? QStringLiteral("manual_seed")
        : previewModel.model_source_mode.trimmed();
        
    previewModel.backend_type = QStringLiteral("pinocchio_kinematic_backend");
    previewModel.urdf_source_path.clear(); // 既然是 DH 主模型，就抛弃外部引入的 URDF 路径
    previewModel.joint_count = static_cast<int>(previewModel.links.size());
    
    // 生成关节顺序签名 (Joint Order Signature) 和统一模型引用 ID
    // 这是保证底层物理引擎内存池不混乱的“身份证”
    previewModel.joint_order_signature = previewModel.joint_order_signature.trimmed().isEmpty()
        ? BuildPreviewJointOrderSignature(previewModel)
        : previewModel.joint_order_signature.trimmed();
        
    if (previewModel.unified_robot_model_ref.trimmed().isEmpty())
    {
        previewModel.unified_robot_model_ref =
            QStringLiteral("dh_preview::%1").arg(previewModel.meta.kinematic_id.trimmed().isEmpty()
                    ? QStringLiteral("default")
                    : previewModel.meta.kinematic_id.trimmed());
    }
    
    // 初始标记物理引擎为“未就绪”，等待下一步检验
    previewModel.pinocchio_model_ready = false;


    // =========================================================================
    // 第二步：底层物理引擎注册与“盖章” (核心数据流动作)
    // =========================================================================
    // 中文说明：先用统一 build-context 诊断把 DH 主模型接入共享 Pinocchio 内核，
    // 再继续走 SolveFk 主链。这样本轮就能把“中央骨架可见”提升为“统一工程主链已就绪”。
    
    // 将模型送入底层适配器进行预编译，如果 DH 参数合法，底层会生成一棵真实的运动学树
    const auto backendContext = InspectBackendBuildContext(previewModel);
    
    // 【关键动作：静默吸收签名】
    // 底层引擎在解析过程中，会对模型进行归一化，并生成官方的签名。
    // 我们必须把这些“盖过章”的官方签名反写回 previewModel，这样上层 UI 保存的 JSON 才是合法的。
    if (backendContext.context.normalized_model.frame_semantics_version > 0) {
        previewModel.frame_semantics_version = backendContext.context.normalized_model.frame_semantics_version;
    }
    if (!backendContext.context.normalized_model.modeling_mode.trimmed().isEmpty()) {
        previewModel.modeling_mode = backendContext.context.normalized_model.modeling_mode.trimmed();
    }
    if (!backendContext.context.normalized_model.parameter_convention.trimmed().isEmpty()) {
        previewModel.parameter_convention = backendContext.context.normalized_model.parameter_convention.trimmed();
    }
    if (!backendContext.context.normalized_model.normalized_backend_type.trimmed().isEmpty()) {
        previewModel.backend_type = backendContext.context.normalized_model.normalized_backend_type.trimmed();
    }
    if (!backendContext.context.normalized_model.unified_robot_model_ref.trimmed().isEmpty()) {
        previewModel.unified_robot_model_ref = backendContext.context.normalized_model.unified_robot_model_ref.trimmed();
    }
    if (!backendContext.context.normalized_model.joint_order_signature.trimmed().isEmpty()) {
        previewModel.joint_order_signature = backendContext.context.normalized_model.joint_order_signature.trimmed();
    }
    
    // 更新状态：引擎是否真的准备好了？
    previewModel.pinocchio_model_ready = backendContext.status.shared_robot_kernel_ready;
    previewModel.conversion_diagnostics = BuildDhUnifiedChainDiagnostics(backendContext);
    previewModel.unified_robot_snapshot = BuildUnifiedRobotSnapshot(previewModel);

    // 异常拦截：如果输入的参数表有严重错误（如越界、非数字），直接退回
    if (!backendContext.IsSuccess())
    {
        result.error_code = backendContext.status.error_code;
        result.message = backendContext.status.status_message.trimmed().isEmpty()
            ? QStringLiteral("DH/MDH 主模型未能进入统一工程主链。")
            : backendContext.status.status_message;
        result.preview_model = previewModel; // 即便失败，也把带有错误标记的模型还给 UI
        return result;
    }


    // =========================================================================
    // 第三步：计算骨架正解 (获取绘图所需的绝对坐标)
    // =========================================================================
    FkRequestDto request;
    request.joint_positions_deg = joint_positions_deg;
    request.joint_positions_deg.resize(static_cast<std::size_t>(previewModel.joint_count), 0.0);

    // 调用已经就绪的物理引擎，算一次 FK（正运动学）。
    // 目的是拿到每一个连杆在 3D 空间中的 (x, y, z) 绝对位置。
    const FkResultDto fkResult = SolveFk(previewModel, request);
    if (!fkResult.success)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = fkResult.message.isEmpty()
            ? QStringLiteral("DH/MDH 骨架预览生成失败。")
            : fkResult.message;
        return result;
    }


    // =========================================================================
    // 第四步：组装 3D 渲染场景 (Nodes 和 Segments)
    // =========================================================================
    UrdfPreviewSceneDto scene;
    scene.model_name = previewModel.meta.name.trimmed().isEmpty()
        ? QStringLiteral("DH/MDH 骨架预览")
        : previewModel.meta.name.trimmed();

    // 1. 创建基坐标系节点 (Base)
    UrdfPreviewNodeDto baseNode;
    baseNode.link_name = QStringLiteral("base_link");
    baseNode.position_m = previewModel.base_frame.position_m;
    baseNode.world_pose = previewModel.base_frame;
    scene.nodes.push_back(baseNode);

    // 暂存所有节点的位置，为了下面画“连杆线”做准备
    std::map<QString, std::array<double, 3>> nodePositions;
    nodePositions[baseNode.link_name] = baseNode.position_m;

    // 2. 遍历 FK 结果，创建所有的运动学关节节点 (Nodes)
    for (const LinkPoseDto& linkPose : fkResult.link_poses)
    {
        UrdfPreviewNodeDto node;
        node.link_name = linkPose.link_id;
        node.position_m = linkPose.pose.position_m;
        node.world_pose = linkPose.pose;
        scene.nodes.push_back(node);
        nodePositions[node.link_name] = node.position_m;
    }

    // 3. 将相邻的节点连成“骨头” (Segments)
    for (std::size_t index = 0; index < fkResult.link_poses.size(); ++index)
    {
        const QString parentLinkName = index == 0
            ? QStringLiteral("base_link")
            : fkResult.link_poses[index - 1].link_id;
        const QString childLinkName = fkResult.link_poses[index].link_id;
        
        UrdfPreviewSegmentDto segment;
        segment.joint_name = index < previewModel.joint_limits.size()
            ? previewModel.joint_limits[index].joint_id
            : QStringLiteral("joint_%1").arg(static_cast<int>(index) + 1);
        segment.joint_type = QStringLiteral("revolute"); // DH 默认都是旋转关节
        segment.parent_link_name = parentLinkName;
        segment.child_link_name = childLinkName;
        // 起点坐标 (父节点)
        segment.start_position_m = nodePositions[parentLinkName];
        // 终点坐标 (当前节点)
        segment.end_position_m = nodePositions[childLinkName];
        
        scene.segments.push_back(segment);
    }

    // 4. 创建工具中心点节点 (TCP)
    UrdfPreviewNodeDto tcpNode;
    tcpNode.link_name = QStringLiteral("tcp_frame");
    tcpNode.position_m = fkResult.tcp_pose.position_m;
    tcpNode.world_pose = fkResult.tcp_pose;
    scene.nodes.push_back(tcpNode);
    nodePositions[tcpNode.link_name] = tcpNode.position_m;

    // 5. 将最后一个连杆连向 TCP (一根固定的虚拟骨头)
    if (!fkResult.link_poses.empty())
    {
        UrdfPreviewSegmentDto tcpSegment;
        tcpSegment.joint_name = QStringLiteral("tcp_fixed_joint");
        tcpSegment.joint_type = QStringLiteral("fixed");
        tcpSegment.parent_link_name = fkResult.link_poses.back().link_id;
        tcpSegment.child_link_name = tcpNode.link_name;
        tcpSegment.start_position_m = nodePositions[tcpSegment.parent_link_name];
        tcpSegment.end_position_m = tcpNode.position_m;
        scene.segments.push_back(tcpSegment);
    }


    // =========================================================================
    // 第五步：持久化派生产物 (写出 URDF 文件)
    // =========================================================================
    result.preview_scene = scene;
    result.preview_model = previewModel;
    
    // 如果工程目录有效，我们不仅仅是在内存里建树，还要把这棵树写成物理的 .urdf 文件存到硬盘上
    if (!projectRootPath.trimmed().isEmpty())
    {
        QString artifactDiagnosticMessage;
        auto snapshot = result.preview_model.unified_robot_snapshot;
        
        // 调用底层 API，根据 DH 参数直接生成纯 XML 格式的 URDF 文件并写入磁盘
        const RoboSDP::Errors::ErrorCode artifactError = WriteDerivedUrdfArtifact(
            projectRootPath,
            result.preview_model,
            snapshot,
            artifactDiagnosticMessage);
            
        // 更新快照，记录派生文件是否生成成功、相对路径等信息
        result.preview_model.unified_robot_snapshot = snapshot;
        result.preview_model.conversion_diagnostics = artifactDiagnosticMessage;
        
        if (artifactError != RoboSDP::Errors::ErrorCode::Ok)
        {
            result.message = QStringLiteral("已根据 DH/MDH 参数生成中央骨架预览，但派生 URDF 写出失败：%1")
                .arg(artifactDiagnosticMessage);
            return result;
        }
    }

    // =========================================================================
    // 第六步：组装成功提示并返回
    // =========================================================================
    result.message = result.preview_model.unified_robot_snapshot.derived_artifact_exists
        ? QStringLiteral("已根据 DH/MDH 参数生成中央骨架预览，并写出派生 URDF 文件。")
        : (previewModel.pinocchio_model_ready
               ? QStringLiteral("已根据 DH/MDH 参数生成中央骨架预览，并同步统一工程主链。")
               : QStringLiteral("已根据 DH/MDH 参数生成中央骨架预览。"));
               
    return result;
}

RoboSDP::Errors::ErrorCode KinematicsService::WriteDerivedUrdfArtifact(
    const QString& projectRootPath,
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    RoboSDP::Kinematics::Dto::UnifiedRobotModelSnapshotDto& snapshot,
    QString& diagnosticMessage) const
{
#if defined(ROBOSDP_HAVE_PINOCCHIO)
    using namespace RoboSDP::Kinematics::Dto;

    if (projectRootPath.trimmed().isEmpty())
    {
        snapshot.derived_artifact_state_code = QStringLiteral("project_root_missing");
        snapshot.derived_artifact_exists = false;
        snapshot.derived_artifact_fresh = false;
        diagnosticMessage = QStringLiteral("当前项目目录为空，无法写出派生 URDF 文件。");
        return RoboSDP::Errors::ErrorCode::InvalidArgument;
    }

    if (model.links.empty() || model.joint_limits.empty() || model.links.size() != model.joint_limits.size())
    {
        snapshot.derived_artifact_state_code = QStringLiteral("invalid_kinematic_chain");
        snapshot.derived_artifact_exists = false;
        snapshot.derived_artifact_fresh = false;
        diagnosticMessage = QStringLiteral("当前 DH/MDH 主链不完整，无法生成派生 URDF 文件。");
        return RoboSDP::Errors::ErrorCode::InvalidArgument;
    }

    if (snapshot.derived_artifact_relative_path.trimmed().isEmpty())
    {
        const QString normalizedKinematicId = model.meta.kinematic_id.trimmed().isEmpty()
            ? QStringLiteral("default")
            : model.meta.kinematic_id.trimmed();
        snapshot.derived_artifact_relative_path =
            QStringLiteral("kinematics/derived/%1.urdf").arg(normalizedKinematicId);
    }

    const QString artifactAbsolutePath =
        QDir(projectRootPath).absoluteFilePath(snapshot.derived_artifact_relative_path);
    const QFileInfo artifactFileInfo(artifactAbsolutePath);
    QDir artifactDirectory = artifactFileInfo.dir();
    if (!artifactDirectory.exists() && !artifactDirectory.mkpath(QStringLiteral(".")))
    {
        snapshot.derived_artifact_state_code = QStringLiteral("artifact_dir_create_failed");
        snapshot.derived_artifact_exists = false;
        snapshot.derived_artifact_fresh = false;
        diagnosticMessage = QStringLiteral("无法创建派生 URDF 目录：%1").arg(artifactDirectory.absolutePath());
        return RoboSDP::Errors::ErrorCode::RepositoryWriteFailed;
    }

    const QString robotName = model.meta.name.trimmed().isEmpty()
        ? QStringLiteral("derived_dh_robot")
        : model.meta.name.trimmed();
    const QString baseRootLinkName = QStringLiteral("base_root");
    const QString baseLinkName = QStringLiteral("base_link");
    const QString flangeLinkName = QStringLiteral("flange_link");
    const QString tcpLinkName = QStringLiteral("tcp_link");
    const bool useModifiedDh = model.parameter_convention == QStringLiteral("MDH");

    QString urdfText;
    QTextStream stream(&urdfText);
    stream.setRealNumberNotation(QTextStream::FixedNotation);
    stream.setRealNumberPrecision(8);
    stream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    stream << "<robot name=\"" << robotName << "\">\n";
    stream << "  <!-- 中文说明：该文件由 DH/MDH 主模型自动派生，仅承载最小运动学主链与固定坐标系语义。 -->\n";
    stream << "  <!-- 当前阶段不导出真实 visual / collision / inertial 数据，避免误解为高保真工程数字样机。 -->\n";
    stream << BuildUrdfLinkBlock(baseRootLinkName);
    stream << BuildUrdfLinkBlock(baseLinkName);
    stream << "  <joint name=\"base_mount_joint\" type=\"fixed\">\n";
    stream << "    <parent link=\"" << baseRootLinkName << "\"/>\n";
    stream << "    <child link=\"" << baseLinkName << "\"/>\n";
    stream << "    <origin " << BuildUrdfOriginAttributes(model.base_frame) << "/>\n";
    stream << "  </joint>\n";

    QString currentParentLink = baseLinkName;
    for (std::size_t index = 0; index < model.links.size(); ++index)
    {
        const auto& link = model.links[index];
        const auto& jointLimit = model.joint_limits[index];
        const QString safeJointId = jointLimit.joint_id.trimmed().isEmpty()
            ? QStringLiteral("joint_%1").arg(static_cast<int>(index) + 1)
            : jointLimit.joint_id.trimmed();
        const QString safeLinkId = link.link_id.trimmed().isEmpty()
            ? QStringLiteral("link_%1").arg(static_cast<int>(index) + 1)
            : link.link_id.trimmed();
        const QString preLinkName = QStringLiteral("%1_pre_link").arg(safeJointId);
        const QString axisLinkName = QStringLiteral("%1_axis_link").arg(safeJointId);
        const QString postFixedJointName = QStringLiteral("%1_post_fixed").arg(safeJointId);

        if (useModifiedDh)
        {
            const auto prePose = BuildArtifactPoseFromSe3(BuildModifiedDhArtifactPrePlacement(link));
            stream << BuildUrdfLinkBlock(preLinkName);
            stream << "  <joint name=\"" << safeJointId << "_pre_fixed\" type=\"fixed\">\n";
            stream << "    <parent link=\"" << currentParentLink << "\"/>\n";
            stream << "    <child link=\"" << preLinkName << "\"/>\n";
            stream << "    <origin " << BuildUrdfOriginAttributes(prePose) << "/>\n";
            stream << "  </joint>\n";
            currentParentLink = preLinkName;
        }

        stream << BuildUrdfLinkBlock(axisLinkName);
        stream << "  <joint name=\"" << safeJointId << "\" type=\"revolute\">\n";
        stream << "    <parent link=\"" << currentParentLink << "\"/>\n";
        stream << "    <child link=\"" << axisLinkName << "\"/>\n";
        stream << "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
        stream << "    <axis xyz=\"0 0 1\"/>\n";
        stream << "    <limit lower=\"" << NormalizeArtifactNumber(PreviewDegToRad(jointLimit.hard_limit[0])) << "\" "
               << "upper=\"" << NormalizeArtifactNumber(PreviewDegToRad(jointLimit.hard_limit[1])) << "\" "
               << "effort=\"1000.00000000\" "
               << "velocity=\"" << NormalizeArtifactNumber(PreviewDegToRad(jointLimit.max_velocity)) << "\"/>\n";
        stream << "  </joint>\n";

        const auto postPose = useModifiedDh
            ? BuildArtifactPoseFromSe3(BuildModifiedDhArtifactPostPlacement(link))
            : BuildArtifactPoseFromSe3(BuildStandardDhArtifactPostPlacement(link));
        stream << BuildUrdfLinkBlock(safeLinkId);
        stream << "  <joint name=\"" << postFixedJointName << "\" type=\"fixed\">\n";
        stream << "    <parent link=\"" << axisLinkName << "\"/>\n";
        stream << "    <child link=\"" << safeLinkId << "\"/>\n";
        stream << "    <origin " << BuildUrdfOriginAttributes(postPose) << "/>\n";
        stream << "  </joint>\n";
        currentParentLink = safeLinkId;
    }

    stream << BuildUrdfLinkBlock(flangeLinkName);
    stream << "  <joint name=\"flange_fixed_joint\" type=\"fixed\">\n";
    stream << "    <parent link=\"" << currentParentLink << "\"/>\n";
    stream << "    <child link=\"" << flangeLinkName << "\"/>\n";
    stream << "    <origin " << BuildUrdfOriginAttributes(model.flange_frame) << "/>\n";
    stream << "  </joint>\n";

    const auto tcpPose = BuildArtifactPoseFromSe3(BuildArtifactTcpSe3(model.tcp_frame));
    stream << BuildUrdfLinkBlock(tcpLinkName);
    stream << "  <joint name=\"tcp_fixed_joint\" type=\"fixed\">\n";
    stream << "    <parent link=\"" << flangeLinkName << "\"/>\n";
    stream << "    <child link=\"" << tcpLinkName << "\"/>\n";
    stream << "    <origin " << BuildUrdfOriginAttributes(tcpPose) << "/>\n";
    stream << "  </joint>\n";
    stream << "</robot>\n";

    QSaveFile artifactFile(artifactAbsolutePath);
    if (!artifactFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        snapshot.derived_artifact_state_code = QStringLiteral("artifact_open_failed");
        snapshot.derived_artifact_exists = false;
        snapshot.derived_artifact_fresh = false;
        diagnosticMessage = QStringLiteral("无法写入派生 URDF 文件：%1").arg(artifactAbsolutePath);
        return RoboSDP::Errors::ErrorCode::RepositoryWriteFailed;
    }

    const QByteArray payload = urdfText.toUtf8();
    if (artifactFile.write(payload) != payload.size() || !artifactFile.commit())
    {
        snapshot.derived_artifact_state_code = QStringLiteral("artifact_write_failed");
        snapshot.derived_artifact_exists = false;
        snapshot.derived_artifact_fresh = false;
        diagnosticMessage = QStringLiteral("派生 URDF 文件写出失败：%1").arg(artifactAbsolutePath);
        return RoboSDP::Errors::ErrorCode::RepositoryWriteFailed;
    }

    const QFileInfo writtenFileInfo(artifactAbsolutePath);
    snapshot.derived_artifact_relative_path =
        QDir(projectRootPath).relativeFilePath(artifactAbsolutePath).replace(QLatin1Char('\\'), QLatin1Char('/'));
    snapshot.derived_artifact_version = QStringLiteral("file_v%1").arg(snapshot.snapshot_version);
    snapshot.derived_artifact_generated_at_utc =
        writtenFileInfo.lastModified().toUTC().toString(Qt::ISODateWithMs);
    snapshot.derived_artifact_state_code = QStringLiteral("file_generated");
    snapshot.derived_artifact_exists = writtenFileInfo.exists();
    snapshot.derived_artifact_fresh = writtenFileInfo.exists();
    diagnosticMessage = QStringLiteral("当前草稿以 DH/MDH 参数为主模型，派生 URDF 已写出：%1")
        .arg(snapshot.derived_artifact_relative_path);
    return RoboSDP::Errors::ErrorCode::Ok;
#else
    Q_UNUSED(projectRootPath);
    Q_UNUSED(model);
    snapshot.derived_artifact_state_code = QStringLiteral("pinocchio_disabled");
    snapshot.derived_artifact_exists = false;
    snapshot.derived_artifact_fresh = false;
    diagnosticMessage = QStringLiteral("当前构建未启用 Pinocchio，无法写出派生 URDF 文件。");
    return RoboSDP::Errors::ErrorCode::UnknownError;
#endif
}
/**
 * @brief 核心业务映射：将抽象拓扑模型具体化为数学运动学模型
 * @param topologyModel 由系统上游提供的抽象机械结构描述
 * @return 细化的运动学表达对象 DTO
 */
RoboSDP::Kinematics::Dto::KinematicModelDto KinematicsService::BuildModelFromTopology(
    const RoboSDP::Topology::Dto::RobotTopologyModelDto& topologyModel) const
{
    using namespace RoboSDP::Kinematics::Dto;

    KinematicModelDto model = KinematicModelDto::CreateDefault();
    
    // =====================================================================
    // 1. 同步并继承和映射上游带来的元数据信息
    // =====================================================================
    model.meta.kinematic_id = QStringLiteral("kinematic_%1").arg(topologyModel.meta.topology_id);
    model.meta.name = QStringLiteral("%1 运动学模型").arg(topologyModel.meta.name);
    model.meta.source = QStringLiteral("topology");
    model.meta.status = QStringLiteral("ready");
    model.meta.topology_ref = topologyModel.meta.topology_id; // 确立追踪关联
    model.meta.requirement_ref = topologyModel.meta.requirement_ref;
    model.master_model_type = QStringLiteral("dh_mdh");
    model.derived_model_state = QStringLiteral("fresh");
    model.dh_editable = true;
    model.urdf_editable = false;
    model.conversion_diagnostics = DefaultConversionDiagnostics(model.master_model_type);

    // 初始化后端引擎配置与状态标记
    model.modeling_mode = QStringLiteral("DH"); // 强制明确指定为标准 DH
    model.parameter_convention = model.modeling_mode;
    model.unified_robot_model_ref.clear();
    model.model_source_mode = QStringLiteral("topology_derived");
    model.backend_type = QStringLiteral("pinocchio_kinematic_backend");
    model.urdf_source_path.clear();
    model.pinocchio_model_ready = false;
    model.frame_semantics_version = 1;
    model.joint_count = topologyModel.robot_definition.joint_count;

    // =====================================================================
    // 2. 将物理规格落实为实际的几何参数映射 (核心修复区)
    // =====================================================================
    
    // (1) 清理 base_frame，遵循标准 DH 规范，基座高度 d1 将放置在 Link 1 中
    model.base_frame.position_m = {0.0, 0.0, 0.0}; 

    // 安装姿态补偿：根据底座挂载类型，在世界基坐标系中硬性注入预偏转角
    if (topologyModel.robot_definition.base_mount_type == QStringLiteral("wall")) // 壁挂式
    {
        model.base_frame.rpy_deg[1] = 90.0; // 绕 Y 轴转 90 度
    }
    else if (topologyModel.robot_definition.base_mount_type == QStringLiteral("ceiling")) // 倒挂/吊装式
    {
        model.base_frame.rpy_deg[0] = 180.0; // 绕 X 轴翻转 180 度
    }

    // (2) 提取拓扑界面的物理尺寸
    const double d1 = topologyModel.robot_definition.base_height_m;      // 基座高度
    const double a1 = topologyModel.robot_definition.shoulder_offset_m;  // 肩部偏置
    const double a2 = topologyModel.robot_definition.upper_arm_length_m; // 大臂长度
    const double d4 = topologyModel.robot_definition.forearm_length_m;   // 小臂长度
    const double d6 = topologyModel.robot_definition.wrist_offset_m;     // 腕部偏置

    // (3) 严格按照 6R 机器人的标准 D-H 规范映射参数
    if (model.links.size() >= 6)
    {
        // Link 1 (腰关节)
        model.links[0].a = a1;         // 肩部偏置
        model.links[0].alpha = 90.0;
        model.links[0].d = d1;         // 基座高度作为 d1
        model.links[0].theta_offset = 0.0;

        // Link 2 (肩关节)
        model.links[1].a = a2;         // 大臂长度作为 a2
        model.links[1].alpha = 0.0;
        model.links[1].d = 0.0;
        // 补偿 -90 度，使模型在零位时大臂前伸（与 Topology 预览 L 型对齐）
        model.links[1].theta_offset = 0.0;

        // Link 3 (肘关节)
        model.links[2].a = 0.0;
        model.links[2].alpha = 90.0;
        model.links[2].d = 0.0;
        // 补偿 90 度，使模型在零位时小臂垂直（与 Topology 预览 L 型对齐）
        model.links[2].theta_offset = 90.0; 

        // Link 4 (小臂滚动)
        model.links[3].a = 0.0;
        model.links[3].alpha = -90.0;
        model.links[3].d = d4;         // 小臂长度作为 d4
        model.links[3].theta_offset = 0.0;

        // Link 5 (手腕摆动)
        model.links[4].a = 0.0;
        model.links[4].alpha = 90.0;
        model.links[4].d = 0.0;
        model.links[4].theta_offset = 0.0;

        // Link 6 (法兰盘)
        model.links[5].a = 0.0;
        model.links[5].alpha = 0.0;
        model.links[5].d = d6;         // 腕部偏置作为 d6
        model.links[5].theta_offset = 0.0;

        // 业务特性体现：空心手腕补偿
        if (topologyModel.layout.hollow_wrist_required)
        {
            model.links[4].d = 0.12;
            model.links[5].d = 0.10;
            model.tcp_frame.translation_m[2] = 0.12; // 确保 TCP 也同步前移
        }
    }

    // =====================================================================
    // 3. 构建并计算关节动力学限制参数
    // =====================================================================
    model.joint_limits.clear();
    for (const auto& topologyJoint : topologyModel.joints)
    {
        KinematicJointLimitDto limit;
        limit.joint_id = topologyJoint.joint_id;
        limit.hard_limit = topologyJoint.motion_range_deg; // 机械硬止挡位
        
        // 调用辅助函数，动态计算带有退让缓冲区的控制软件限位
        limit.soft_limit = {
            PreferredSoftLimit(topologyJoint.motion_range_deg[0], true),
            PreferredSoftLimit(topologyJoint.motion_range_deg[1], false)};
            
        // 经验公式法则：靠近末端的手腕轴惯量小，允许运行得更快；底座重轴相对较慢
        limit.max_velocity = topologyJoint.axis_index >= 4 ? 260.0 : 180.0;
        limit.max_acceleration = topologyJoint.axis_index >= 4 ? 520.0 : 360.0;
        model.joint_limits.push_back(limit);
    }

    // =====================================================================
    // 4. 生成防篡改字符串签名
    // =====================================================================
    QString jointOrderSignature;
    for (std::size_t index = 0; index < model.joint_limits.size(); ++index)
    {
        if (index > 0)
        {
            jointOrderSignature.append(QLatin1Char('|'));
        }
        jointOrderSignature.append(model.joint_limits[index].joint_id);
    }
    model.joint_order_signature = jointOrderSignature;

    return model;
}

RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto KinematicsService::GenerateSkeletonPreview(
    const RoboSDP::Topology::Dto::RobotTopologyModelDto& topologyModel,
    const std::vector<double>& jointAnglesDeg)
{
    RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto scene;
    scene.model_name = topologyModel.meta.name.isEmpty() ? QStringLiteral("6R 运动学骨架") : topologyModel.meta.name;

    // =====================================================================
    // 1. 提取拓扑物理尺寸
    // =====================================================================
    const double d1 = topologyModel.robot_definition.base_height_m;      // 基座高度
    const double a1 = topologyModel.robot_definition.shoulder_offset_m;  // 肩部偏置
    const double a2 = topologyModel.robot_definition.upper_arm_length_m; // 大臂长度
    const double d4 = topologyModel.robot_definition.forearm_length_m;   // 小臂长度
    const double d6 = topologyModel.robot_definition.wrist_offset_m;     // 腕部偏置

    // =====================================================================
    // 2. 构造标准 D-H (Standard DH) 参数表
    // =====================================================================
    struct DHTableRow { double a, alpha, d, theta; };
    std::array<DHTableRow, 6> dhTable = {{
        {a1,  90.0,  d1,  0.0},    // Link 1: 腰部旋转，应用肩部偏置 a1
        {a2,  0.0,   0.0, -90.0},  // Link 2: 肩部俯仰，初始补偿 -90 度
        {0.0, 90.0,  0.0, 0.0},    // Link 3: 肘部俯仰
        {0.0, -90.0, d4,  0.0},    // Link 4: 小臂自转
        {0.0, 90.0,  0.0, 0.0},    // Link 5: 腕部摆动
        {0.0, 0.0,   d6,  0.0}     // Link 6: 法兰盘自转
    }};

    // =====================================================================
    // 3. 应用输入的关节角度或默认零位补偿
    // =====================================================================
    if (jointAnglesDeg.size() >= 6) {
        for (int i = 0; i < 6; ++i) { dhTable[i].theta += jointAnglesDeg[i]; }
    } else {
        // 【视觉优化】：强制给 J2 和 J3 补偿 90 度，呈现易于观察的“L型展开”姿态
        dhTable[1].theta += 90.0; 
        dhTable[2].theta += 90.0;
    }

    // =====================================================================
    // 4. 正运动学 (Forward Kinematics) 推导
    // =====================================================================
    std::vector<Matrix4x4> globalTransforms;
    
    // 引入基座安装姿态 (Base Mount Type) 补偿
    RoboSDP::Kinematics::Dto::CartesianPoseDto basePose;
    basePose.position_m = {0.0, 0.0, 0.0};
    basePose.rpy_deg = {0.0, 0.0, 0.0};
    if (topologyModel.robot_definition.base_mount_type == QStringLiteral("wall")) {
        basePose.rpy_deg[1] = 90.0; // 壁挂
    } else if (topologyModel.robot_definition.base_mount_type == QStringLiteral("ceiling")) {
        basePose.rpy_deg[0] = 180.0; // 倒挂
    }
    
    Matrix4x4 T_current = PoseToMatrix(basePose);
    globalTransforms.push_back(T_current);

    // 矩阵连乘：T_global_i = T_global_{i-1} * T_local_i
    for (int i = 0; i < 6; ++i) {
        Matrix4x4 T_local = ComputeDHMatrix(dhTable[i].a, dhTable[i].alpha, dhTable[i].d, dhTable[i].theta);
        T_current = T_current * T_local;
        globalTransforms.push_back(T_current);
    }

    // =====================================================================
    // 5. 组装 3D 渲染节点 (Nodes) 与连杆 (Segments)
    // =====================================================================
    auto addNode = [&scene](const QString& name, const Matrix4x4& T) {
        RoboSDP::Kinematics::Dto::UrdfPreviewNodeDto node;
        node.link_name = name;
        node.position_m = T.GetTranslation();
        node.world_pose.position_m = node.position_m;
        node.world_pose.rpy_deg = MatrixToRPY(T);
        scene.nodes.push_back(node);
    };

    auto addSegment = [&scene](const QString& parent, const QString& child, const QString& jointName, 
                               const Matrix4x4& T_parent, const Matrix4x4& T_child) {
        RoboSDP::Kinematics::Dto::UrdfPreviewSegmentDto seg;
        seg.parent_link_name = parent;
        seg.child_link_name = child;
        seg.joint_name = jointName;
        seg.joint_type = QStringLiteral("revolute");
        seg.start_position_m = T_parent.GetTranslation();
        seg.end_position_m = T_child.GetTranslation();
        
        // 拓扑阶段统一采用标准 DH 约定：旋转轴在父坐标系的 Z 轴上
        seg.joint_axis_xyz = T_parent.GetZAxis();
        scene.segments.push_back(seg);
    };

    // 【核心修复】：使用预定义的 7 个节点名称，完美规避 kinematicModel 变量缺失问题
    const std::array<QString, 7> nodeNames = {
        QStringLiteral("Base / J1 (轴)"),
        QStringLiteral("J2 (肩)"),
        QStringLiteral("J3 (肘)"),
        QStringLiteral("J4 (小臂中心)"),
        QStringLiteral("J5 (腕摆中心)"),
        QStringLiteral("J6 (法兰)"),
        QStringLiteral("TCP (末端)")
    };

    // 利用 for 循环动态压入节点
    for (size_t i = 0; i < nodeNames.size(); ++i) {
        addNode(nodeNames[i], globalTransforms[i]);
    }

    // 利用 for 循环动态压入首尾相连的 6 根连杆
    for (size_t i = 0; i < 6; ++i) {
        addSegment(nodeNames[i], 
                   nodeNames[i+1], 
                   QStringLiteral("Joint_%1").arg(i + 1), 
                   globalTransforms[i], 
                   globalTransforms[i+1]);
    }

    return scene;
}

} // namespace RoboSDP::Kinematics::Service
