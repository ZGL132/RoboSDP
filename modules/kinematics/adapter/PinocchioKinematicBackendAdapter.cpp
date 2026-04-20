#include "modules/kinematics/adapter/PinocchioKinematicBackendAdapter.h"

#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QSet>
#include <QStringList>
#include <QXmlStreamReader>
#include <cmath>
#include <exception>
#include <functional>
#include <sstream>
#include <utility>

#if defined(ROBOSDP_HAVE_PINOCCHIO)
#include <Eigen/Geometry>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/frame.hpp>
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/joint/joint-mimic.hpp>
#include <pinocchio/multibody/joint/joint-revolute.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/spatial/se3.hpp>
#endif

namespace RoboSDP::Kinematics::Adapter
{

// 匿名命名空间，包裹一些不对外暴露的辅助工具函数
namespace
{

constexpr double kPi = 3.14159265358979323846;

#if defined(ROBOSDP_HAVE_PINOCCHIO)

/**
 * @brief 最小 Pinocchio 关节集合，只保留 Revolute-Z。
 *
 * @details 这沿用 Dynamics 模块已验证的 MinGW 友好策略：避免链接 conda-forge
 * Pinocchio 默认模板实例化库，同时仍然真实持有 Pinocchio Model/Data 对象。
 */
template<typename Scalar, int Options>
struct MinimalRevoluteZJointCollectionTpl
{
    typedef pinocchio::JointModelRevoluteTpl<Scalar, Options, 2> JointModelRZ;
    typedef pinocchio::JointDataRevoluteTpl<Scalar, Options, 2> JointDataRZ;
    typedef pinocchio::JointModelCompositeTpl<Scalar, Options, MinimalRevoluteZJointCollectionTpl>
        JointModelComposite;
    typedef pinocchio::JointDataCompositeTpl<Scalar, Options, MinimalRevoluteZJointCollectionTpl>
        JointDataComposite;
    typedef pinocchio::JointModelMimicTpl<Scalar, Options, MinimalRevoluteZJointCollectionTpl>
        JointModelMimic;
    typedef pinocchio::JointDataMimicTpl<Scalar, Options, MinimalRevoluteZJointCollectionTpl>
        JointDataMimic;
    typedef boost::variant<
        JointModelRZ,
        boost::recursive_wrapper<JointModelComposite>,
        boost::recursive_wrapper<JointModelMimic>>
        JointModelVariant;
    typedef boost::variant<
        JointDataRZ,
        boost::recursive_wrapper<JointDataComposite>,
        boost::recursive_wrapper<JointDataMimic>>
        JointDataVariant;
};

using NativePinocchioModel = RoboSDP::Core::Kinematics::SharedPinocchioModel;
using NativePinocchioData = RoboSDP::Core::Kinematics::SharedPinocchioData;
using NativeJointModelRZ = RoboSDP::Core::Kinematics::SharedJointModelRZ;

/// @brief URDF 最小关节输入，只用于从本地 URDF 文件组装 Pinocchio 原生树。
struct MinimalUrdfJointInput
{
    QString joint_id;
    QString joint_type;
    QString parent_link_name;
    QString child_link_name;
    int parent_link_index = -1;
    int child_link_index = -1;
    std::array<double, 3> xyz {0.0, 0.0, 0.0};
    std::array<double, 3> rpy {0.0, 0.0, 0.0};
};

/// @brief URDF 最小 link 输入，用于在 adapter 内部建立树状拓扑并提取主干。
struct MinimalUrdfLinkInput
{
    QString link_id;
    int parent_joint_index = -1;
    std::vector<int> child_joint_indices;
};

/// @brief 最小 URDF 语义模型，只保留当前运动学主链真正需要的 link/joint/mesh 预警信息。
struct MinimalUrdfModelInput
{
    QString robot_name;
    std::vector<MinimalUrdfLinkInput> links;
    std::vector<MinimalUrdfJointInput> joints;
    QStringList root_link_names;
    QString warning_message;
};

/// @brief 从复杂 URDF 树中提取出的“主干”结果，保留固定关节链与可动关节链的顺序。
struct MinimalUrdfTrunkInput
{
    std::vector<int> ordered_joint_indices;
    int movable_joint_count = 0;
    QString warning_message;
};

#endif

/// @brief 角度单位转换，DTO 对外统一使用 deg，Pinocchio 原生 API 使用 rad。
double DegToRad(double degrees)
{
    return degrees * kPi / 180.0;
}

/// @brief 以“；”为分隔符追加中文预警，避免多条预警互相覆盖。
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

/// @brief 将字符串去首尾空格并转换为大写，方便进行无视大小写的比对
QString NormalizeUpperToken(const QString& value)
{
    return value.trimmed().toUpper();
}

/// @brief 将字符串去首尾空格并转换为小写
QString NormalizeLowerToken(const QString& value)
{
    return value.trimmed().toLower();
}

/// @brief 检查浮点数是否为安全有限值（防止 NaN 或 Inf 导致物理引擎崩溃）
bool IsFiniteValue(double value)
{
    return std::isfinite(value);
}

/// @brief 将浮点数追加到缓存键，避免结构参数变化后误复用旧的 Pinocchio Model/Data。
void AppendDoubleToKey(QString& key, double value)
{
    key.append(QStringLiteral("|%1").arg(value, 0, 'g', 17));
}

/// @brief 生成原生模型缓存键，核心以 joint_order_signature 为主，同时纳入会影响刚体树的结构字段。
QString BuildNativeModelCacheKey(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const QString& modelingMode,
    const QString& jointOrderSignature)
{
    QString key = modelingMode;
    key.append(QStringLiteral("|")).append(model.parameter_convention.trimmed());
    key.append(QStringLiteral("|")).append(model.model_source_mode.trimmed());
    key.append(QStringLiteral("|")).append(model.urdf_source_path.trimmed());
    key.append(QStringLiteral("|")).append(jointOrderSignature);
    key.append(QStringLiteral("|frames:")).append(QString::number(model.frame_semantics_version));

    for (const auto& link : model.links)
    {
        key.append(QStringLiteral("|link:")).append(link.link_id.trimmed());
        AppendDoubleToKey(key, link.a);
        AppendDoubleToKey(key, link.alpha);
        AppendDoubleToKey(key, link.d);
        AppendDoubleToKey(key, link.theta_offset);
    }

    for (const auto& jointLimit : model.joint_limits)
    {
        key.append(QStringLiteral("|joint:")).append(jointLimit.joint_id.trimmed());
    }

    for (double value : model.base_frame.position_m) AppendDoubleToKey(key, value);
    for (double value : model.base_frame.rpy_deg) AppendDoubleToKey(key, value);
    for (double value : model.flange_frame.position_m) AppendDoubleToKey(key, value);
    for (double value : model.flange_frame.rpy_deg) AppendDoubleToKey(key, value);
    for (double value : model.tcp_frame.translation_m) AppendDoubleToKey(key, value);
    for (double value : model.tcp_frame.rpy_deg) AppendDoubleToKey(key, value);
    return key;
}

/**
 * @brief 根据模型中定义的各关节 ID 顺序，拼装出一个唯一的防篡改“签名字符串”
 * @details 例如有三个关节 J1, J2, J3，拼接结果为 "J1|J2|J3"。
 * 以后系统流转时如果发现关节顺序变成了 "J2|J1|J3"，立刻就能通过签名比对抓出错误。
 */
QString BuildJointOrderSignatureFromLimits(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    QString signature;
    for (std::size_t index = 0; index < model.joint_limits.size(); ++index)
    {
        if (index > 0)
        {
            signature.append(QLatin1Char('|')); // 用管道符分割
        }
        signature.append(model.joint_limits[index].joint_id.trimmed());
    }
    return signature;
}

/// @brief 将签名字符串按管道符拆分回关节 ID 列表
QStringList SplitSignature(const QString& signature)
{
    return signature.split(QLatin1Char('|'), Qt::KeepEmptyParts);
}

/// @brief 结构转换：将 TCP 对象转为通用的笛卡尔位姿对象
RoboSDP::Kinematics::Dto::CartesianPoseDto ToPose(const RoboSDP::Kinematics::Dto::TcpFrameDto& tcpFrame)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = tcpFrame.translation_m;
    pose.rpy_deg = tcpFrame.rpy_deg;
    return pose;
}

#if defined(ROBOSDP_HAVE_PINOCCHIO)

/// @brief 将 DTO 中的平移 + RPY 转换为 Pinocchio 使用的 SE3。
pinocchio::SE3 BuildSe3FromPose(const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    const Eigen::AngleAxisd roll(DegToRad(pose.rpy_deg[0]), Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(DegToRad(pose.rpy_deg[1]), Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(DegToRad(pose.rpy_deg[2]), Eigen::Vector3d::UnitZ());
    const Eigen::Matrix3d rotation = (yaw * pitch * roll).toRotationMatrix();
    const Eigen::Vector3d translation(pose.position_m[0], pose.position_m[1], pose.position_m[2]);
    return pinocchio::SE3(rotation, translation);
}

/// @brief 将 TCP DTO 转为 Pinocchio SE3，语义上表示 flange -> tcp。
pinocchio::SE3 BuildSe3FromTcp(const RoboSDP::Kinematics::Dto::TcpFrameDto& tcpFrame)
{
    return BuildSe3FromPose(ToPose(tcpFrame));
}

/// @brief 将 Pinocchio SE3 转换为项目通用 CartesianPoseDto，避免向外暴露原生对象。
RoboSDP::Kinematics::Dto::CartesianPoseDto BuildPoseFromSe3(const pinocchio::SE3& transform)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = {
        transform.translation()[0],
        transform.translation()[1],
        transform.translation()[2]};

    const Eigen::Matrix3d& rotation = transform.rotation();
    const double sy = std::sqrt(rotation(0, 0) * rotation(0, 0) + rotation(1, 0) * rotation(1, 0));
    const bool singular = sy < 1.0e-9;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    if (!singular)
    {
        roll = std::atan2(rotation(2, 1), rotation(2, 2));
        pitch = std::atan2(-rotation(2, 0), sy);
        yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    }
    else
    {
        roll = std::atan2(-rotation(1, 2), rotation(1, 1));
        pitch = std::atan2(-rotation(2, 0), sy);
        yaw = 0.0;
    }

    pose.rpy_deg = {
        roll * 180.0 / kPi,
        pitch * 180.0 / kPi,
        yaw * 180.0 / kPi};
    return pose;
}

/// @brief 将 Pinocchio SE3 转为项目统一使用的 4x4 齐次变换矩阵，按行主序展平保存。
std::array<double, 16> BuildTransformMatrixFromSe3(const pinocchio::SE3& transform)
{
    std::array<double, 16> matrix {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0};

    const Eigen::Matrix3d& rotation = transform.rotation();
    matrix[0] = rotation(0, 0);
    matrix[1] = rotation(0, 1);
    matrix[2] = rotation(0, 2);
    matrix[4] = rotation(1, 0);
    matrix[5] = rotation(1, 1);
    matrix[6] = rotation(1, 2);
    matrix[8] = rotation(2, 0);
    matrix[9] = rotation(2, 1);
    matrix[10] = rotation(2, 2);
    matrix[3] = transform.translation()[0];
    matrix[7] = transform.translation()[1];
    matrix[11] = transform.translation()[2];
    return matrix;
}

/// @brief 标准 DH 固定部分：Rz(theta_offset) * Tz(d) * Tx(a) * Rx(alpha)。
pinocchio::SE3 BuildStandardDhPlacement(const RoboSDP::Kinematics::Dto::KinematicLinkParameterDto& link)
{
    const double theta = DegToRad(link.theta_offset);
    const double alpha = DegToRad(link.alpha);
    const Eigen::Matrix3d rotation =
        (Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const Eigen::Vector3d translation(link.a * std::cos(theta), link.a * std::sin(theta), link.d);
    return pinocchio::SE3(rotation, translation);
}

/// @brief 标准 DH 的关节后固定段：Tz(d) * Tx(a) * Rx(alpha)，变量 Rz(q) 交给 Pinocchio JointModelRZ。
pinocchio::SE3 BuildStandardDhPostJointPlacement(const RoboSDP::Kinematics::Dto::KinematicLinkParameterDto& link)
{
    const double alpha = DegToRad(link.alpha);
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Vector3d translation(link.a, 0.0, link.d);
    return pinocchio::SE3(rotation, translation);
}

/// @brief 改进 DH 固定部分：Tx(a) * Rx(alpha) * Rz(theta_offset) * Tz(d)。
pinocchio::SE3 BuildModifiedDhPlacement(const RoboSDP::Kinematics::Dto::KinematicLinkParameterDto& link)
{
    const double theta = DegToRad(link.theta_offset);
    const double alpha = DegToRad(link.alpha);
    const Eigen::Matrix3d rotation =
        (Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()))
            .toRotationMatrix();
    const Eigen::Vector3d translation(link.a, -link.d * std::sin(alpha), link.d * std::cos(alpha));
    return pinocchio::SE3(rotation, translation);
}

/// @brief 改进 DH 的关节前固定段：Rx(alpha) * Tx(a)，变量 Rz(q) 交给 Pinocchio JointModelRZ。
pinocchio::SE3 BuildModifiedDhPreJointPlacement(const RoboSDP::Kinematics::Dto::KinematicLinkParameterDto& link)
{
    const double alpha = DegToRad(link.alpha);
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Vector3d translation(link.a, 0.0, 0.0);
    return pinocchio::SE3(rotation, translation);
}

/// @brief 改进 DH 的关节后固定段：Tz(d)，需要挂到下一个 joint placement 或末端 frame placement。
pinocchio::SE3 BuildModifiedDhPostJointPlacement(const RoboSDP::Kinematics::Dto::KinematicLinkParameterDto& link)
{
    return pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, link.d));
}

/// @brief 解析 URDF origin 中的 xyz/rpy 三元组，失败时抛出中文异常。
std::array<double, 3> ParseUrdfTriple(const QString& value, const QString& fieldName)
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

/// @brief 用项目内最小 URDF 读取逻辑提取 link/joint 树与 mesh 预警信息。
MinimalUrdfModelInput ReadMinimalUrdfModel(const QString& urdfPath)
{
    QFile file(urdfPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        throw std::runtime_error(
            QStringLiteral("URDF 解析失败：无法打开文件 %1。").arg(urdfPath).toUtf8().constData());
    }

    MinimalUrdfModelInput model;
    QXmlStreamReader reader(&file);
    bool insideJoint = false;
    int visualDepth = 0;
    int collisionDepth = 0;
    MinimalUrdfJointInput currentJoint;
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

        MinimalUrdfLinkInput link;
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
                currentJoint.joint_type = NormalizeLowerToken(
                    reader.attributes().value(QStringLiteral("type")).toString());
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
                currentJoint.xyz = ParseUrdfTriple(
                    reader.attributes().value(QStringLiteral("xyz")).toString(),
                    QStringLiteral("origin.xyz"));
                const auto rpyRad = ParseUrdfTriple(
                    reader.attributes().value(QStringLiteral("rpy")).toString(),
                    QStringLiteral("origin.rpy"));
                currentJoint.rpy = {
                    rpyRad[0] * 180.0 / kPi,
                    rpyRad[1] * 180.0 / kPi,
                    rpyRad[2] * 180.0 / kPi};
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
                QString meshPath = reader.attributes().value(QStringLiteral("filename")).toString().trimmed();
                if (meshPath.isEmpty())
                {
                    meshPath = reader.attributes().value(QStringLiteral("url")).toString().trimmed();
                }
                const QString normalizedMeshPath = NormalizeLowerToken(meshPath);
                if (!normalizedMeshPath.isEmpty() &&
                    (normalizedMeshPath.startsWith(QStringLiteral("package://")) ||
                     normalizedMeshPath.endsWith(QStringLiteral(".stl")) ||
                     normalizedMeshPath.endsWith(QStringLiteral(".dae")) ||
                     normalizedMeshPath.endsWith(QStringLiteral(".obj")) ||
                     normalizedMeshPath.endsWith(QStringLiteral(".ply"))))
                {
                    AppendWarningMessage(
                        model.warning_message,
                        QStringLiteral("当前运动学版本已解析骨架，但尚未支持外部 Mesh 几何体的 3D 渲染与碰撞计算。"));
                }
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

                currentJoint.parent_link_index = ensureLinkIndex(currentJoint.parent_link_name);
                currentJoint.child_link_index = ensureLinkIndex(currentJoint.child_link_name);
                const int jointIndex = static_cast<int>(model.joints.size());
                model.joints.push_back(currentJoint);

                auto& childLink = model.links[static_cast<std::size_t>(currentJoint.child_link_index)];
                if (childLink.parent_joint_index >= 0)
                {
                    throw std::runtime_error(
                        QStringLiteral("URDF child link=%1 同时挂接了多个父 joint，当前最小实现无法安全处理。")
                            .arg(currentJoint.child_link_name)
                            .toUtf8()
                            .constData());
                }
                childLink.parent_joint_index = jointIndex;

                auto& parentLink = model.links[static_cast<std::size_t>(currentJoint.parent_link_index)];
                parentLink.child_joint_indices.push_back(jointIndex);
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

    if (model.joints.empty())
    {
        throw std::runtime_error("URDF 解析失败：未找到任何 joint。");
    }

    for (const auto& link : model.links)
    {
        if (link.parent_joint_index < 0)
        {
            model.root_link_names.push_back(link.link_id);
        }
    }

    if (model.root_link_names.isEmpty())
    {
        throw std::runtime_error("URDF 解析失败：未找到根 link，当前无法提取主干。");
    }
    return model;
}

/// @brief 从 URDF 树中挑选“可动关节最多”的主干，确保多分支模型也能安全提取最小工作链。
MinimalUrdfTrunkInput ExtractMinimalUrdfTrunk(const MinimalUrdfModelInput& model)
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

    std::function<PathCandidate(int, QSet<int>&)> visitLink = [&](int linkIndex, QSet<int>& visiting) -> PathCandidate {
        if (visiting.contains(linkIndex))
        {
            throw std::runtime_error("URDF 解析失败：检测到环状 link/joint 结构，当前无法安全构建主干。");
        }

        visiting.insert(linkIndex);
        PathCandidate bestCandidate;
        bestCandidate.signature = QStringLiteral("~");
        const auto& link = model.links[static_cast<std::size_t>(linkIndex)];
        if (link.child_joint_indices.empty())
        {
            bestCandidate.signature.clear();
            visiting.remove(linkIndex);
            return bestCandidate;
        }

        for (int jointIndex : link.child_joint_indices)
        {
            const auto& joint = model.joints[static_cast<std::size_t>(jointIndex)];
            PathCandidate candidate;
            candidate.joint_indices.push_back(jointIndex);

            QSet<int> childVisiting = visiting;
            const auto childCandidate = visitLink(joint.child_link_index, childVisiting);
            candidate.joint_indices.insert(
                candidate.joint_indices.end(),
                childCandidate.joint_indices.begin(),
                childCandidate.joint_indices.end());
            candidate.total_joint_count = 1 + childCandidate.total_joint_count;
            candidate.movable_joint_count =
                ((joint.joint_type == QStringLiteral("revolute") || joint.joint_type == QStringLiteral("continuous"))
                     ? 1
                     : 0) +
                childCandidate.movable_joint_count;
            candidate.signature = buildSignature(candidate.joint_indices);

            if (bestCandidate.signature == QStringLiteral("~") || chooseBetterCandidate(bestCandidate, candidate))
            {
                bestCandidate = candidate;
            }
        }

        visiting.remove(linkIndex);
        return bestCandidate;
    };

    PathCandidate bestOverall;
    bestOverall.signature = QStringLiteral("~");
    for (const QString& rootLinkName : model.root_link_names)
    {
        const int rootLinkIndex = [&model, &rootLinkName]() {
            for (std::size_t index = 0; index < model.links.size(); ++index)
            {
                if (model.links[index].link_id == rootLinkName)
                {
                    return static_cast<int>(index);
                }
            }
            return -1;
        }();
        if (rootLinkIndex < 0)
        {
            continue;
        }

        QSet<int> visiting;
        const auto candidate = visitLink(rootLinkIndex, visiting);
        if (bestOverall.signature == QStringLiteral("~") || chooseBetterCandidate(bestOverall, candidate))
        {
            bestOverall = candidate;
        }
    }

    if (bestOverall.movable_joint_count <= 0)
    {
        throw std::runtime_error("URDF 解析失败：未找到可映射到 Pinocchio 的 revolute/continuous 主干关节。");
    }

    MinimalUrdfTrunkInput trunk;
    trunk.ordered_joint_indices = std::move(bestOverall.joint_indices);
    trunk.movable_joint_count = bestOverall.movable_joint_count;

    if (model.root_link_names.size() > 1)
    {
        AppendWarningMessage(
            trunk.warning_message,
            QStringLiteral("检测到多个根 link，当前仅按可动关节最多的主干提取骨架。"));
    }

    for (const auto& link : model.links)
    {
        if (link.child_joint_indices.size() > 1)
        {
            AppendWarningMessage(
                trunk.warning_message,
                QStringLiteral("检测到 URDF 多分支树结构，当前仅提取可动关节最多的主干用于运动学主链。"));
            break;
        }
    }
    return trunk;
}

/// @brief 将 URDF origin 中读取到的位姿转换为 Pinocchio 关节安装位姿。
pinocchio::SE3 BuildUrdfJointPlacement(const MinimalUrdfJointInput& joint)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = joint.xyz;
    pose.rpy_deg = joint.rpy;
    return BuildSe3FromPose(pose);
}

#endif

/// @brief 生成“占位符报错信息”，明确告诉外界当前还没写完实际的算法
QString BuildStubMessage(const QString& actionName, const QString& statusMessage)
{
    return QStringLiteral("Pinocchio 运动学后端尚未接管 %1 主路径：%2")
        .arg(actionName, statusMessage);
}

/**
 * @brief 生成确定性的伪随机比例（范围 0.0 到 1.0）
 * @details
 * 这里刻意沿用历史工作空间采样的关节分布口径，
 * 以保证在相同 sampleIndex / jointIndex 下，Pinocchio 版本仍能生成稳定可复现的关节角分布，
 * 便于后续包围盒极值和点云统计摘要持续对齐。
 */
double DeterministicRatio(int sampleIndex, int jointIndex)
{
    const int seed = (sampleIndex + 1) * (jointIndex * 17 + 13);
    const int folded = (seed % 9973 + 9973) % 9973;
    return static_cast<double>(folded) / 9972.0;
}

/// @brief 计算三维位置点到世界原点的半径，用于维护 max_radius_m。
double Radius(const std::array<double, 3>& position)
{
    return std::sqrt(
        position[0] * position[0] +
        position[1] * position[1] +
        position[2] * position[2]);
}

} // namespace

// =================================================================================
// 成员方法实现
// =================================================================================

struct PinocchioKinematicBackendAdapter::NativeKernelState
{
#if defined(ROBOSDP_HAVE_PINOCCHIO)
    /// @brief Pinocchio 原生机器人模型，只在 adapter 内部持有，禁止向外暴露。
    std::shared_ptr<const NativePinocchioModel> model;

    /// @brief 与 model 配套的 Pinocchio 运行时数据缓存，只在 adapter 内部持有。
    std::unique_ptr<NativePinocchioData> data;

    /// @brief base 语义 frame 的原生索引，仅在 adapter 内部使用。
    pinocchio::FrameIndex base_frame_id = 0;

    /// @brief flange 语义 frame 的原生索引，仅在 adapter 内部使用。
    pinocchio::FrameIndex flange_frame_id = 0;

    /// @brief tcp 语义 frame 的原生索引，仅在 adapter 内部使用。
    pinocchio::FrameIndex tcp_frame_id = 0;

    /// @brief 每个 Pinocchio q 分量需要叠加的零位偏移，单位为度。
    std::vector<double> native_position_offsets_deg;

    /// @brief 每个业务连杆在原生模型中的语义 frame 索引，用于把 FK 结果回填到 FkResultDto::link_poses。
    std::vector<pinocchio::FrameIndex> link_frame_ids;
#endif

    /// @brief 最近一次成功构建的缓存键，用于避免相同结构反复实例化。
    QString cache_key;

    /// @brief 当前 adapter 生命周期内真实构建 Model/Data 的累计次数。
    int build_count = 0;

    /// @brief 最近一次原生模型关节数量。
    int native_joint_count = 0;

    /// @brief 最近一次原生模型 frame 数量。
    int native_frame_count = 0;

    /// @brief 最近一次构建是否已完成真实共享内核。
    bool ready = false;

    /// @brief 最近一次构建失败或未 ready 的中文原因。
    QString last_message;

    /// @brief 最近一次成功构建时携带的非阻断性预警，例如 mesh 暂不支持或多分支主干裁剪提示。
    QString last_warning_message;
};

PinocchioKinematicBackendAdapter::PinocchioKinematicBackendAdapter(RoboSDP::Logging::ILogger* logger)
    : m_logger(logger)
    , m_native_kernel_state(std::make_unique<NativeKernelState>())
{
    // 初始化时，给最后一次的体检报告塞入默认的“未执行”状态
    m_last_build_context_result.backend_id = BackendId();
    m_last_build_context_result.backend_description = BackendDescription();
    m_last_build_context_result.status.error_code = RoboSDP::Errors::ErrorCode::UnknownError;
    m_last_build_context_result.status.status_code = QStringLiteral("not_built_yet");
    m_last_build_context_result.status.status_message =
        QStringLiteral("尚未执行 build-context 校验，当前共享 Pinocchio 内核尚未 ready。");
}

PinocchioKinematicBackendAdapter::~PinocchioKinematicBackendAdapter() = default;

QString PinocchioKinematicBackendAdapter::CurrentBuildStatusCode(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const
{
    return ValidateBuildContext(model).status.status_code;
}

QString PinocchioKinematicBackendAdapter::CurrentBuildStatusMessage(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const
{
    return ValidateBuildContext(model).status.status_message;
}

QString PinocchioKinematicBackendAdapter::BackendId() const
{
    return QStringLiteral("pinocchio_kinematic_backend");
}

QString PinocchioKinematicBackendAdapter::BackendDescription() const
{
    return QStringLiteral("Pinocchio 共享底层运动学后端（纯 Pinocchio 模式）。");
}

bool PinocchioKinematicBackendAdapter::UsesSharedRobotKernel() const
{
    // 中文说明：只有内部真正持有 Pinocchio Model/Data 时，才允许对外宣称共享内核 ready。
    return m_native_kernel_state != nullptr && m_native_kernel_state->ready;
}

std::uintptr_t PinocchioKinematicBackendAdapter::NativeModelAddressForDiagnostics() const
{
#if defined(ROBOSDP_HAVE_PINOCCHIO)
    return (m_native_kernel_state != nullptr && m_native_kernel_state->model != nullptr)
        ? reinterpret_cast<std::uintptr_t>(m_native_kernel_state->model.get())
        : 0U;
#else
    return 0U;
#endif
}

std::uintptr_t PinocchioKinematicBackendAdapter::NativeDataAddressForDiagnostics() const
{
#if defined(ROBOSDP_HAVE_PINOCCHIO)
    return (m_native_kernel_state != nullptr && m_native_kernel_state->data != nullptr)
        ? reinterpret_cast<std::uintptr_t>(m_native_kernel_state->data.get())
        : 0U;
#else
    return 0U;
#endif
}

/**
 * @brief 执行 FK 主链求解
 * @details
 * 当前策略是“纯 Pinocchio 主链”：
 * 1. 先尝试复用已清洗好的 build-context 与原生 Model/Data。
 * 2. 主路径成功时，直接从 Pinocchio 的世界位姿中装配 FkResultDto。
 * 3. 只要主路径任一步失败，就立即返回明确错误，让上层直接感知失败原因。
 */
RoboSDP::Kinematics::Dto::FkResultDto PinocchioKinematicBackendAdapter::SolveFk(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const RoboSDP::Kinematics::Dto::FkRequestDto& request) const
{
    RoboSDP::Kinematics::Dto::FkResultDto result;
    result.joint_positions_deg = request.joint_positions_deg;
    const auto failWithReason = [this, &request](const QString& reason)
        -> RoboSDP::Kinematics::Dto::FkResultDto
    {
        RoboSDP::Kinematics::Dto::FkResultDto failureResult;
        failureResult.joint_positions_deg = request.joint_positions_deg;
        failureResult.success = false;
        failureResult.message = QStringLiteral("Pinocchio FK 计算失败：%1").arg(reason);

        if (m_logger != nullptr)
        {
            m_logger->Log(
                RoboSDP::Logging::LogLevel::Warning,
                failureResult.message,
                RoboSDP::Errors::ErrorCode::Ok,
                {
                    QStringLiteral("Kinematics"),
                    QStringLiteral("SolveFk"),
                    QStringLiteral("PinocchioKinematicBackendAdapter")});
        }
        return failureResult;
    };

    const auto buildResult = BuildNormalizedContext(model);
    if (!buildResult.IsSuccess() || !buildResult.status.shared_robot_kernel_ready)
    {
        return failWithReason(QStringLiteral("Pinocchio build-context 未就绪：%1")
            .arg(buildResult.status.status_message));
    }

#if !defined(ROBOSDP_HAVE_PINOCCHIO)
    return failWithReason(QStringLiteral("当前构建未启用 Pinocchio C++ 依赖。"));
#else
    if (m_native_kernel_state == nullptr ||
        m_native_kernel_state->model == nullptr ||
        m_native_kernel_state->data == nullptr ||
        !m_native_kernel_state->ready)
    {
        return failWithReason(QStringLiteral("原生 Model/Data 未实例化。"));
    }

    const auto expectedConfigCount = static_cast<std::size_t>(m_native_kernel_state->model->nq);
    if (m_native_kernel_state->native_position_offsets_deg.size() != expectedConfigCount)
    {
        return failWithReason(QStringLiteral("内部零位偏移数量=%1，但原生模型 nq=%2。")
            .arg(m_native_kernel_state->native_position_offsets_deg.size())
            .arg(expectedConfigCount));
    }

    if (request.joint_positions_deg.size() != expectedConfigCount)
    {
        return failWithReason(QStringLiteral("输入关节角数量=%1，但原生模型 nq=%2。")
            .arg(request.joint_positions_deg.size())
            .arg(expectedConfigCount));
    }

    try
    {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(m_native_kernel_state->model->nq);
        for (std::size_t index = 0; index < request.joint_positions_deg.size(); ++index)
        {
            if (!IsFiniteValue(request.joint_positions_deg[index]))
            {
                return failWithReason(QStringLiteral("第 %1 个关节角不是有效数值。").arg(index + 1));
            }

            const double nativeDegrees =
                request.joint_positions_deg[index] + m_native_kernel_state->native_position_offsets_deg[index];
            q[static_cast<Eigen::Index>(index)] = DegToRad(nativeDegrees);
        }

        pinocchio::forwardKinematics(
            *m_native_kernel_state->model,
            *m_native_kernel_state->data,
            q);
        pinocchio::updateFramePlacements(
            *m_native_kernel_state->model,
            *m_native_kernel_state->data);

        result.link_poses.clear();
        const std::size_t availableLinkFrames = m_native_kernel_state->link_frame_ids.size();
        const std::size_t linkPoseCount = std::min(model.links.size(), availableLinkFrames);
        if (buildResult.context.normalized_model.modeling_mode != QStringLiteral("URDF") &&
            availableLinkFrames != model.links.size())
        {
            return failWithReason(QStringLiteral("业务连杆数量=%1，但原生 link frame 数量=%2。")
                .arg(model.links.size())
                .arg(availableLinkFrames));
        }

        for (std::size_t index = 0; index < linkPoseCount; ++index)
        {
            const auto frameId = m_native_kernel_state->link_frame_ids[index];
            if (frameId >= static_cast<pinocchio::FrameIndex>(m_native_kernel_state->model->nframes))
            {
                return failWithReason(QStringLiteral("第 %1 个 link frame_id=%2 超出原生 frame 数量=%3。")
                    .arg(index + 1)
                    .arg(frameId)
                    .arg(m_native_kernel_state->model->nframes));
            }

            RoboSDP::Kinematics::Dto::LinkPoseDto linkPose;
            linkPose.link_id = model.links[index].link_id;
            linkPose.pose = BuildPoseFromSe3(m_native_kernel_state->data->oMf[frameId]);
            result.link_poses.push_back(linkPose);
        }

        if (m_native_kernel_state->tcp_frame_id >= static_cast<pinocchio::FrameIndex>(m_native_kernel_state->model->nframes))
        {
            return failWithReason(QStringLiteral("TCP frame_id=%1 超出原生 frame 数量=%2。")
                .arg(m_native_kernel_state->tcp_frame_id)
                .arg(m_native_kernel_state->model->nframes));
        }

        const pinocchio::SE3& tcpTransform = m_native_kernel_state->data->oMf[m_native_kernel_state->tcp_frame_id];
        result.tcp_pose = BuildPoseFromSe3(tcpTransform);
        result.tcp_transform_matrix = BuildTransformMatrixFromSe3(tcpTransform);
        result.success = true;
        result.message = buildResult.context.normalized_model.modeling_mode == QStringLiteral("URDF") &&
                linkPoseCount != model.links.size()
            ? QStringLiteral("FK 求解完成：由 Pinocchio 原生内核计算；URDF 原生链路仅回填了 %1/%2 个业务连杆位姿。")
                  .arg(linkPoseCount)
                  .arg(model.links.size())
            : QStringLiteral("FK 求解完成：由 Pinocchio 原生内核计算。");

        if (m_logger != nullptr)
        {
            m_logger->Log(
                RoboSDP::Logging::LogLevel::Info,
                result.message,
                RoboSDP::Errors::ErrorCode::Ok,
                {
                    QStringLiteral("Kinematics"),
                    QStringLiteral("SolveFk"),
                    QStringLiteral("PinocchioKinematicBackendAdapter")});
        }
        return result;
    }
    catch (const std::exception& exception)
    {
        return failWithReason(QStringLiteral("Pinocchio FK 异常：%1").arg(QString::fromUtf8(exception.what())));
    }
    catch (...)
    {
        return failWithReason(QStringLiteral("Pinocchio FK 出现未知异常，已拦截。"));
    }
#endif
}

/**
 * @brief 执行工作空间采样
 * @details
 * 当前策略升级为“纯 Pinocchio 主链”：
 * 1. 先尝试复用 build-context 与原生 Model/Data。
 * 2. 主路径成功时，在 Adapter 内部直接完成确定性伪随机采样与包围盒摘要统计。
 * 3. 若构建失败、输入不一致或底层抛出异常，则直接返回明确错误信息。
 */
RoboSDP::Kinematics::Dto::WorkspaceResultDto PinocchioKinematicBackendAdapter::SampleWorkspace(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const RoboSDP::Kinematics::Dto::WorkspaceRequestDto& request) const
{
    RoboSDP::Kinematics::Dto::WorkspaceResultDto result;
    result.requested_sample_count = std::max(1, request.sample_count);
    const auto failWithReason = [this, &request](const QString& reason)
        -> RoboSDP::Kinematics::Dto::WorkspaceResultDto
    {
        RoboSDP::Kinematics::Dto::WorkspaceResultDto failureResult;
        failureResult.requested_sample_count = std::max(1, request.sample_count);
        failureResult.success = false;
        failureResult.message = QStringLiteral("Pinocchio 工作空间采样失败：%1").arg(reason);

        if (m_logger != nullptr)
        {
            m_logger->Log(
                RoboSDP::Logging::LogLevel::Warning,
                failureResult.message,
                RoboSDP::Errors::ErrorCode::Ok,
                {
                    QStringLiteral("Kinematics"),
                    QStringLiteral("SampleWorkspace"),
                    QStringLiteral("PinocchioKinematicBackendAdapter")});
        }
        return failureResult;
    };

    const auto buildResult = BuildNormalizedContext(model);
    if (!buildResult.IsSuccess() || !buildResult.status.shared_robot_kernel_ready)
    {
        return failWithReason(QStringLiteral("Pinocchio build-context 未就绪：%1")
            .arg(buildResult.status.status_message));
    }

#if !defined(ROBOSDP_HAVE_PINOCCHIO)
    return failWithReason(QStringLiteral("当前构建未启用 Pinocchio C++ 依赖。"));
#else
    if (m_native_kernel_state == nullptr ||
        m_native_kernel_state->model == nullptr ||
        m_native_kernel_state->data == nullptr ||
        !m_native_kernel_state->ready)
    {
        return failWithReason(QStringLiteral("原生 Model/Data 未实例化。"));
    }

    const auto expectedConfigCount = static_cast<std::size_t>(m_native_kernel_state->model->nq);
    if (expectedConfigCount == 0)
    {
        return failWithReason(QStringLiteral("原生模型 nq=0，无法执行 Pinocchio 工作空间采样。"));
    }

    if (m_native_kernel_state->native_position_offsets_deg.size() != expectedConfigCount)
    {
        return failWithReason(QStringLiteral("内部零位偏移数量=%1，但原生模型 nq=%2。")
            .arg(m_native_kernel_state->native_position_offsets_deg.size())
            .arg(expectedConfigCount));
    }

    if (model.joint_limits.size() != expectedConfigCount)
    {
        return failWithReason(QStringLiteral("joint_limits 数量=%1，但原生模型 nq=%2，当前无法在 Pinocchio 主路径中完成工作空间采样。")
            .arg(model.joint_limits.size())
            .arg(expectedConfigCount));
    }

    if (m_native_kernel_state->tcp_frame_id >= static_cast<pinocchio::FrameIndex>(m_native_kernel_state->model->nframes))
    {
        return failWithReason(QStringLiteral("TCP frame_id=%1 超出原生 frame 数量=%2。")
            .arg(m_native_kernel_state->tcp_frame_id)
            .arg(m_native_kernel_state->model->nframes));
    }

    try
    {
        // 性能优化：高频采样循环外提前分配 q 和关节角缓存，避免成千上万次重复堆分配。
        Eigen::VectorXd q = Eigen::VectorXd::Zero(m_native_kernel_state->model->nq);
        std::vector<double> jointPositionsDeg(expectedConfigCount, 0.0);
        result.sampled_points.clear();
        result.sampled_points.reserve(static_cast<std::size_t>(result.requested_sample_count));

        bool hasPoint = false;
        for (int sampleIndex = 0; sampleIndex < result.requested_sample_count; ++sampleIndex)
        {
            for (std::size_t jointIndex = 0; jointIndex < expectedConfigCount; ++jointIndex)
            {
                const auto& limits = model.joint_limits[jointIndex].hard_limit;
                if (!IsFiniteValue(limits[0]) ||
                    !IsFiniteValue(limits[1]) ||
                    limits[0] > limits[1])
                {
                    return failWithReason(QStringLiteral("第 %1 个关节 hard_limit 非法，无法执行 Pinocchio 工作空间采样。")
                        .arg(jointIndex + 1));
                }

                const double ratio = DeterministicRatio(sampleIndex, static_cast<int>(jointIndex));
                jointPositionsDeg[jointIndex] = limits[0] + (limits[1] - limits[0]) * ratio;
                const double nativeDegrees =
                    jointPositionsDeg[jointIndex] + m_native_kernel_state->native_position_offsets_deg[jointIndex];
                q[static_cast<Eigen::Index>(jointIndex)] = DegToRad(nativeDegrees);
            }

            pinocchio::forwardKinematics(
                *m_native_kernel_state->model,
                *m_native_kernel_state->data,
                q);
            pinocchio::updateFramePlacements(
                *m_native_kernel_state->model,
                *m_native_kernel_state->data);

            const pinocchio::SE3& tcpTransform = m_native_kernel_state->data->oMf[m_native_kernel_state->tcp_frame_id];
            RoboSDP::Kinematics::Dto::WorkspacePointDto point;
            point.joint_positions_deg = jointPositionsDeg;
            point.tcp_pose = BuildPoseFromSe3(tcpTransform);
            result.sampled_points.push_back(point);
            ++result.reachable_sample_count;

            result.max_radius_m = std::max(result.max_radius_m, Radius(point.tcp_pose.position_m));
            if (!hasPoint)
            {
                result.min_position_m = point.tcp_pose.position_m;
                result.max_position_m = point.tcp_pose.position_m;
                hasPoint = true;
            }
            else
            {
                for (int axis = 0; axis < 3; ++axis)
                {
                    result.min_position_m[axis] =
                        std::min(result.min_position_m[axis], point.tcp_pose.position_m[axis]);
                    result.max_position_m[axis] =
                        std::max(result.max_position_m[axis], point.tcp_pose.position_m[axis]);
                }
            }
        }

        result.success = result.reachable_sample_count > 0;
        result.message = result.success
            ? QStringLiteral("Pinocchio 引擎工作空间采样完成。")
            : QStringLiteral("Pinocchio 引擎工作空间采样未获得有效点。");

        if (m_logger != nullptr)
        {
            m_logger->Log(
                RoboSDP::Logging::LogLevel::Info,
                result.message,
                RoboSDP::Errors::ErrorCode::Ok,
                {
                    QStringLiteral("Kinematics"),
                    QStringLiteral("SampleWorkspace"),
                    QStringLiteral("PinocchioKinematicBackendAdapter")});
        }
        return result;
    }
    catch (const std::exception& exception)
    {
        return failWithReason(QStringLiteral("Pinocchio 工作空间采样异常：%1").arg(QString::fromUtf8(exception.what())));
    }
    catch (...)
    {
        return failWithReason(QStringLiteral("Pinocchio 工作空间采样出现未知异常，已拦截。"));
    }
#endif
}

RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto
PinocchioKinematicBackendAdapter::ValidateBuildContext(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const
{
    return BuildContextInternal(model);
}

RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto
PinocchioKinematicBackendAdapter::BuildNormalizedContext(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const
{
    return BuildContextInternal(model);
}

RoboSDP::Kinematics::Dto::NativeFkDryRunResultDto
PinocchioKinematicBackendAdapter::EvaluateNativeFkDryRun(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const std::vector<double>& joint_positions_deg) const
{
    RoboSDP::Kinematics::Dto::NativeFkDryRunResultDto result;
    result.joint_positions_deg = joint_positions_deg;

    // 第一步：复用上一阶段的 build-context + native Model/Data 同步逻辑。
    const auto buildResult = BuildNormalizedContext(model);
    if (!buildResult.IsSuccess() || !buildResult.status.shared_robot_kernel_ready)
    {
        result.message = QStringLiteral("Pinocchio FK 干跑失败：共享内核尚未 ready，原因：%1")
            .arg(buildResult.status.status_message);
        return result;
    }

#if !defined(ROBOSDP_HAVE_PINOCCHIO)
    result.message = QStringLiteral("Pinocchio FK 干跑失败：当前构建未启用 Pinocchio 依赖。");
    return result;
#else
    if (m_native_kernel_state == nullptr ||
        m_native_kernel_state->model == nullptr ||
        m_native_kernel_state->data == nullptr ||
        !m_native_kernel_state->ready)
    {
        result.message = QStringLiteral("Pinocchio FK 干跑失败：原生 Model/Data 未实例化。");
        return result;
    }

    const auto expectedJointCount = static_cast<std::size_t>(m_native_kernel_state->model->nq);
    if (m_native_kernel_state->native_position_offsets_deg.size() != expectedJointCount)
    {
        result.message = QStringLiteral("Pinocchio FK 干跑失败：内部零位偏移数量=%1，但原生模型 nq=%2。")
            .arg(m_native_kernel_state->native_position_offsets_deg.size())
            .arg(expectedJointCount);
        return result;
    }

    if (joint_positions_deg.size() != expectedJointCount)
    {
        result.message = QStringLiteral("Pinocchio FK 干跑失败：输入关节角数量=%1，但原生模型 nq=%2，已拦截以避免越界。")
            .arg(joint_positions_deg.size())
            .arg(expectedJointCount);
        return result;
    }

    try
    {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(m_native_kernel_state->model->nq);
        result.joint_positions_rad.resize(joint_positions_deg.size(), 0.0);
        for (std::size_t index = 0; index < joint_positions_deg.size(); ++index)
        {
            if (!IsFiniteValue(joint_positions_deg[index]))
            {
                result.message = QStringLiteral("Pinocchio FK 干跑失败：第 %1 个关节角不是有效数值。")
                    .arg(index + 1);
                return result;
            }

            // 数学口径：UI/DTO 使用角度，Pinocchio 使用弧度；转换公式 rad = deg * pi / 180。
            // 对 DH/MDH 模型，theta_offset 是机构零位偏移，需要在下发给 Pinocchio q 前一并叠加。
            const double nativeDegrees = joint_positions_deg[index] + m_native_kernel_state->native_position_offsets_deg[index];
            const double radians = DegToRad(nativeDegrees);
            result.joint_positions_rad[index] = radians;
            q[static_cast<Eigen::Index>(index)] = radians;
        }

        pinocchio::forwardKinematics(
            *m_native_kernel_state->model,
            *m_native_kernel_state->data,
            q);
        pinocchio::updateFramePlacements(
            *m_native_kernel_state->model,
            *m_native_kernel_state->data);

        result.base_pose = BuildPoseFromSe3(
            m_native_kernel_state->data->oMf[m_native_kernel_state->base_frame_id]);
        result.flange_pose = BuildPoseFromSe3(
            m_native_kernel_state->data->oMf[m_native_kernel_state->flange_frame_id]);
        result.tcp_pose = BuildPoseFromSe3(
            m_native_kernel_state->data->oMf[m_native_kernel_state->tcp_frame_id]);
        result.success = true;
        result.message = QStringLiteral("Pinocchio FK 干跑完成：已更新 base/flange/tcp 语义 frame 位姿。");
        return result;
    }
    catch (const std::exception& exception)
    {
        result.success = false;
        result.message = QStringLiteral("Pinocchio FK 干跑异常：%1")
            .arg(QString::fromUtf8(exception.what()));
        return result;
    }
    catch (...)
    {
        result.success = false;
        result.message = QStringLiteral("Pinocchio FK 干跑出现未知异常，已拦截以避免崩溃。");
        return result;
    }
#endif
}

RoboSDP::Kinematics::Dto::NativeJacobianDryRunResultDto
PinocchioKinematicBackendAdapter::EvaluateNativeJacobianDryRun(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const std::vector<double>& joint_positions_deg) const
{
    RoboSDP::Kinematics::Dto::NativeJacobianDryRunResultDto result;
    result.rows = 6;
    result.reference_frame = QStringLiteral("LOCAL_WORLD_ALIGNED");
    result.joint_positions_deg = joint_positions_deg;

    // 第一步：复用 build-context 与原生 Model/Data 同步逻辑，确保 Jacobian 干跑看到的是同一套底层模型。
    const auto buildResult = BuildNormalizedContext(model);
    if (!buildResult.IsSuccess() || !buildResult.status.shared_robot_kernel_ready)
    {
        result.message = QStringLiteral("Pinocchio Jacobian 干跑失败：共享内核尚未 ready，原因：%1")
            .arg(buildResult.status.status_message);
        return result;
    }

#if !defined(ROBOSDP_HAVE_PINOCCHIO)
    result.message = QStringLiteral("Pinocchio Jacobian 干跑失败：当前构建未启用 Pinocchio 依赖。");
    return result;
#else
    if (m_native_kernel_state == nullptr ||
        m_native_kernel_state->model == nullptr ||
        m_native_kernel_state->data == nullptr ||
        !m_native_kernel_state->ready)
    {
        result.message = QStringLiteral("Pinocchio Jacobian 干跑失败：原生 Model/Data 未实例化。");
        return result;
    }

    const auto expectedConfigCount = static_cast<std::size_t>(m_native_kernel_state->model->nq);
    const auto expectedVelocityCount = static_cast<std::size_t>(m_native_kernel_state->model->nv);
    result.cols = static_cast<int>(expectedVelocityCount);

    if (m_native_kernel_state->native_position_offsets_deg.size() != expectedConfigCount)
    {
        result.message = QStringLiteral("Pinocchio Jacobian 干跑失败：内部零位偏移数量=%1，但原生模型 nq=%2。")
            .arg(m_native_kernel_state->native_position_offsets_deg.size())
            .arg(expectedConfigCount);
        return result;
    }

    if (joint_positions_deg.size() != expectedConfigCount)
    {
        result.message = QStringLiteral("Pinocchio Jacobian 干跑失败：输入关节角数量=%1，但原生模型 nq=%2，已拦截以避免越界。")
            .arg(joint_positions_deg.size())
            .arg(expectedConfigCount);
        return result;
    }

    // Frame ID / Joint ID 前置拦截：getFrameJacobian 内部也会检查，但这里先翻译成中文错误，避免底层断言式失败。
    if (m_native_kernel_state->tcp_frame_id >= static_cast<pinocchio::FrameIndex>(m_native_kernel_state->model->nframes))
    {
        result.message = QStringLiteral("Pinocchio Jacobian 干跑失败：TCP frame_id=%1 超出原生 frame 数量=%2。")
            .arg(m_native_kernel_state->tcp_frame_id)
            .arg(m_native_kernel_state->model->nframes);
        return result;
    }
    const auto& tcpFrame = m_native_kernel_state->model->frames[m_native_kernel_state->tcp_frame_id];
    if (tcpFrame.parentJoint >= static_cast<pinocchio::JointIndex>(m_native_kernel_state->model->njoints))
    {
        result.message = QStringLiteral("Pinocchio Jacobian 干跑失败：TCP frame 绑定的 parentJoint=%1 超出原生 joint 数量=%2。")
            .arg(tcpFrame.parentJoint)
            .arg(m_native_kernel_state->model->njoints);
        return result;
    }

    try
    {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(m_native_kernel_state->model->nq);
        result.joint_positions_rad.resize(joint_positions_deg.size(), 0.0);
        for (std::size_t index = 0; index < joint_positions_deg.size(); ++index)
        {
            if (!IsFiniteValue(joint_positions_deg[index]))
            {
                result.message = QStringLiteral("Pinocchio Jacobian 干跑失败：第 %1 个关节角不是有效数值。")
                    .arg(index + 1);
                return result;
            }

            // 数学口径：DTO 输入为角度，Pinocchio q 使用弧度；DH/MDH 模型还要叠加 theta_offset 零位偏移。
            const double nativeDegrees = joint_positions_deg[index] + m_native_kernel_state->native_position_offsets_deg[index];
            const double radians = DegToRad(nativeDegrees);
            result.joint_positions_rad[index] = radians;
            q[static_cast<Eigen::Index>(index)] = radians;
        }

        pinocchio::computeJointJacobians(
            *m_native_kernel_state->model,
            *m_native_kernel_state->data,
            q);
        pinocchio::updateFramePlacements(
            *m_native_kernel_state->model,
            *m_native_kernel_state->data);

        Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, m_native_kernel_state->model->nv);
        jacobian.setZero();

        // 参考系固定为 LOCAL_WORLD_ALIGNED：作用点位于 TCP frame 原点，速度分量投影到世界坐标轴。
        // 该口径比 LOCAL 更便于和世界系 FK 位姿及后续诊断界面做一致性阅读。
        pinocchio::getFrameJacobian(
            *m_native_kernel_state->model,
            *m_native_kernel_state->data,
            m_native_kernel_state->tcp_frame_id,
            pinocchio::LOCAL_WORLD_ALIGNED,
            jacobian);

        result.jacobian_matrix.clear();
        result.jacobian_matrix.reserve(static_cast<std::size_t>(jacobian.rows() * jacobian.cols()));
        for (Eigen::Index row = 0; row < jacobian.rows(); ++row)
        {
            for (Eigen::Index col = 0; col < jacobian.cols(); ++col)
            {
                // 对外固定行主序：matrix[row * cols + col]，让 DTO 使用方无需理解 Eigen 的列主序存储。
                result.jacobian_matrix.push_back(jacobian(row, col));
            }
        }

        result.success = true;
        result.message = QStringLiteral("Pinocchio Jacobian 干跑完成：已提取 TCP frame 的 6x%1 空间雅可比矩阵。")
            .arg(result.cols);
        return result;
    }
    catch (const std::exception& exception)
    {
        result.success = false;
        result.message = QStringLiteral("Pinocchio Jacobian 干跑异常：%1")
            .arg(QString::fromUtf8(exception.what()));
        return result;
    }
    catch (...)
    {
        result.success = false;
        result.message = QStringLiteral("Pinocchio Jacobian 干跑出现未知异常，已拦截以避免崩溃。");
        return result;
    }
#endif
}

RoboSDP::Kinematics::Dto::KinematicBackendBuildStatusDto PinocchioKinematicBackendAdapter::GetLastBuildStatus() const
{
    return m_last_build_context_result.status;
}

/**
 * @brief 用非常人性化的方式，解释为什么底层的计算核心还不能用
 */
QString PinocchioKinematicBackendAdapter::ExplainWhySharedKernelNotReady() const
{
    if (m_last_build_context_result.status.shared_robot_kernel_ready)
    {
        return QStringLiteral("共享 Pinocchio 内核已就绪。");
    }

    if (m_last_build_context_result.status.build_context_ready)
    {
        return QStringLiteral("统一 build-context 已完成，但真实 Pinocchio Model/Data 尚未构建成功，因此共享内核仍未 ready。");
    }

    if (!m_last_build_context_result.status.status_message.trimmed().isEmpty())
    {
        return m_last_build_context_result.status.status_message; // 返回最近一次具体卡在哪一步的报错
    }

    return QStringLiteral("尚未执行 build-context 校验，当前共享 Pinocchio 内核尚未 ready。");
}

/**
 * @brief 核心内部方法：对传入的模型执行全套的“洗数据”和“严查”流程
 */
RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto
PinocchioKinematicBackendAdapter::BuildContextInternal(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const
{
    RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto result;
    result.backend_id = BackendId();
    result.backend_description = BackendDescription();

    BackendBuildStatus status;
    status.error_code = RoboSDP::Errors::ErrorCode::Ok;
    status.status_code = QStringLiteral("building_context");
    status.status_message = QStringLiteral("正在归一化运动学 build-context。");

    // 1. 梳理基本建模信息和源数据类型
    const auto buildContext = BuildUnifiedRobotModelContext(model, status);
    
    // 2. 梳理并体检基础坐标系
    const auto frameMapping = BuildFrameSemanticMapping(model, status);
    
    // 3. 梳理并体检关节顺序是否发生被篡改或错乱
    const auto jointOrderMapping = BuildJointOrderMapping(model, buildContext, status);
    
    // 4. 在 build-context 三关通过后，尝试同步原生 Pinocchio Model/Data。
    const auto nativeBuildSummary = SyncNativeModel(model, buildContext, frameMapping, jointOrderMapping);

    // 5. 汇总得出结论报告
    const auto finalStatus =
        EvaluateBuildStatus(buildContext, frameMapping, jointOrderMapping, nativeBuildSummary, status);

    // 将内部安全的数据结构装箱打包成对外公开的 DTO
    result.context = ToPublicBuildContext(buildContext, frameMapping, jointOrderMapping);
    result.status = ToPublicBuildStatus(finalStatus);
    
    // 缓存这次的报告，供外部随时来查询
    m_last_build_context_result = result;
    return result;
}

/**
 * @brief 1. 清洗基本建模数据：分析你传给我的到底是一份基于 DH、MDH 还是 URDF 文件的模型
 */
PinocchioKinematicBackendAdapter::UnifiedRobotModelBuildContext
PinocchioKinematicBackendAdapter::BuildUnifiedRobotModelContext(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    BackendBuildStatus& status) const
{
    UnifiedRobotModelBuildContext context;
    // 决定后端类型。如果没有指定，系统默认写入当前唯一的 Pinocchio 后端标识。
    context.declared_backend_type = model.backend_type.trimmed().isEmpty()
        ? BackendId()
        : model.backend_type.trimmed();
    context.normalized_backend_type = BackendId();
    context.unified_robot_model_ref = model.unified_robot_model_ref.trimmed();
    context.urdf_source_path = model.urdf_source_path.trimmed();
    context.joint_count = model.joint_count;
    context.frame_semantics_version = model.frame_semantics_version;

    // 清洗大小写
    const QString declaredModelingMode = NormalizeUpperToken(model.modeling_mode);
    const QString declaredConvention = NormalizeUpperToken(model.parameter_convention);

    // 严密推断当前的“建模模式 (modeling mode)”
    if (!declaredModelingMode.isEmpty())
    {
        context.modeling_mode = declaredModelingMode;
    }
    else if (!context.urdf_source_path.isEmpty())
    {
        context.modeling_mode = QStringLiteral("URDF"); // 如果带了URDF路径，自然就是URDF模式
    }
    else if (declaredConvention == QStringLiteral("MDH"))
    {
        context.modeling_mode = QStringLiteral("MDH");
    }
    else
    {
        context.modeling_mode = QStringLiteral("DH"); // 缺省使用标准 DH
    }

    // 拦截不支持的邪门歪道模式
    if (context.modeling_mode != QStringLiteral("DH") &&
        context.modeling_mode != QStringLiteral("MDH") &&
        context.modeling_mode != QStringLiteral("URDF"))
    {
        status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        status.status_code = QStringLiteral("unsupported_modeling_mode");
        status.status_message = QStringLiteral("modeling_mode=%1 不受支持，当前仅支持 DH、MDH、URDF。")
            .arg(model.modeling_mode);
        return context;
    }

    // 处理参数约定逻辑
    if (context.modeling_mode == QStringLiteral("URDF"))
    {
        context.parameter_convention = QStringLiteral("URDF");
    }
    else
    {
        const QString normalizedConvention = declaredConvention.isEmpty()
            ? context.modeling_mode
            : declaredConvention;
            
        // 确保没有乱填
        if (normalizedConvention != QStringLiteral("DH") && normalizedConvention != QStringLiteral("MDH"))
        {
            status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
            status.status_code = QStringLiteral("unsupported_parameter_convention");
            status.status_message = QStringLiteral("parameter_convention=%1 不受支持，当前仅支持 DH 或 MDH。")
                .arg(model.parameter_convention);
            return context;
        }
        
        // 模式(Mode)和参数约定(Convention)必须保持一致，不能牛头不对马嘴
        if (normalizedConvention != context.modeling_mode)
        {
            status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
            status.status_code = QStringLiteral("modeling_parameter_mismatch");
            status.status_message = QStringLiteral("modeling_mode=%1 与 parameter_convention=%2 不一致，无法构建统一语义。")
                .arg(context.modeling_mode, normalizedConvention);
            return context;
        }
        context.parameter_convention = normalizedConvention;
    }

    // 分析“模型是怎么产生的 (Source Mode)”
    QString normalizedSourceMode = NormalizeLowerToken(model.model_source_mode);
    if (context.modeling_mode == QStringLiteral("URDF"))
    {
        if (normalizedSourceMode.isEmpty() ||
            normalizedSourceMode == QStringLiteral("manual_seed") ||
            normalizedSourceMode == QStringLiteral("topology_derived"))
        {
            // 对于 URDF，只分为真 URDF(imported) 和 空的占位符(placeholder)
            normalizedSourceMode = context.urdf_source_path.isEmpty()
                ? QStringLiteral("urdf_placeholder")
                : QStringLiteral("urdf_imported");
        }

        if (normalizedSourceMode != QStringLiteral("urdf_imported") &&
            normalizedSourceMode != QStringLiteral("urdf_placeholder"))
        {
            status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
            status.status_code = QStringLiteral("unsupported_urdf_source_mode");
            status.status_message = QStringLiteral("URDF 模式下 model_source_mode=%1 不受支持，当前仅支持 urdf_imported 或 urdf_placeholder。")
                .arg(model.model_source_mode);
            return context;
        }
    }
    else
    {
        if (normalizedSourceMode.isEmpty())
        {
            normalizedSourceMode = QStringLiteral("manual_seed");
        }

        if (normalizedSourceMode != QStringLiteral("manual_seed") &&
            normalizedSourceMode != QStringLiteral("topology_derived"))
        {
            status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
            status.status_code = QStringLiteral("unsupported_model_source_mode");
            status.status_message = QStringLiteral("%1 模式下 model_source_mode=%2 不受支持，当前仅支持 manual_seed 或 topology_derived。")
                .arg(context.modeling_mode, model.model_source_mode);
            return context;
        }
    }
    context.model_source_mode = normalizedSourceMode;
    context.is_urdf_semantic_placeholder = normalizedSourceMode == QStringLiteral("urdf_placeholder");

    // 物理结构的数量对齐安检
    if (context.joint_count <= 0)
    {
        status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        status.status_code = QStringLiteral("invalid_joint_count");
        status.status_message = QStringLiteral("joint_count 必须大于 0，当前值为 %1。")
            .arg(context.joint_count);
        return context;
    }

    if (static_cast<int>(model.links.size()) != context.joint_count)
    {
        status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        status.status_code = QStringLiteral("joint_count_links_mismatch");
        status.status_message = QStringLiteral("joint_count=%1 与 links 数量=%2 不一致，无法构建统一机器人模型。")
            .arg(context.joint_count)
            .arg(model.links.size());
        return context;
    }

    if (static_cast<int>(model.joint_limits.size()) != context.joint_count)
    {
        status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        status.status_code = QStringLiteral("joint_count_joint_limits_mismatch");
        status.status_message = QStringLiteral("joint_count=%1 与 joint_limits 数量=%2 不一致，无法构建统一机器人模型。")
            .arg(context.joint_count)
            .arg(model.joint_limits.size());
        return context;
    }

    // 检查 ID 是否存在缺失
    for (std::size_t index = 0; index < model.links.size(); ++index)
    {
        if (model.links[index].link_id.trimmed().isEmpty())
        {
            status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
            status.status_code = QStringLiteral("link_id_missing");
            status.status_message = QStringLiteral("第 %1 个 link 缺少 link_id，无法构建统一机器人模型。")
                .arg(index + 1);
            return context;
        }
    }

    for (std::size_t index = 0; index < model.joint_limits.size(); ++index)
    {
        if (model.joint_limits[index].joint_id.trimmed().isEmpty())
        {
            status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
            status.status_code = QStringLiteral("joint_id_missing");
            status.status_message = QStringLiteral("第 %1 个 joint_limit 缺少 joint_id，无法构建统一关节顺序。")
                .arg(index + 1);
            return context;
        }
    }

    // 取出现有的关节签名，如果没有就当场用顺序造一个
    context.joint_order_signature = model.joint_order_signature.trimmed().isEmpty()
        ? BuildJointOrderSignatureFromLimits(model)
        : model.joint_order_signature.trimmed();
        
    status.normalized_semantics_ready = true; // 安检第一关：通过！
    return context;
}

/**
 * @brief 2. 坐标系 (Frame) 数据安检防爆
 */
PinocchioKinematicBackendAdapter::FrameSemanticMapping
PinocchioKinematicBackendAdapter::BuildFrameSemanticMapping(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    BackendBuildStatus& status) const
{
    FrameSemanticMapping mapping;
    // 赋予具体的业务角色标识
    mapping.base_frame_role = QStringLiteral("project_world_to_robot_base");
    mapping.flange_frame_role = QStringLiteral("last_link_to_flange");
    mapping.tcp_frame_role = QStringLiteral("flange_to_tcp");

    // 如果上一关挂了，这一关就不做了
    if (status.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        return mapping;
    }

    // 定义一个 Lambda 表达式（局部函数）专门用来做位置越界排雷检查
    auto validatePose = [&status](const QString& frameName, const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose, const QString& codePrefix) {
        constexpr double kPositionLimitM = 10.0;  // 10米以上视为异常数据
        constexpr double kAngleLimitDeg = 360.0;

        for (std::size_t index = 0; index < pose.position_m.size(); ++index)
        {
            if (!IsFiniteValue(pose.position_m[index]))
            {
                status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
                status.status_code = codePrefix + QStringLiteral("_position_invalid");
                status.status_message = QStringLiteral("%1 存在非数值位置分量，无法构建统一 frame 语义。")
                    .arg(frameName);
                return false;
            }

            if (std::abs(pose.position_m[index]) > kPositionLimitM)
            {
                status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
                status.status_code = codePrefix + QStringLiteral("_position_out_of_range");
                status.status_message = QStringLiteral("%1 的位置分量超出允许范围 ±%2 m，无法构建统一 frame 语义。")
                    .arg(frameName)
                    .arg(kPositionLimitM, 0, 'f', 1);
                return false;
            }
        }

        for (std::size_t index = 0; index < pose.rpy_deg.size(); ++index)
        {
            if (!IsFiniteValue(pose.rpy_deg[index]))
            {
                status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
                status.status_code = codePrefix + QStringLiteral("_orientation_invalid");
                status.status_message = QStringLiteral("%1 存在非数值姿态分量，无法构建统一 frame 语义。")
                    .arg(frameName);
                return false;
            }

            if (std::abs(pose.rpy_deg[index]) > kAngleLimitDeg)
            {
                status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
                status.status_code = codePrefix + QStringLiteral("_orientation_out_of_range");
                status.status_message = QStringLiteral("%1 的姿态分量超出允许范围 ±%2 deg，无法构建统一 frame 语义。")
                    .arg(frameName)
                    .arg(kAngleLimitDeg, 0, 'f', 0);
                return false;
            }
        }

        return true; // 没有踩雷
    };

    if (model.frame_semantics_version < 1)
    {
        status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        status.status_code = QStringLiteral("frame_semantics_version_invalid");
        status.status_message = QStringLiteral("frame_semantics_version=%1 无效，当前最小支持版本为 1。")
            .arg(model.frame_semantics_version);
        return mapping;
    }

    // 依次排雷 Base, Flange, TCP
    if (!validatePose(QStringLiteral("Base Frame"), model.base_frame, QStringLiteral("base_frame"))) return mapping;
    if (!validatePose(QStringLiteral("Flange Frame"), model.flange_frame, QStringLiteral("flange_frame"))) return mapping;
    if (!validatePose(QStringLiteral("TCP Frame"), ToPose(model.tcp_frame), QStringLiteral("tcp_frame"))) return mapping;

    mapping.semantics_ready = true;
    status.frame_semantics_ready = true; // 安检第二关：通过！
    return mapping;
}

/**
 * @brief 3. 关节防篡改与顺序严查
 */
PinocchioKinematicBackendAdapter::JointOrderMapping
PinocchioKinematicBackendAdapter::BuildJointOrderMapping(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const UnifiedRobotModelBuildContext& buildContext,
    BackendBuildStatus& status) const
{
    JointOrderMapping mapping;
    mapping.signature = buildContext.joint_order_signature;
    mapping.expected_joint_count = buildContext.joint_count;

    if (status.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        return mapping;
    }

    if (mapping.expected_joint_count <= 0)
    {
        status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        status.status_code = QStringLiteral("joint_order_joint_count_invalid");
        status.status_message = QStringLiteral("joint_count 无效，无法构建 joint_order_signature 映射。");
        return mapping;
    }

    // 将长字符串签名 "J1|J2|J3" 拆开
    const QStringList signatureJointIds = SplitSignature(mapping.signature);
    if (signatureJointIds.size() != mapping.expected_joint_count)
    {
        status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        status.status_code = QStringLiteral("joint_order_signature_count_mismatch");
        status.status_message = QStringLiteral("joint_order_signature 中的关节数量=%1 与 joint_count=%2 不一致。")
            .arg(signatureJointIds.size())
            .arg(mapping.expected_joint_count);
        return mapping;
    }

    QSet<QString> uniqueJointIds;
    // 逐个比对拆开的签名和实际模型里数组的顺序
    for (int index = 0; index < signatureJointIds.size(); ++index)
    {
        const QString signatureJointId = signatureJointIds[index].trimmed();
        if (signatureJointId.isEmpty())
        {
            status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
            status.status_code = QStringLiteral("joint_order_signature_empty_entry");
            status.status_message = QStringLiteral("joint_order_signature 第 %1 个关节标识为空，无法构建顺序映射。")
                .arg(index + 1);
            return mapping;
        }

        if (uniqueJointIds.contains(signatureJointId))
        {
            status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
            status.status_code = QStringLiteral("joint_order_signature_duplicate");
            status.status_message = QStringLiteral("joint_order_signature 中存在重复关节标识 %1。")
                .arg(signatureJointId);
            return mapping;
        }
        uniqueJointIds.insert(signatureJointId);

        // 这是最核心的检查：签名里的顺序和实际内存数组里的顺序必须严丝合缝
        const QString modelJointId = model.joint_limits[static_cast<std::size_t>(index)].joint_id.trimmed();
        if (signatureJointId != modelJointId)
        {
            status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
            status.status_code = QStringLiteral("joint_order_signature_mismatch");
            status.status_message = QStringLiteral("joint_order_signature 第 %1 位为 %2，但 joint_limits 对应位置为 %3，顺序不一致。")
                .arg(index + 1)
                .arg(signatureJointId, modelJointId);
            return mapping;
        }

        mapping.ordered_joint_ids.push_back(signatureJointId);
    }

    mapping.order_ready = true;
    status.joint_order_ready = true; // 安检第三关：通过！
    return mapping;
}

/**
 * @brief 4. 将清洗后的上下文真正同步为 Pinocchio 原生 Model/Data。
 *
 * @details 本方法仍然只服务于诊断链路，不会通过 SolveFk/SampleWorkspace 输出真实结果。
 * 它的职责是把“能否构建共享内核”这件事做实，并把所有三方库异常翻译成中文状态。
 */
PinocchioKinematicBackendAdapter::NativeBuildSummary
PinocchioKinematicBackendAdapter::SyncNativeModel(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const UnifiedRobotModelBuildContext& buildContext,
    const FrameSemanticMapping& frameMapping,
    const JointOrderMapping& jointOrderMapping) const
{
    NativeBuildSummary summary;

    if (m_native_kernel_state == nullptr)
    {
        m_native_kernel_state = std::make_unique<NativeKernelState>();
    }

    summary.build_count = m_native_kernel_state->build_count;
    summary.native_joint_count = m_native_kernel_state->native_joint_count;
    summary.native_frame_count = m_native_kernel_state->native_frame_count;

    if (!frameMapping.semantics_ready || !jointOrderMapping.order_ready)
    {
        summary.status_code = QStringLiteral("native_build_context_incomplete");
        summary.status_message = QStringLiteral("build-context 尚未通过 frame 或 joint 顺序校验，跳过 Pinocchio 原生模型构建。");
        return summary;
    }

#if !defined(ROBOSDP_HAVE_PINOCCHIO)
    m_native_kernel_state->ready = false;
    m_native_kernel_state->last_message =
        QStringLiteral("当前构建未检测到 Pinocchio C++ 依赖，无法实例化共享 Model/Data。");
    summary.status_code = QStringLiteral("pinocchio_dependency_unavailable");
    summary.status_message = m_native_kernel_state->last_message;
    return summary;
#else
    const RoboSDP::Core::Kinematics::SharedRobotKernelRequest request {
        &model,
        nullptr,
        buildContext.unified_robot_model_ref,
        buildContext.modeling_mode,
        buildContext.joint_order_signature,
        QString(),
        true};

    const auto acquireResult =
        RoboSDP::Core::Kinematics::SharedRobotKernelRegistry::Instance().GetOrBuildKernel(request);

    summary.cache_key = acquireResult.metadata.cache_key;
    summary.status_code = acquireResult.metadata.status_code;
    summary.status_message = acquireResult.metadata.status_message;
    summary.warning_message = acquireResult.metadata.warning_message;
    summary.cache_hit = acquireResult.cache_hit;
    summary.build_count = acquireResult.metadata.registry_build_count;
    summary.native_joint_count = acquireResult.metadata.native_joint_count;
    summary.native_frame_count = acquireResult.metadata.native_frame_count;

    if (!acquireResult.success || acquireResult.model == nullptr)
    {
        m_native_kernel_state->ready = false;
        m_native_kernel_state->last_message = summary.status_message;
        m_native_kernel_state->last_warning_message = summary.warning_message;
        return summary;
    }

    if (m_native_kernel_state->ready &&
        m_native_kernel_state->cache_key == summary.cache_key &&
        m_native_kernel_state->model == acquireResult.model &&
        m_native_kernel_state->data != nullptr)
    {
        summary.success = true;
        summary.cache_hit = true;
        summary.status_code = QStringLiteral("pinocchio_shared_model_cache_hit");
        summary.status_message = QStringLiteral("共享注册表 Model 与当前 adapter Data 均命中缓存，未重复初始化。");
        return summary;
    }

    auto nativeData = std::make_unique<NativePinocchioData>(*acquireResult.model);
    m_native_kernel_state->model = acquireResult.model;
    m_native_kernel_state->data = std::move(nativeData);
    m_native_kernel_state->base_frame_id =
        static_cast<pinocchio::FrameIndex>(acquireResult.metadata.base_frame_id);
    m_native_kernel_state->flange_frame_id =
        static_cast<pinocchio::FrameIndex>(acquireResult.metadata.flange_frame_id);
    m_native_kernel_state->tcp_frame_id =
        static_cast<pinocchio::FrameIndex>(acquireResult.metadata.tcp_frame_id);
    m_native_kernel_state->native_position_offsets_deg = acquireResult.metadata.native_position_offsets_deg;
    m_native_kernel_state->link_frame_ids.clear();
    m_native_kernel_state->link_frame_ids.reserve(acquireResult.metadata.link_frame_ids.size());
    for (int frameId : acquireResult.metadata.link_frame_ids)
    {
        m_native_kernel_state->link_frame_ids.push_back(static_cast<pinocchio::FrameIndex>(frameId));
    }

    m_native_kernel_state->ready = true;
    m_native_kernel_state->cache_key = summary.cache_key;
    m_native_kernel_state->build_count = acquireResult.metadata.registry_build_count;
    m_native_kernel_state->native_joint_count = acquireResult.metadata.native_joint_count;
    m_native_kernel_state->native_frame_count = acquireResult.metadata.native_frame_count;
    m_native_kernel_state->last_message = QStringLiteral("共享注册表 Model 已接入，adapter 私有 Data 已重新初始化。");
    m_native_kernel_state->last_warning_message = summary.warning_message;
    AppendWarningMessage(m_native_kernel_state->last_message, summary.warning_message);

    if (m_logger != nullptr && !summary.warning_message.trimmed().isEmpty())
    {
        m_logger->Log(
            RoboSDP::Logging::LogLevel::Warning,
            summary.warning_message,
            RoboSDP::Errors::ErrorCode::Ok,
            {
                QStringLiteral("Kinematics"),
                QStringLiteral("SharedRobotKernelRegistry"),
                QStringLiteral("PinocchioKinematicBackendAdapter")});
    }

    summary.success = true;
    summary.status_code = acquireResult.cache_hit
        ? QStringLiteral("pinocchio_shared_model_reused")
        : QStringLiteral("pinocchio_shared_model_ready");
    summary.status_message = acquireResult.cache_hit
        ? QStringLiteral("已从 SharedRobotKernelRegistry 复用共享 Model，并重建本 adapter 私有 Data。")
        : QStringLiteral("已通过 SharedRobotKernelRegistry 构建共享 Model，并初始化本 adapter 私有 Data。");
    return summary;
#endif
}

/**
 * @brief 4. 汇总总装报告
 */
PinocchioKinematicBackendAdapter::BackendBuildStatus
PinocchioKinematicBackendAdapter::EvaluateBuildStatus(
    const UnifiedRobotModelBuildContext& buildContext,
    const FrameSemanticMapping& frameMapping,
    const JointOrderMapping& jointOrderMapping,
    const NativeBuildSummary& nativeBuildSummary,
    const BackendBuildStatus& currentStatus) const
{
    BackendBuildStatus status = currentStatus;
    
    // 检查上述三关是否全过
    status.normalized_semantics_ready =
        status.normalized_semantics_ready && !buildContext.modeling_mode.isEmpty() && !buildContext.parameter_convention.isEmpty();
    status.frame_semantics_ready = frameMapping.semantics_ready;
    status.joint_order_ready = jointOrderMapping.order_ready;
    status.build_context_ready =
        status.error_code == RoboSDP::Errors::ErrorCode::Ok &&
        status.normalized_semantics_ready &&
        status.frame_semantics_ready &&
        status.joint_order_ready;
        
    status.native_model_cache_hit = nativeBuildSummary.cache_hit;
    status.native_model_build_count = nativeBuildSummary.build_count;
    status.native_joint_count = nativeBuildSummary.native_joint_count;
    status.native_frame_count = nativeBuildSummary.native_frame_count;
    status.native_model_cache_key = nativeBuildSummary.cache_key;
    status.shared_kernel_ready = nativeBuildSummary.success;
    status.warning_message = currentStatus.warning_message;
    AppendWarningMessage(status.warning_message, nativeBuildSummary.warning_message);

    if (status.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        if (!status.warning_message.trimmed().isEmpty())
        {
            status.status_message =
                QStringLiteral("%1；预警：%2").arg(status.status_message, status.warning_message);
        }
        return status; // 如果出错了，保持当前的错误信息原路返回
    }

    if (!status.build_context_ready)
    {
        status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        status.status_code = QStringLiteral("build_context_incomplete");
        status.status_message = QStringLiteral("统一 build-context 尚未准备完成，当前不能进入 Pinocchio 共享内核构建。");
        if (!status.warning_message.trimmed().isEmpty())
        {
            status.status_message =
                QStringLiteral("%1；预警：%2").arg(status.status_message, status.warning_message);
        }
        return status;
    }

    if (!nativeBuildSummary.success)
    {
        status.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        status.status_code = nativeBuildSummary.status_code.trimmed().isEmpty()
            ? QStringLiteral("pinocchio_native_build_failed")
            : nativeBuildSummary.status_code;
        status.status_message = nativeBuildSummary.status_message.trimmed().isEmpty()
            ? QStringLiteral("build-context 已完成，但 Pinocchio Model/Data 构建失败。")
            : nativeBuildSummary.status_message;
        if (!status.warning_message.trimmed().isEmpty())
        {
            status.status_message =
                QStringLiteral("%1；预警：%2").arg(status.status_message, status.warning_message);
        }
        return status;
    }

    status.error_code = RoboSDP::Errors::ErrorCode::Ok;
    status.status_code = nativeBuildSummary.cache_hit
        ? QStringLiteral("pinocchio_native_model_cache_hit")
        : QStringLiteral("pinocchio_native_model_ready");
    status.status_message = nativeBuildSummary.status_message;
    if (!status.warning_message.trimmed().isEmpty())
    {
        status.status_message =
            QStringLiteral("%1；预警：%2").arg(status.status_message, status.warning_message);
    }
    return status;
}

/**
 * @brief 将内部数据转为 DTO 结构返回
 */
RoboSDP::Kinematics::Dto::KinematicBackendBuildContextDto
PinocchioKinematicBackendAdapter::ToPublicBuildContext(
    const UnifiedRobotModelBuildContext& buildContext,
    const FrameSemanticMapping& frameMapping,
    const JointOrderMapping& jointOrderMapping) const
{
    RoboSDP::Kinematics::Dto::KinematicBackendBuildContextDto contextDto;
    // 拷贝基础信息
    contextDto.normalized_model.modeling_mode = buildContext.modeling_mode;
    contextDto.normalized_model.parameter_convention = buildContext.parameter_convention;
    contextDto.normalized_model.declared_backend_type = buildContext.declared_backend_type;
    contextDto.normalized_model.normalized_backend_type = buildContext.normalized_backend_type;
    contextDto.normalized_model.model_source_mode = buildContext.model_source_mode;
    contextDto.normalized_model.unified_robot_model_ref = buildContext.unified_robot_model_ref;
    contextDto.normalized_model.joint_count = buildContext.joint_count;
    contextDto.normalized_model.joint_order_signature = buildContext.joint_order_signature;
    contextDto.normalized_model.urdf_source_path = buildContext.urdf_source_path;
    contextDto.normalized_model.frame_semantics_version = buildContext.frame_semantics_version;
    contextDto.normalized_model.is_urdf_semantic_placeholder = buildContext.is_urdf_semantic_placeholder;

    // 拷贝坐标系角色
    contextDto.frame_mapping.base_frame_role = frameMapping.base_frame_role;
    contextDto.frame_mapping.flange_frame_role = frameMapping.flange_frame_role;
    contextDto.frame_mapping.tcp_frame_role = frameMapping.tcp_frame_role;
    contextDto.frame_mapping.semantics_complete = frameMapping.semantics_ready;

    // 拷贝关节映射
    contextDto.joint_order_mapping.signature = jointOrderMapping.signature;
    contextDto.joint_order_mapping.expected_joint_count = jointOrderMapping.expected_joint_count;
    contextDto.joint_order_mapping.mapping_complete = jointOrderMapping.order_ready;
    contextDto.joint_order_mapping.ordered_joint_ids = jointOrderMapping.ordered_joint_ids;
    return contextDto;
}

/**
 * @brief 将内部状态转为公开的状态 DTO
 */
RoboSDP::Kinematics::Dto::KinematicBackendBuildStatusDto
PinocchioKinematicBackendAdapter::ToPublicBuildStatus(const BackendBuildStatus& status) const
{
    RoboSDP::Kinematics::Dto::KinematicBackendBuildStatusDto statusDto;
    statusDto.error_code = status.error_code;
    statusDto.status_code = status.status_code;
    statusDto.status_message = status.status_message;
    statusDto.normalized_semantics_ready = status.normalized_semantics_ready;
    statusDto.frame_semantics_ready = status.frame_semantics_ready;
    statusDto.joint_order_ready = status.joint_order_ready;
    statusDto.build_context_ready = status.build_context_ready;
    statusDto.shared_robot_kernel_ready = status.shared_kernel_ready;
    statusDto.native_model_cache_hit = status.native_model_cache_hit;
    statusDto.native_model_build_count = status.native_model_build_count;
    statusDto.native_joint_count = status.native_joint_count;
    statusDto.native_frame_count = status.native_frame_count;
    statusDto.native_model_cache_key = status.native_model_cache_key;
    return statusDto;
}

/**
 * @brief 在系统日志中写下当前发生的事情
 */
void PinocchioKinematicBackendAdapter::LogStubInvocation(
    const QString& actionName,
    const QString& message) const
{
    if (m_logger == nullptr)
    {
        return;
    }

    m_logger->Log(
        RoboSDP::Logging::LogLevel::Info,
        message,
        RoboSDP::Errors::ErrorCode::Ok,
        {
            QStringLiteral("Kinematics"),
            actionName,
            QStringLiteral("PinocchioKinematicBackendAdapter")});
}

} // namespace RoboSDP::Kinematics::Adapter
