#include "core/kinematics/SharedRobotKernelRegistry.h"

#include "modules/dynamics/dto/DynamicModelDto.h"
#include "modules/kinematics/dto/KinematicModelDto.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QSet>
#include <QStringList>
#include <QUrl>
#include <QXmlStreamReader>

#include <cmath>
#include <mutex>
#include <optional>
#include <stdexcept>

#if defined(ROBOSDP_HAVE_PINOCCHIO)
#include <Eigen/Geometry>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/frame.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/spatial/se3.hpp>
#endif

namespace RoboSDP::Core::Kinematics
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

struct MinimalUrdfJointInput
{
    QString joint_id;
    QString joint_type;
    QString parent_link_name;
    QString child_link_name;
    int parent_link_index = -1;
    int child_link_index = -1;
    std::array<double, 3> xyz {0.0, 0.0, 0.0};
    std::array<double, 3> rpy_deg {0.0, 0.0, 0.0};
};

struct MinimalUrdfLinkInput
{
    QString link_id;
    int parent_joint_index = -1;
    std::vector<int> child_joint_indices;
};

struct MinimalUrdfGeometryInput
{
    QString link_name;
    QString geometry_type;
    QString raw_resource_path;
    QString absolute_file_path;
    RoboSDP::Kinematics::Dto::CartesianPoseDto local_pose;
    std::array<double, 3> scale {1.0, 1.0, 1.0};
    bool resource_available = true;
    QString status_message;
};

struct MinimalUrdfModelInput
{
    QString robot_name;
    std::vector<MinimalUrdfLinkInput> links;
    std::vector<MinimalUrdfJointInput> joints;
    std::vector<MinimalUrdfGeometryInput> visual_geometries;
    std::vector<MinimalUrdfGeometryInput> collision_geometries;
    QStringList root_link_names;
    QString warning_message;
};

struct MinimalUrdfTrunkInput
{
    std::vector<int> ordered_joint_indices;
    int movable_joint_count = 0;
    QString warning_message;
};

QString NormalizeLowerToken(const QString& value)
{
    return value.trimmed().toLower();
}

bool IsFiniteValue(double value)
{
    return std::isfinite(value);
}

double DegToRad(double degrees)
{
    return degrees * kPi / 180.0;
}

QString NormalizeAbsoluteFilePath(const QString& filePath)
{
    return QDir::fromNativeSeparators(QFileInfo(filePath).absoluteFilePath());
}

void AppendDoubleToKey(QString& key, double value)
{
    key.append(QStringLiteral("|%1").arg(value, 0, 'g', 17));
}

void AppendWarningMessage(QString& target, const QString& warning)
{
    const QString normalized = warning.trimmed();
    if (normalized.isEmpty())
    {
        return;
    }

    if (target.trimmed().isEmpty())
    {
        target = normalized;
        return;
    }

    if (!target.contains(normalized))
    {
        target.append(QStringLiteral("；")).append(normalized);
    }
}

QStringList BuildMeshSearchDirectories(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    QStringList directories;
    for (const QString& searchDirectory : model.mesh_search_directories)
    {
        const QString normalizedDirectory = searchDirectory.trimmed();
        if (!normalizedDirectory.isEmpty())
        {
            directories.push_back(NormalizeAbsoluteFilePath(normalizedDirectory));
        }
    }

    const QFileInfo urdfFileInfo(model.urdf_source_path);
    if (urdfFileInfo.exists())
    {
        directories.push_back(NormalizeAbsoluteFilePath(urdfFileInfo.absolutePath()));

        const QDir urdfDirectory(urdfFileInfo.absolutePath());
        const QString parentDirectory = NormalizeAbsoluteFilePath(urdfDirectory.absoluteFilePath(QStringLiteral("..")));
        if (parentDirectory != NormalizeAbsoluteFilePath(urdfDirectory.absolutePath()))
        {
            directories.push_back(parentDirectory);
        }
    }

    directories.removeDuplicates();
    return directories;
}

std::array<double, 3> ParseOptionalTriple(
    const QString& value,
    const QString& fieldName,
    const std::array<double, 3>& defaultValue)
{
    if (value.trimmed().isEmpty())
    {
        return defaultValue;
    }

    std::array<double, 3> result = defaultValue;
    const QStringList tokens = value.split(QLatin1Char(' '), Qt::SkipEmptyParts);
    if (tokens.size() != 3)
    {
        throw std::runtime_error(QStringLiteral("URDF 字段 %1 必须包含 3 个数值。").arg(fieldName).toUtf8().constData());
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

struct ResolvedMeshPath
{
    QString absolute_file_path;
    bool resource_available = false;
    QString warning_message;
};

ResolvedMeshPath ResolveMeshFilePath(
    const QString& rawPath,
    const QString& urdfAbsolutePath,
    const QStringList& meshSearchDirectories)
{
    ResolvedMeshPath result;
    const QString normalizedRawPath = rawPath.trimmed();
    if (normalizedRawPath.isEmpty())
    {
        result.warning_message = QStringLiteral("URDF mesh 节点缺少 filename/url，已跳过皮肤资源加载。");
        return result;
    }

    auto tryResolveExistingFile =
        [](const QString& candidatePath, ResolvedMeshPath& resolvedResult) -> bool
    {
        const QFileInfo candidateInfo(candidatePath);
        if (candidateInfo.exists() && candidateInfo.isFile())
        {
            resolvedResult.absolute_file_path = NormalizeAbsoluteFilePath(candidateInfo.absoluteFilePath());
            resolvedResult.resource_available = true;
            return true;
        }

        resolvedResult.absolute_file_path = NormalizeAbsoluteFilePath(candidateInfo.absoluteFilePath());
        return false;
    };

    if (normalizedRawPath.startsWith(QStringLiteral("file://"), Qt::CaseInsensitive))
    {
        QString localFile = QUrl(normalizedRawPath).toLocalFile();
        if (localFile.trimmed().isEmpty())
        {
            localFile = normalizedRawPath.mid(QStringLiteral("file://").size());
            if (localFile.startsWith(QLatin1Char('/')) && localFile.size() > 2 && localFile[2] == QLatin1Char(':'))
            {
                localFile.remove(0, 1);
            }
        }

        if (!tryResolveExistingFile(localFile, result))
        {
            result.warning_message = QStringLiteral("URDF mesh 文件不存在：%1").arg(result.absolute_file_path);
        }
        return result;
    }

    if (QFileInfo(normalizedRawPath).isAbsolute())
    {
        if (!tryResolveExistingFile(normalizedRawPath, result))
        {
            result.warning_message = QStringLiteral("URDF mesh 文件不存在：%1").arg(result.absolute_file_path);
        }
        return result;
    }

    if (normalizedRawPath.startsWith(QStringLiteral("package://"), Qt::CaseInsensitive))
    {
        const QString packageRelativePath = normalizedRawPath.mid(QStringLiteral("package://").size());
        const QStringList pathTokens = packageRelativePath.split(QLatin1Char('/'), Qt::SkipEmptyParts);
        if (pathTokens.isEmpty())
        {
            result.warning_message = QStringLiteral("package:// mesh 路径格式非法：%1").arg(normalizedRawPath);
            return result;
        }

        const QString packageName = pathTokens.front();
        const QString relativeInsidePackage = pathTokens.mid(1).join(QStringLiteral("/"));
        QStringList candidatePaths;
        for (const QString& searchDirectory : meshSearchDirectories)
        {
            const QFileInfo searchInfo(searchDirectory);
            candidatePaths.push_back(QDir(searchDirectory).absoluteFilePath(packageRelativePath));
            if (!relativeInsidePackage.trimmed().isEmpty() &&
                searchInfo.fileName().compare(packageName, Qt::CaseInsensitive) == 0)
            {
                candidatePaths.push_back(QDir(searchDirectory).absoluteFilePath(relativeInsidePackage));
            }
        }

        candidatePaths.removeDuplicates();
        for (const QString& candidatePath : candidatePaths)
        {
            if (tryResolveExistingFile(candidatePath, result))
            {
                return result;
            }
        }

        result.warning_message = QStringLiteral(
            "package:// mesh 解析失败：未在搜索目录中找到 %1。").arg(packageRelativePath);
        return result;
    }

    const QFileInfo urdfFileInfo(urdfAbsolutePath);
    const QString relativeCandidatePath =
        QDir(urdfFileInfo.absolutePath()).absoluteFilePath(normalizedRawPath);
    if (!tryResolveExistingFile(relativeCandidatePath, result))
    {
        result.warning_message = QStringLiteral("URDF 相对 mesh 文件不存在：%1").arg(result.absolute_file_path);
    }
    return result;
}

QString BuildStructuralKey(const SharedRobotKernelRequest& request)
{
    QString unifiedRef = request.unified_robot_model_ref.trimmed();
    if (unifiedRef.isEmpty() && request.kinematic_model != nullptr)
    {
        unifiedRef = request.kinematic_model->unified_robot_model_ref.trimmed();
        if (unifiedRef.isEmpty())
        {
            unifiedRef = request.kinematic_model->meta.kinematic_id.trimmed();
        }
    }

    return QStringLiteral("%1|%2|%3")
        .arg(unifiedRef, request.modeling_mode.trimmed(), request.joint_order_signature.trimmed());
}

QString BuildInertialHash(const SharedRobotKernelRequest& request)
{
    if (!request.inertial_hash.trimmed().isEmpty())
    {
        return request.inertial_hash.trimmed();
    }

    if (request.dynamic_model == nullptr)
    {
        return QStringLiteral("placeholder_inertia");
    }

    QString hash = QStringLiteral("gravity");
    for (double gravityValue : request.dynamic_model->gravity)
    {
        AppendDoubleToKey(hash, gravityValue);
    }

    for (const auto& link : request.dynamic_model->links)
    {
        hash.append(QStringLiteral("|link:")).append(link.link_id.trimmed());
        AppendDoubleToKey(hash, link.mass);
        for (double value : link.cog) AppendDoubleToKey(hash, value);
        for (double value : link.inertia_tensor) AppendDoubleToKey(hash, value);
    }

    hash.append(QStringLiteral("|tool"));
    AppendDoubleToKey(hash, request.dynamic_model->end_effector.mass);
    for (double value : request.dynamic_model->end_effector.cog) AppendDoubleToKey(hash, value);
    for (double value : request.dynamic_model->end_effector.inertia_tensor) AppendDoubleToKey(hash, value);
    return hash;
}

QString BuildFullCacheKey(const QString& structuralKey, const QString& inertialHash)
{
    return structuralKey + QStringLiteral("|") + inertialHash;
}

QHash<QString, SharedRobotKernelAcquireResult>& KernelCache()
{
    static QHash<QString, SharedRobotKernelAcquireResult> cache;
    return cache;
}

QHash<QString, QString>& StructuralAliasCache()
{
    static QHash<QString, QString> cache;
    return cache;
}

std::mutex& RegistryMutex()
{
    static std::mutex mutex;
    return mutex;
}

int& RegistryBuildCounter()
{
    static int counter = 0;
    return counter;
}

#if defined(ROBOSDP_HAVE_PINOCCHIO)
pinocchio::SE3 BuildSe3FromPose(const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    const Eigen::AngleAxisd roll(DegToRad(pose.rpy_deg[0]), Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(DegToRad(pose.rpy_deg[1]), Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(DegToRad(pose.rpy_deg[2]), Eigen::Vector3d::UnitZ());
    return pinocchio::SE3(
        (yaw * pitch * roll).toRotationMatrix(),
        Eigen::Vector3d(pose.position_m[0], pose.position_m[1], pose.position_m[2]));
}

pinocchio::SE3 BuildSe3FromTcp(const RoboSDP::Kinematics::Dto::TcpFrameDto& tcpFrame)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = tcpFrame.translation_m;
    pose.rpy_deg = tcpFrame.rpy_deg;
    return BuildSe3FromPose(pose);
}

pinocchio::SE3 BuildStandardDhPostJointPlacement(const RoboSDP::Kinematics::Dto::KinematicLinkParameterDto& link)
{
    const double alpha = DegToRad(link.alpha);
    return pinocchio::SE3(
        Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()).toRotationMatrix(),
        Eigen::Vector3d(link.a, 0.0, link.d));
}

pinocchio::SE3 BuildModifiedDhPreJointPlacement(const RoboSDP::Kinematics::Dto::KinematicLinkParameterDto& link)
{
    const double alpha = DegToRad(link.alpha);
    return pinocchio::SE3(
        Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()).toRotationMatrix(),
        Eigen::Vector3d(link.a, 0.0, 0.0));
}

pinocchio::SE3 BuildModifiedDhPostJointPlacement(const RoboSDP::Kinematics::Dto::KinematicLinkParameterDto& link)
{
    return pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, link.d));
}

Eigen::Matrix3d BuildRotationalInertia(const std::array<double, 6>& inertiaTensor)
{
    Eigen::Matrix3d rotationalInertia = Eigen::Matrix3d::Zero();
    rotationalInertia(0, 0) = std::max(1.0e-4, std::abs(inertiaTensor[0]));
    rotationalInertia(1, 1) = std::max(1.0e-4, std::abs(inertiaTensor[1]));
    rotationalInertia(2, 2) = std::max(1.0e-4, std::abs(inertiaTensor[2]));
    rotationalInertia(0, 1) = rotationalInertia(1, 0) = inertiaTensor[3];
    rotationalInertia(0, 2) = rotationalInertia(2, 0) = inertiaTensor[4];
    rotationalInertia(1, 2) = rotationalInertia(2, 1) = inertiaTensor[5];
    return rotationalInertia;
}

pinocchio::Inertia BuildPlaceholderInertia()
{
    // TODO(stage11-shared-kernel): 后续应从统一的真实惯量源读取占位分支的质量/质心/惯量。
    return pinocchio::Inertia(
        1.0,
        Eigen::Vector3d::Zero(),
        Eigen::Matrix3d::Identity() * 1.0e-4);
}

pinocchio::Inertia BuildLinkInertia(
    const RoboSDP::Dynamics::Dto::DynamicLinkDto& link,
    bool& usedPlaceholder)
{
    const double safeMass = link.mass > 1.0e-6 ? link.mass : 1.0;
    usedPlaceholder = usedPlaceholder || link.mass <= 1.0e-6;
    return pinocchio::Inertia(
        safeMass,
        Eigen::Vector3d(link.cog[0], link.cog[1], link.cog[2]),
        BuildRotationalInertia(link.inertia_tensor));
}

pinocchio::Inertia BuildToolInertia(
    const RoboSDP::Dynamics::Dto::EndEffectorDto& tool,
    bool& usedPlaceholder)
{
    const double safeMass = tool.mass > 1.0e-6 ? tool.mass : 1.0;
    usedPlaceholder = usedPlaceholder || tool.mass <= 1.0e-6;
    return pinocchio::Inertia(
        safeMass,
        Eigen::Vector3d(tool.cog[0], tool.cog[1], tool.cog[2]),
        BuildRotationalInertia(tool.inertia_tensor));
}

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
        throw std::runtime_error(QStringLiteral("URDF 字段 %1 必须包含 3 个数值。").arg(fieldName).toUtf8().constData());
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

RoboSDP::Kinematics::Dto::GeometryObjectDto ConvertGeometryObjectDto(const MinimalUrdfGeometryInput& geometry)
{
    RoboSDP::Kinematics::Dto::GeometryObjectDto dto;
    dto.link_name = geometry.link_name;
    dto.geometry_type = geometry.geometry_type;
    dto.absolute_file_path = geometry.absolute_file_path;
    dto.local_pose = geometry.local_pose;
    dto.scale = geometry.scale;
    dto.resource_available = geometry.resource_available;
    dto.status_message = geometry.status_message;
    return dto;
}

MinimalUrdfModelInput ReadMinimalUrdfModel(const RoboSDP::Kinematics::Dto::KinematicModelDto& kinematicModel)
{
    const QString urdfPath = kinematicModel.urdf_source_path;
    QFile file(urdfPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        throw std::runtime_error(QStringLiteral("URDF 解析失败：无法打开文件 %1。").arg(urdfPath).toUtf8().constData());
    }

    MinimalUrdfModelInput model;
    QXmlStreamReader reader(&file);
    const QStringList meshSearchDirectories = BuildMeshSearchDirectories(kinematicModel);
    bool insideJoint = false;
    int visualDepth = 0;
    int collisionDepth = 0;
    QString currentLinkName;
    MinimalUrdfJointInput currentJoint;
    std::optional<MinimalUrdfGeometryInput> currentGeometry;
    QHash<QString, int> linkIndexByName;

    auto ensureLinkIndex = [&model, &linkIndexByName](const QString& linkName) -> int
    {
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
                currentLinkName = reader.attributes().value(QStringLiteral("name")).toString().trimmed();
                ensureLinkIndex(currentLinkName);
            }
            else if (elementName == QStringLiteral("joint"))
            {
                insideJoint = true;
                currentJoint = {};
                currentJoint.joint_id = reader.attributes().value(QStringLiteral("name")).toString().trimmed();
                currentJoint.joint_type = NormalizeLowerToken(reader.attributes().value(QStringLiteral("type")).toString());
            }
            else if (insideJoint && elementName == QStringLiteral("parent"))
            {
                currentJoint.parent_link_name = reader.attributes().value(QStringLiteral("link")).toString().trimmed();
            }
            else if (insideJoint && elementName == QStringLiteral("child"))
            {
                currentJoint.child_link_name = reader.attributes().value(QStringLiteral("link")).toString().trimmed();
            }
            else if (insideJoint && elementName == QStringLiteral("origin"))
            {
                currentJoint.xyz = ParseUrdfTriple(reader.attributes().value(QStringLiteral("xyz")).toString(), QStringLiteral("origin.xyz"));
                const auto rpyRad = ParseUrdfTriple(reader.attributes().value(QStringLiteral("rpy")).toString(), QStringLiteral("origin.rpy"));
                currentJoint.rpy_deg = {
                    rpyRad[0] * 180.0 / kPi,
                    rpyRad[1] * 180.0 / kPi,
                    rpyRad[2] * 180.0 / kPi};
            }
            else if (elementName == QStringLiteral("visual"))
            {
                ++visualDepth;
                if (!insideJoint && visualDepth == 1)
                {
                    currentGeometry = MinimalUrdfGeometryInput {};
                    currentGeometry->link_name = currentLinkName;
                    currentGeometry->geometry_type = QStringLiteral("unknown");
                }
            }
            else if (elementName == QStringLiteral("collision"))
            {
                ++collisionDepth;
                if (!insideJoint && collisionDepth == 1)
                {
                    currentGeometry = MinimalUrdfGeometryInput {};
                    currentGeometry->link_name = currentLinkName;
                    currentGeometry->geometry_type = QStringLiteral("unknown");
                }
            }
            else if (!insideJoint && currentGeometry.has_value() && elementName == QStringLiteral("origin"))
            {
                currentGeometry->local_pose.position_m = ParseOptionalTriple(
                    reader.attributes().value(QStringLiteral("xyz")).toString(),
                    QStringLiteral("visual.origin.xyz"),
                    {0.0, 0.0, 0.0});
                const auto rpyRad = ParseOptionalTriple(
                    reader.attributes().value(QStringLiteral("rpy")).toString(),
                    QStringLiteral("visual.origin.rpy"),
                    {0.0, 0.0, 0.0});
                currentGeometry->local_pose.rpy_deg = {
                    rpyRad[0] * 180.0 / kPi,
                    rpyRad[1] * 180.0 / kPi,
                    rpyRad[2] * 180.0 / kPi};
            }
            else if (!insideJoint && currentGeometry.has_value() && elementName == QStringLiteral("mesh"))
            {
                currentGeometry->geometry_type = QStringLiteral("mesh");
                currentGeometry->raw_resource_path =
                    reader.attributes().value(QStringLiteral("filename")).toString().trimmed();
                if (currentGeometry->raw_resource_path.isEmpty())
                {
                    currentGeometry->raw_resource_path =
                        reader.attributes().value(QStringLiteral("url")).toString().trimmed();
                }
                currentGeometry->scale = ParseOptionalTriple(
                    reader.attributes().value(QStringLiteral("scale")).toString(),
                    QStringLiteral("mesh.scale"),
                    {1.0, 1.0, 1.0});
                const auto resolvedMesh = ResolveMeshFilePath(
                    currentGeometry->raw_resource_path,
                    urdfPath,
                    meshSearchDirectories);
                currentGeometry->absolute_file_path = resolvedMesh.absolute_file_path;
                currentGeometry->resource_available = resolvedMesh.resource_available;
                currentGeometry->status_message = resolvedMesh.warning_message;
                if (!resolvedMesh.warning_message.trimmed().isEmpty())
                {
                    AppendWarningMessage(model.warning_message, resolvedMesh.warning_message);
                }
                const QString normalizedMeshPath = NormalizeLowerToken(currentGeometry->raw_resource_path);
                const bool isStlMesh = normalizedMeshPath.endsWith(QStringLiteral(".stl"));
                const bool isKnownMesh =
                    normalizedMeshPath.startsWith(QStringLiteral("package://")) ||
                    normalizedMeshPath.endsWith(QStringLiteral(".stl")) ||
                    normalizedMeshPath.endsWith(QStringLiteral(".dae")) ||
                    normalizedMeshPath.endsWith(QStringLiteral(".obj")) ||
                    normalizedMeshPath.endsWith(QStringLiteral(".ply"));
                if (isKnownMesh && visualDepth > 0 && !isStlMesh)
                {
                    // 中文说明：第 21 阶段已经支持可用 STL 的静态渲染，因此不要再对 STL visual 输出旧的“完全不支持”误导提示。
                    AppendWarningMessage(
                        model.warning_message,
                        QStringLiteral("当前 VTK 静态渲染 MVP 仅支持 STL visual mesh；DAE/OBJ/PLY 等格式已解析为元数据，但暂不渲染。"));
                }
                if (isKnownMesh && collisionDepth > 0)
                {
                    // 中文说明：collision mesh 当前只作为后续 HPP-FCL 接入的元数据，尚不参与实时碰撞计算。
                    AppendWarningMessage(
                        model.warning_message,
                        QStringLiteral("URDF collision mesh 已解析为元数据，但碰撞计算尚未接入，将在后续 HPP-FCL 阶段启用。"));
                }
            }
            else if (!insideJoint && currentGeometry.has_value() && elementName == QStringLiteral("box"))
            {
                currentGeometry->geometry_type = QStringLiteral("box");
            }
            else if (!insideJoint && currentGeometry.has_value() && elementName == QStringLiteral("cylinder"))
            {
                currentGeometry->geometry_type = QStringLiteral("cylinder");
            }
            else if (!insideJoint && currentGeometry.has_value() && elementName == QStringLiteral("sphere"))
            {
                currentGeometry->geometry_type = QStringLiteral("sphere");
            }
        }
        else if (reader.isEndElement())
        {
            const QString elementName = reader.name().toString();
            if (elementName == QStringLiteral("joint"))
            {
                if (currentJoint.parent_link_name.trimmed().isEmpty() || currentJoint.child_link_name.trimmed().isEmpty())
                {
                    throw std::runtime_error(QStringLiteral("URDF joint=%1 缺少 parent/child link 定义。").arg(currentJoint.joint_id).toUtf8().constData());
                }

                currentJoint.parent_link_index = ensureLinkIndex(currentJoint.parent_link_name);
                currentJoint.child_link_index = ensureLinkIndex(currentJoint.child_link_name);
                const int jointIndex = static_cast<int>(model.joints.size());
                model.joints.push_back(currentJoint);
                model.links[static_cast<std::size_t>(currentJoint.child_link_index)].parent_joint_index = jointIndex;
                model.links[static_cast<std::size_t>(currentJoint.parent_link_index)].child_joint_indices.push_back(jointIndex);
                insideJoint = false;
            }
            else if (elementName == QStringLiteral("link"))
            {
                currentLinkName.clear();
            }
            else if (elementName == QStringLiteral("visual") && visualDepth > 0)
            {
                if (visualDepth == 1 && currentGeometry.has_value())
                {
                    model.visual_geometries.push_back(*currentGeometry);
                    currentGeometry.reset();
                }
                --visualDepth;
            }
            else if (elementName == QStringLiteral("collision") && collisionDepth > 0)
            {
                if (collisionDepth == 1 && currentGeometry.has_value())
                {
                    model.collision_geometries.push_back(*currentGeometry);
                    currentGeometry.reset();
                }
                --collisionDepth;
            }
        }
    }

    if (reader.hasError())
    {
        throw std::runtime_error(QStringLiteral("URDF XML 解析失败：%1。").arg(reader.errorString()).toUtf8().constData());
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

MinimalUrdfTrunkInput ExtractMinimalUrdfTrunk(const MinimalUrdfModelInput& model)
{
    struct PathCandidate
    {
        std::vector<int> joint_indices;
        int movable_joint_count = 0;
        int total_joint_count = 0;
        QString signature;
    };

    auto buildSignature = [&model](const std::vector<int>& jointIndices)
    {
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

    auto chooseBetterCandidate = [](const PathCandidate& left, const PathCandidate& right)
    {
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

    std::function<PathCandidate(int, QSet<int>&)> visitLink =
        [&](int linkIndex, QSet<int>& visiting) -> PathCandidate
    {
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
                ((joint.joint_type == QStringLiteral("revolute") || joint.joint_type == QStringLiteral("continuous")) ? 1 : 0) +
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
        int rootLinkIndex = -1;
        for (std::size_t index = 0; index < model.links.size(); ++index)
        {
            if (model.links[index].link_id == rootLinkName)
            {
                rootLinkIndex = static_cast<int>(index);
                break;
            }
        }

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
        AppendWarningMessage(trunk.warning_message, QStringLiteral("检测到多个根 link，当前仅按可动关节最多的主干提取骨架。"));
    }

    for (const auto& link : model.links)
    {
        if (link.child_joint_indices.size() > 1)
        {
            AppendWarningMessage(trunk.warning_message, QStringLiteral("检测到 URDF 多分支树结构，当前仅提取可动关节最多的主干用于共享内核。"));
            break;
        }
    }
    return trunk;
}

pinocchio::SE3 BuildUrdfJointPlacement(const MinimalUrdfJointInput& joint)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = joint.xyz;
    pose.rpy_deg = joint.rpy_deg;
    return BuildSe3FromPose(pose);
}

SharedRobotKernelAcquireResult BuildKernel(
    const SharedRobotKernelRequest& request,
    const QString& structuralKey,
    const QString& inertialHash,
    const QString& cacheKey)
{
    SharedRobotKernelAcquireResult result;
    result.metadata.structural_key = structuralKey;
    result.metadata.inertial_hash = inertialHash;
    result.metadata.cache_key = cacheKey;
    result.metadata.status_code = QStringLiteral("shared_kernel_building");

    if (request.kinematic_model == nullptr)
    {
        result.metadata.status_code = QStringLiteral("shared_kernel_missing_kinematic_model");
        result.metadata.status_message = QStringLiteral("共享机器人内核构建失败：缺少 KinematicModelDto。");
        return result;
    }

    const auto& model = *request.kinematic_model;
    const QString modelingMode = request.modeling_mode.trimmed().toUpper();
    if (modelingMode != QStringLiteral("DH") &&
        modelingMode != QStringLiteral("MDH") &&
        modelingMode != QStringLiteral("URDF"))
    {
        result.metadata.status_code = QStringLiteral("shared_kernel_modeling_mode_unsupported");
        result.metadata.status_message = QStringLiteral("共享机器人内核暂不支持 modeling_mode=%1。").arg(request.modeling_mode);
        return result;
    }

    auto nativeModel = std::make_shared<SharedPinocchioModel>();
    std::vector<int> linkFrameIds;
    std::vector<SharedRobotPreviewNodeMetadata> previewNodes;
    std::vector<SharedRobotPreviewSegmentMetadata> previewSegments;
    std::vector<double> nativePositionOffsetsDeg;
    pinocchio::SE3 terminalToolPlacement = pinocchio::SE3::Identity();
    bool usedPlaceholderInertia = request.dynamic_model == nullptr;
    result.metadata.model_name = model.meta.name.trimmed();

    if (request.dynamic_model != nullptr)
    {
        nativeModel->gravity = pinocchio::Motion(
            Eigen::Vector3d(
                request.dynamic_model->gravity[0],
                request.dynamic_model->gravity[1],
                request.dynamic_model->gravity[2]),
            Eigen::Vector3d::Zero());
    }

    try
    {
        if (modelingMode == QStringLiteral("URDF"))
        {
            const QFileInfo urdfFile(model.urdf_source_path);
            if (!urdfFile.exists() || !urdfFile.isFile())
            {
                result.metadata.status_code = QStringLiteral("shared_kernel_urdf_file_not_found");
                result.metadata.status_message = QStringLiteral("共享机器人内核构建失败：找不到 URDF 文件 %1。").arg(model.urdf_source_path);
                return result;
            }

            const auto urdfModel = ReadMinimalUrdfModel(model);
            const auto urdfTrunk = ExtractMinimalUrdfTrunk(urdfModel);
            result.metadata.model_name =
                urdfModel.robot_name.trimmed().isEmpty() ? urdfFile.completeBaseName() : urdfModel.robot_name.trimmed();
            nativeModel->name = result.metadata.model_name.toStdString();
            result.metadata.warning_message = urdfModel.warning_message;
            for (const auto& geometry : urdfModel.visual_geometries)
            {
                result.metadata.visual_geometries.push_back(ConvertGeometryObjectDto(geometry));
            }
            for (const auto& geometry : urdfModel.collision_geometries)
            {
                result.metadata.collision_geometries.push_back(ConvertGeometryObjectDto(geometry));
            }
            AppendWarningMessage(result.metadata.warning_message, urdfTrunk.warning_message);

            pinocchio::JointIndex parentJoint = 0;
            pinocchio::SE3 accumulatedPlacement = pinocchio::SE3::Identity();
            QHash<QString, int> previewFrameIdByLinkName;
            const QString rootLinkName =
                !urdfTrunk.ordered_joint_indices.empty()
                    ? urdfModel.joints[static_cast<std::size_t>(urdfTrunk.ordered_joint_indices.front())].parent_link_name
                    : (urdfModel.root_link_names.isEmpty() ? QStringLiteral("base_link") : urdfModel.root_link_names.front());
            const int rootPreviewFrameId = static_cast<int>(nativeModel->addFrame(pinocchio::Frame(
                QStringLiteral("robosdp_urdf_preview_root_%1").arg(rootLinkName).toStdString(),
                0,
                0,
                BuildSe3FromPose(model.base_frame),
                pinocchio::OP_FRAME)));
            previewNodes.push_back({rootLinkName, rootPreviewFrameId});
            previewFrameIdByLinkName.insert(rootLinkName, rootPreviewFrameId);

            const auto registerPreviewNode =
                [&previewNodes, &previewFrameIdByLinkName](const QString& linkName, int frameId)
            {
                if (!previewFrameIdByLinkName.contains(linkName))
                {
                    previewNodes.push_back({linkName, frameId});
                }
                previewFrameIdByLinkName.insert(linkName, frameId);
            };

            const auto registerPreviewSegment =
                [&previewSegments](
                    const QString& jointName,
                    const QString& jointType,
                    const QString& parentLinkName,
                    const QString& childLinkName,
                    int startFrameId,
                    int endFrameId)
            {
                previewSegments.push_back({
                    jointName,
                    jointType,
                    parentLinkName,
                    childLinkName,
                    startFrameId,
                    endFrameId});
            };

            for (int orderedJointIndex : urdfTrunk.ordered_joint_indices)
            {
                const auto& urdfJoint = urdfModel.joints[static_cast<std::size_t>(orderedJointIndex)];
                accumulatedPlacement = accumulatedPlacement * BuildUrdfJointPlacement(urdfJoint);
                const int parentPreviewFrameId =
                    previewFrameIdByLinkName.value(urdfJoint.parent_link_name, rootPreviewFrameId);

                if (urdfJoint.joint_type == QStringLiteral("fixed"))
                {
                    const pinocchio::SE3 childLinkPlacement =
                        parentJoint == 0 ? BuildSe3FromPose(model.base_frame) * accumulatedPlacement : accumulatedPlacement;
                    const int fixedChildFrameId = static_cast<int>(nativeModel->addFrame(pinocchio::Frame(
                        QStringLiteral("robosdp_urdf_fixed_link_frame_%1").arg(urdfJoint.child_link_name).toStdString(),
                        parentJoint,
                        0,
                        childLinkPlacement,
                        pinocchio::OP_FRAME)));
                    registerPreviewNode(urdfJoint.child_link_name, fixedChildFrameId);
                    registerPreviewSegment(
                        urdfJoint.joint_id,
                        urdfJoint.joint_type,
                        urdfJoint.parent_link_name,
                        urdfJoint.child_link_name,
                        parentPreviewFrameId,
                        fixedChildFrameId);
                    continue;
                }

                if (urdfJoint.joint_type != QStringLiteral("revolute") &&
                    urdfJoint.joint_type != QStringLiteral("continuous"))
                {
                    result.metadata.status_code = QStringLiteral("shared_kernel_urdf_joint_type_unsupported");
                    result.metadata.status_message = QStringLiteral("共享机器人内核暂不支持 URDF joint=%1 的 type=%2。")
                        .arg(urdfJoint.joint_id, urdfJoint.joint_type);
                    return result;
                }

                const pinocchio::SE3 jointPlacement =
                    parentJoint == 0 ? BuildSe3FromPose(model.base_frame) * accumulatedPlacement : accumulatedPlacement;
                const pinocchio::JointIndex currentJoint = nativeModel->addJoint(
                    parentJoint,
                    SharedJointModelRZ(),
                    jointPlacement,
                    urdfJoint.joint_id.toStdString());
                nativeModel->appendBodyToJoint(currentJoint, BuildPlaceholderInertia(), pinocchio::SE3::Identity());
                const int childLinkFrameId = static_cast<int>(nativeModel->addFrame(pinocchio::Frame(
                    QStringLiteral("robosdp_urdf_link_frame_%1").arg(urdfJoint.child_link_name).toStdString(),
                    currentJoint,
                    0,
                    pinocchio::SE3::Identity(),
                    pinocchio::OP_FRAME)));
                linkFrameIds.push_back(childLinkFrameId);
                registerPreviewNode(urdfJoint.child_link_name, childLinkFrameId);
                registerPreviewSegment(
                    urdfJoint.joint_id,
                    urdfJoint.joint_type,
                    urdfJoint.parent_link_name,
                    urdfJoint.child_link_name,
                    parentPreviewFrameId,
                    childLinkFrameId);
                parentJoint = currentJoint;
                nativePositionOffsetsDeg.push_back(0.0);
                accumulatedPlacement = pinocchio::SE3::Identity();
            }
            terminalToolPlacement = accumulatedPlacement;
        }
        else
        {
            const bool hasDynamicInertia =
                request.dynamic_model != nullptr &&
                static_cast<int>(request.dynamic_model->links.size()) >= static_cast<int>(model.links.size());

            if (!hasDynamicInertia)
            {
                AppendWarningMessage(
                    result.metadata.warning_message,
                    QStringLiteral("共享机器人内核当前未拿到完整动力学惯量参数，已注入安全占位惯量。"));
            }

            pinocchio::JointIndex parentJoint = 0;
            pinocchio::SE3 previousPostJointPlacement = pinocchio::SE3::Identity();
            for (std::size_t jointIndex = 0; jointIndex < model.links.size(); ++jointIndex)
            {
                const auto& link = model.links[jointIndex];
                const pinocchio::SE3 currentPreJointPlacement =
                    modelingMode == QStringLiteral("MDH") ? BuildModifiedDhPreJointPlacement(link) : pinocchio::SE3::Identity();
                const pinocchio::SE3 jointPlacement =
                    parentJoint == 0
                        ? BuildSe3FromPose(model.base_frame) * previousPostJointPlacement * currentPreJointPlacement
                        : previousPostJointPlacement * currentPreJointPlacement;

                const QString jointId =
                    jointIndex < model.joint_limits.size() ? model.joint_limits[jointIndex].joint_id.trimmed()
                                                           : QStringLiteral("joint_%1").arg(jointIndex + 1);
                const pinocchio::JointIndex currentJoint = nativeModel->addJoint(
                    parentJoint,
                    SharedJointModelRZ(),
                    jointPlacement,
                    jointId.toStdString());

                pinocchio::Inertia bodyInertia = BuildPlaceholderInertia();
                if (hasDynamicInertia)
                {
                    bodyInertia = BuildLinkInertia(request.dynamic_model->links[jointIndex], usedPlaceholderInertia);
                }
                nativeModel->appendBodyToJoint(currentJoint, bodyInertia, pinocchio::SE3::Identity());

                if (hasDynamicInertia && jointIndex + 1 == model.links.size())
                {
                    nativeModel->appendBodyToJoint(
                        currentJoint,
                        BuildToolInertia(request.dynamic_model->end_effector, usedPlaceholderInertia),
                        pinocchio::SE3::Identity());
                }

                const pinocchio::SE3 currentPostJointPlacement =
                    modelingMode == QStringLiteral("MDH") ? BuildModifiedDhPostJointPlacement(link)
                                                          : BuildStandardDhPostJointPlacement(link);
                linkFrameIds.push_back(static_cast<int>(nativeModel->addFrame(pinocchio::Frame(
                    QStringLiteral("robosdp_link_frame_%1").arg(jointId).toStdString(),
                    currentJoint,
                    0,
                    currentPostJointPlacement,
                    pinocchio::OP_FRAME))));
                parentJoint = currentJoint;
                nativePositionOffsetsDeg.push_back(link.theta_offset);
                previousPostJointPlacement = currentPostJointPlacement;
            }
            terminalToolPlacement = previousPostJointPlacement;
        }

        if (nativeModel->nq <= 0 || nativeModel->nv <= 0 || nativeModel->njoints <= 1)
        {
            result.metadata.status_code = QStringLiteral("shared_kernel_empty_model");
            result.metadata.status_message = QStringLiteral("共享机器人内核构建后没有有效自由度。");
            return result;
        }

        const pinocchio::JointIndex semanticParentJoint = static_cast<pinocchio::JointIndex>(nativeModel->njoints - 1);
        result.metadata.base_frame_id = static_cast<int>(nativeModel->addFrame(pinocchio::Frame(
            "robosdp_base_frame",
            0,
            0,
            BuildSe3FromPose(model.base_frame),
            pinocchio::OP_FRAME)));
        result.metadata.flange_frame_id = static_cast<int>(nativeModel->addFrame(pinocchio::Frame(
            "robosdp_flange_frame",
            semanticParentJoint,
            0,
            terminalToolPlacement * BuildSe3FromPose(model.flange_frame),
            pinocchio::OP_FRAME)));
        result.metadata.tcp_frame_id = static_cast<int>(nativeModel->addFrame(pinocchio::Frame(
            "robosdp_tcp_frame",
            semanticParentJoint,
            0,
            terminalToolPlacement * BuildSe3FromPose(model.flange_frame) * BuildSe3FromTcp(model.tcp_frame),
            pinocchio::OP_FRAME)));

        result.success = true;
        result.model = std::move(nativeModel);
        result.metadata.link_frame_ids = std::move(linkFrameIds);
        result.metadata.preview_nodes = std::move(previewNodes);
        result.metadata.preview_segments = std::move(previewSegments);
        result.metadata.native_position_offsets_deg = std::move(nativePositionOffsetsDeg);
        result.metadata.shared_kernel_ready = true;
        result.metadata.contains_explicit_inertia = request.dynamic_model != nullptr && !usedPlaceholderInertia;
        result.metadata.native_joint_count = static_cast<int>(result.model->njoints);
        result.metadata.native_frame_count = static_cast<int>(result.model->nframes);
        result.metadata.registry_build_count = ++RegistryBuildCounter();
        result.metadata.status_code = QStringLiteral("shared_kernel_ready");
        result.metadata.status_message = QStringLiteral("SharedRobotKernelRegistry 已生成并缓存共享 Pinocchio Model。");
        return result;
    }
    catch (const std::exception& exception)
    {
        result.metadata.status_code = QStringLiteral("shared_kernel_build_exception");
        result.metadata.status_message = QStringLiteral("共享机器人内核构建异常：%1").arg(QString::fromUtf8(exception.what()));
        return result;
    }
    catch (...)
    {
        result.metadata.status_code = QStringLiteral("shared_kernel_build_unknown_exception");
        result.metadata.status_message = QStringLiteral("共享机器人内核构建出现未知异常，已拦截以避免崩溃。");
        return result;
    }
}
#endif

} // namespace

SharedRobotKernelRegistry& SharedRobotKernelRegistry::Instance()
{
    static SharedRobotKernelRegistry registry;
    return registry;
}

SharedRobotKernelAcquireResult SharedRobotKernelRegistry::GetOrBuildKernel(const SharedRobotKernelRequest& request)
{
    SharedRobotKernelAcquireResult result;
    const QString structuralKey = BuildStructuralKey(request);
    const QString inertialHash = BuildInertialHash(request);
    const QString cacheKey = BuildFullCacheKey(structuralKey, inertialHash);

    result.metadata.structural_key = structuralKey;
    result.metadata.inertial_hash = inertialHash;
    result.metadata.cache_key = cacheKey;

    if (structuralKey.trimmed().isEmpty())
    {
        result.metadata.status_code = QStringLiteral("shared_kernel_missing_structural_key");
        result.metadata.status_message = QStringLiteral("共享机器人内核缓存键构建失败：缺少 unified_robot_model_ref / modeling_mode / joint_order_signature。");
        return result;
    }

#if !defined(ROBOSDP_HAVE_PINOCCHIO)
    result.metadata.status_code = QStringLiteral("shared_kernel_dependency_unavailable");
    result.metadata.status_message = QStringLiteral("当前构建未检测到 Pinocchio C++ 依赖，无法创建共享机器人内核。");
    return result;
#else
    std::lock_guard<std::mutex> lock(RegistryMutex());
    auto& kernelCache = KernelCache();
    auto& aliasCache = StructuralAliasCache();

    if (request.allow_structural_alias)
    {
        const auto aliasIterator = aliasCache.constFind(structuralKey);
        if (aliasIterator != aliasCache.constEnd())
        {
            const auto kernelIterator = kernelCache.constFind(aliasIterator.value());
            if (kernelIterator != kernelCache.constEnd())
            {
                result = kernelIterator.value();
                result.cache_hit = true;
                result.metadata.status_code = QStringLiteral("shared_kernel_alias_cache_hit");
                result.metadata.status_message = QStringLiteral("SharedRobotKernelRegistry 命中结构别名缓存，已复用更完整的共享 Model。");
                return result;
            }
        }
    }

    const auto existing = kernelCache.constFind(cacheKey);
    if (existing != kernelCache.constEnd())
    {
        result = existing.value();
        result.cache_hit = true;
        result.metadata.status_code = QStringLiteral("shared_kernel_exact_cache_hit");
        result.metadata.status_message = QStringLiteral("SharedRobotKernelRegistry 命中精确缓存，未重复建树。");
        return result;
    }

    result = BuildKernel(request, structuralKey, inertialHash, cacheKey);
    if (result.success)
    {
        kernelCache.insert(cacheKey, result);
        const auto aliasIterator = aliasCache.constFind(structuralKey);
        if (result.metadata.contains_explicit_inertia || aliasIterator == aliasCache.constEnd())
        {
            aliasCache.insert(structuralKey, cacheKey);
        }
    }
    return result;
#endif
}

} // namespace RoboSDP::Core::Kinematics
