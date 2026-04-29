#include "modules/topology/service/TopologyTemplateLoader.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace RoboSDP::Topology::Service
{

namespace
{

QString ReadString(const QJsonObject& object, const QString& key, const QString& defaultValue = {})
{
    return object.contains(key) ? object.value(key).toString(defaultValue) : defaultValue;
}

double ReadDouble(const QJsonObject& object, const QString& key, double defaultValue = 0.0)
{
    return object.contains(key) ? object.value(key).toDouble(defaultValue) : defaultValue;
}

int ReadInt(const QJsonObject& object, const QString& key, int defaultValue = 0)
{
    return object.contains(key) ? object.value(key).toInt(defaultValue) : defaultValue;
}

bool ReadBool(const QJsonObject& object, const QString& key, bool defaultValue = false)
{
    return object.contains(key) ? object.value(key).toBool(defaultValue) : defaultValue;
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

std::vector<QString> ReadStringArray(const QJsonArray& array)
{
    std::vector<QString> values;
    values.reserve(static_cast<std::size_t>(array.size()));
    for (const QJsonValue& value : array)
    {
        values.push_back(value.toString());
    }

    return values;
}

} // namespace

TopologyTemplateLoader::TopologyTemplateLoader(const QString& templateDirectory)
    : m_template_directory(templateDirectory)
{
}

RoboSDP::Errors::ErrorCode TopologyTemplateLoader::LoadTemplateSummaries(
    std::vector<TopologyTemplateSummary>& summaries) const
{
    std::vector<TopologyTemplateRecord> templates;
    const RoboSDP::Errors::ErrorCode errorCode = LoadTemplates(templates);
    if (errorCode != RoboSDP::Errors::ErrorCode::Ok)
    {
        return errorCode;
    }

    summaries.clear();
    summaries.reserve(templates.size());
    for (const auto& templateRecord : templates)
    {
        summaries.push_back(templateRecord.summary);
    }

    return RoboSDP::Errors::ErrorCode::Ok;
}

RoboSDP::Errors::ErrorCode TopologyTemplateLoader::LoadTemplates(
    std::vector<TopologyTemplateRecord>& templates) const
{
    const QString templateDirectory = ResolveTemplateDirectory();
    const QDir dir(templateDirectory);
    if (!dir.exists())
    {
        return RoboSDP::Errors::ErrorCode::RepositoryRootNotFound;
    }

    const QFileInfoList fileInfos = dir.entryInfoList(
        QStringList {QStringLiteral("*.json")},
        QDir::Files | QDir::Readable,
        QDir::Name);

    if (fileInfos.isEmpty())
    {
        return RoboSDP::Errors::ErrorCode::RepositoryDocumentNotFound;
    }

    templates.clear();
    templates.reserve(static_cast<std::size_t>(fileInfos.size()));

    for (const QFileInfo& fileInfo : fileInfos)
    {
        QFile file(fileInfo.absoluteFilePath());
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            return RoboSDP::Errors::ErrorCode::RepositoryReadFailed;
        }

        QJsonParseError parseError;
        const QJsonDocument jsonDocument = QJsonDocument::fromJson(file.readAll(), &parseError);
        file.close();

        if (parseError.error != QJsonParseError::NoError || !jsonDocument.isObject())
        {
            return RoboSDP::Errors::ErrorCode::JsonFormatInvalid;
        }

        templates.push_back(ParseTemplateDocument(jsonDocument.object(), fileInfo.absoluteFilePath()));
    }

    return RoboSDP::Errors::ErrorCode::Ok;
}

RoboSDP::Errors::ErrorCode TopologyTemplateLoader::LoadTemplate(
    const QString& templateId,
    TopologyTemplateRecord& templateRecord) const
{
    std::vector<TopologyTemplateRecord> templates;
    const RoboSDP::Errors::ErrorCode errorCode = LoadTemplates(templates);
    if (errorCode != RoboSDP::Errors::ErrorCode::Ok)
    {
        return errorCode;
    }

    for (const auto& candidate : templates)
    {
        if (candidate.summary.template_id == templateId)
        {
            templateRecord = candidate;
            return RoboSDP::Errors::ErrorCode::Ok;
        }
    }

    return RoboSDP::Errors::ErrorCode::RepositoryDocumentNotFound;
}

QString TopologyTemplateLoader::TemplateDirectory() const
{
    return ResolveTemplateDirectory();
}

QString TopologyTemplateLoader::ResolveTemplateDirectory() const
{
    if (!m_template_directory.trimmed().isEmpty())
    {
        return QDir::fromNativeSeparators(m_template_directory.trimmed());
    }

#ifdef ROBOSDP_SOURCE_DIR
    const QString sourceDirectory =
        QDir(QString::fromUtf8(ROBOSDP_SOURCE_DIR)).filePath(QStringLiteral("resources/topology/templates"));
    if (QDir(sourceDirectory).exists())
    {
        return sourceDirectory;
    }
#endif

    const QString currentDirectory =
        QDir::current().filePath(QStringLiteral("resources/topology/templates"));
    if (QDir(currentDirectory).exists())
    {
        return currentDirectory;
    }

    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("../resources/topology/templates"));
}

TopologyTemplateRecord TopologyTemplateLoader::ParseTemplateDocument(
    const QJsonObject& jsonObject,
    const QString& filePath) const
{
    TopologyTemplateRecord record;
    record.summary.template_id = ReadString(jsonObject, QStringLiteral("template_id"));
    record.summary.display_name = ReadString(jsonObject, QStringLiteral("display_name"));
    record.summary.description = ReadString(jsonObject, QStringLiteral("description"));
    record.file_path = filePath;

    const QJsonObject modelObject = jsonObject.value(QStringLiteral("model")).toObject();
    record.model = ParseModelObject(modelObject);
    if (record.model.meta.template_id.trimmed().isEmpty())
    {
        record.model.meta.template_id = record.summary.template_id;
    }
    if (record.model.meta.name.trimmed().isEmpty())
    {
        record.model.meta.name = record.summary.display_name;
    }

    return record;
}

RoboSDP::Topology::Dto::RobotTopologyModelDto TopologyTemplateLoader::ParseModelObject(
    const QJsonObject& jsonObject) const
{
    using namespace RoboSDP::Topology::Dto;

    RobotTopologyModelDto model = RobotTopologyModelDto::CreateDefault();

    const QJsonObject metaObject = jsonObject.value(QStringLiteral("meta")).toObject();
    model.meta.topology_id = ReadString(metaObject, QStringLiteral("topology_id"), model.meta.topology_id);
    model.meta.name = ReadString(metaObject, QStringLiteral("name"), model.meta.name);
    model.meta.version = ReadInt(metaObject, QStringLiteral("version"), model.meta.version);
    model.meta.source = ReadString(metaObject, QStringLiteral("source"), model.meta.source);
    model.meta.status = ReadString(metaObject, QStringLiteral("status"), model.meta.status);
    model.meta.remarks = ReadString(metaObject, QStringLiteral("remarks"));
    model.meta.template_id = ReadString(metaObject, QStringLiteral("template_id"));
    model.meta.requirement_ref = ReadString(metaObject, QStringLiteral("requirement_ref"));

// ---------- 替换：定义对象反序列化（增加 DH 尺寸解析） ----------
    const QJsonObject definitionObject =
        jsonObject.value(QStringLiteral("robot_definition")).toObject();
    model.robot_definition.robot_type =
        ReadString(definitionObject, QStringLiteral("robot_type"), model.robot_definition.robot_type);
    model.robot_definition.joint_count =
        ReadInt(definitionObject, QStringLiteral("joint_count"), model.robot_definition.joint_count);
    model.robot_definition.application_tags = ReadStringArray(
        definitionObject.value(QStringLiteral("application_tags")).toArray());
    model.robot_definition.base_mount_type = ReadString(
        definitionObject,
        QStringLiteral("base_mount_type"),
        model.robot_definition.base_mount_type);
    
    // 读取所有 DH 关键尺寸
    model.robot_definition.base_height_m =
        ReadDouble(definitionObject, QStringLiteral("base_height_m"), model.robot_definition.base_height_m);
    model.robot_definition.shoulder_offset_m =
        ReadDouble(definitionObject, QStringLiteral("shoulder_offset_m"), model.robot_definition.shoulder_offset_m);
    model.robot_definition.upper_arm_length_m =
        ReadDouble(definitionObject, QStringLiteral("upper_arm_length_m"), model.robot_definition.upper_arm_length_m);
    model.robot_definition.forearm_length_m =
        ReadDouble(definitionObject, QStringLiteral("forearm_length_m"), model.robot_definition.forearm_length_m);
    model.robot_definition.wrist_offset_m =
        ReadDouble(definitionObject, QStringLiteral("wrist_offset_m"), model.robot_definition.wrist_offset_m);

    model.robot_definition.base_orientation = ReadArray3(
        definitionObject.value(QStringLiteral("base_orientation")).toArray());
    model.robot_definition.j1_rotation_range_deg = ReadArray2(
        definitionObject.value(QStringLiteral("j1_rotation_range_deg")).toArray());

    // ---------- 替换：布局对象反序列化（删除废弃的文本字段） ----------
    const QJsonObject layoutObject = jsonObject.value(QStringLiteral("layout")).toObject();
    
    // 已删除 shoulder_type, elbow_type, wrist_type, wrist_intersection, wrist_offset 的解析
    
    model.layout.internal_routing_required =
        ReadBool(
            layoutObject,
            QStringLiteral("internal_routing_required"),
            model.layout.internal_routing_required);
    model.layout.hollow_joint_ids = ReadStringArray(
        layoutObject.value(QStringLiteral("hollow_joint_ids")).toArray());
    model.layout.hollow_wrist_required =
        ReadBool(layoutObject, QStringLiteral("hollow_wrist_required"), model.layout.hollow_wrist_required);
    model.layout.reserved_channel_diameter_mm =
        ReadDouble(layoutObject, QStringLiteral("reserved_channel_diameter_mm"), model.layout.reserved_channel_diameter_mm);
    model.layout.seventh_axis_reserved =
        ReadBool(layoutObject, QStringLiteral("seventh_axis_reserved"), model.layout.seventh_axis_reserved);

    // 下面接着是 axis_relations.clear() 等原本的代码...

    model.axis_relations.clear();
    const QJsonArray axisRelationsArray = jsonObject.value(QStringLiteral("axis_relations")).toArray();
    for (const QJsonValue& relationValue : axisRelationsArray)
    {
        const QJsonObject relationObject = relationValue.toObject();
        const QJsonArray jointPairArray = relationObject.value(QStringLiteral("joint_pair")).toArray();
        TopologyAxisRelationDto relation;
        if (jointPairArray.size() > 0)
        {
            relation.joint_pair[0] = jointPairArray.at(0).toString();
        }
        if (jointPairArray.size() > 1)
        {
            relation.joint_pair[1] = jointPairArray.at(1).toString();
        }
        relation.relation_type = ReadString(relationObject, QStringLiteral("relation_type"));
        model.axis_relations.push_back(relation);
    }

    model.joints.clear();
    const QJsonArray jointsArray = jsonObject.value(QStringLiteral("joints")).toArray();
    for (const QJsonValue& jointValue : jointsArray)
    {
        const QJsonObject jointObject = jointValue.toObject();
        TopologyJointDto joint;
        joint.joint_id = ReadString(jointObject, QStringLiteral("joint_id"));
        joint.axis_index = ReadInt(jointObject, QStringLiteral("axis_index"));
        joint.role = ReadString(jointObject, QStringLiteral("role"));
        joint.joint_type = ReadString(
            jointObject,
            QStringLiteral("joint_type"),
            QStringLiteral("revolute"));
        joint.parent_link_id = ReadString(jointObject, QStringLiteral("parent_link_id"));
        joint.child_link_id = ReadString(jointObject, QStringLiteral("child_link_id"));
        joint.axis_direction = ReadArray3(jointObject.value(QStringLiteral("axis_direction")).toArray());
        joint.motion_range_deg = ReadArray2(jointObject.value(QStringLiteral("motion_range_deg")).toArray());
        model.joints.push_back(joint);
    }

    model.topology_graph.links.clear();
    model.topology_graph.joints_graph.clear();
    const QJsonObject topologyGraphObject =
        jsonObject.value(QStringLiteral("topology_graph")).toObject();

    const QJsonArray linksArray = topologyGraphObject.value(QStringLiteral("links")).toArray();
    for (const QJsonValue& linkValue : linksArray)
    {
        const QJsonObject linkObject = linkValue.toObject();
        model.topology_graph.links.push_back({
            ReadString(linkObject, QStringLiteral("link_id")),
            ReadString(linkObject, QStringLiteral("parent_joint_id")),
            ReadString(linkObject, QStringLiteral("name"))});
    }

    const QJsonArray graphJointsArray =
        topologyGraphObject.value(QStringLiteral("joints_graph")).toArray();
    for (const QJsonValue& graphJointValue : graphJointsArray)
    {
        const QJsonObject graphJointObject = graphJointValue.toObject();
        model.topology_graph.joints_graph.push_back({
            ReadString(graphJointObject, QStringLiteral("joint_id")),
            ReadString(graphJointObject, QStringLiteral("parent_link_id")),
            ReadString(graphJointObject, QStringLiteral("child_link_id"))});
    }

    return model;
}

} // namespace RoboSDP::Topology::Service
