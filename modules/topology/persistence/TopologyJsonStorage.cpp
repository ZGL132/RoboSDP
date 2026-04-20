#include "modules/topology/persistence/TopologyJsonStorage.h"

#include <QDir>
#include <QJsonArray>
#include <QJsonObject>

namespace RoboSDP::Topology::Persistence
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

QJsonArray ToStringArray(const std::vector<QString>& values)
{
    QJsonArray array;
    for (const QString& value : values)
    {
        array.append(value);
    }

    return array;
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

TopologyJsonStorage::TopologyJsonStorage(RoboSDP::Repository::IJsonRepository& repository)
    : m_repository(repository)
{
}

RoboSDP::Errors::ErrorCode TopologyJsonStorage::Save(
    const QString& projectRootPath,
    const RoboSDP::Topology::Dto::TopologyWorkspaceStateDto& state) const
{
    const RoboSDP::Errors::ErrorCode openProjectError = m_repository.OpenProject(projectRootPath);
    if (openProjectError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return openProjectError;
    }

    return m_repository.WriteDocument(RelativeFilePath(), ToJsonObject(state));
}

RoboSDP::Errors::ErrorCode TopologyJsonStorage::Load(
    const QString& projectRootPath,
    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto& state) const
{
    const RoboSDP::Errors::ErrorCode openProjectError = m_repository.OpenProject(projectRootPath);
    if (openProjectError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return openProjectError;
    }

    QJsonObject jsonObject;
    const RoboSDP::Errors::ErrorCode readError = m_repository.ReadDocument(RelativeFilePath(), jsonObject);
    if (readError != RoboSDP::Errors::ErrorCode::Ok)
    {
        return readError;
    }

    state = FromJsonObject(jsonObject);
    return RoboSDP::Errors::ErrorCode::Ok;
}

QString TopologyJsonStorage::RelativeFilePath() const
{
    return QStringLiteral("topology/topology-model.json");
}

QString TopologyJsonStorage::BuildAbsoluteFilePath(const QString& projectRootPath) const
{
    return QDir(projectRootPath).filePath(RelativeFilePath());
}

QJsonObject TopologyJsonStorage::ToJsonObject(
    const RoboSDP::Topology::Dto::TopologyWorkspaceStateDto& state) const
{
    QJsonObject rootObject = ToModelObject(state.current_model);

    QJsonArray candidatesArray;
    for (const auto& candidate : state.candidates)
    {
        QJsonObject candidateObject;
        candidateObject.insert(QStringLiteral("candidate_id"), candidate.candidate_id);
        candidateObject.insert(QStringLiteral("title"), candidate.title);
        candidateObject.insert(QStringLiteral("template_id"), candidate.template_id);
        candidateObject.insert(QStringLiteral("score"), candidate.score);
        candidateObject.insert(QStringLiteral("matches_requirement"), candidate.matches_requirement);
        candidateObject.insert(
            QStringLiteral("recommendation_reason"),
            ToStringArray(candidate.recommendation_reason));
        candidateObject.insert(QStringLiteral("model"), ToModelObject(candidate.model));
        candidatesArray.append(candidateObject);
    }
    rootObject.insert(QStringLiteral("candidates"), candidatesArray);

    QJsonObject recommendationObject;
    recommendationObject.insert(
        QStringLiteral("recommended_candidate_id"),
        state.recommendation.recommended_candidate_id);
    recommendationObject.insert(
        QStringLiteral("recommended_topology_id"),
        state.recommendation.recommended_topology_id);
    recommendationObject.insert(
        QStringLiteral("recommended_template_id"),
        state.recommendation.recommended_template_id);
    recommendationObject.insert(QStringLiteral("combined_score"), state.recommendation.combined_score);
    recommendationObject.insert(
        QStringLiteral("requirement_loaded"),
        state.recommendation.requirement_loaded);
    recommendationObject.insert(
        QStringLiteral("requirement_constraints_applied"),
        state.recommendation.requirement_constraints_applied);
    recommendationObject.insert(
        QStringLiteral("recommendation_reason"),
        ToStringArray(state.recommendation.recommendation_reason));
    rootObject.insert(QStringLiteral("recommendation"), recommendationObject);

    return rootObject;
}

RoboSDP::Topology::Dto::TopologyWorkspaceStateDto TopologyJsonStorage::FromJsonObject(
    const QJsonObject& jsonObject) const
{
    using namespace RoboSDP::Topology::Dto;

    TopologyWorkspaceStateDto state = TopologyWorkspaceStateDto::CreateDefault();
    state.current_model = FromModelObject(jsonObject);
    state.candidates.clear();

    const QJsonArray candidatesArray = jsonObject.value(QStringLiteral("candidates")).toArray();
    for (const QJsonValue& candidateValue : candidatesArray)
    {
        const QJsonObject candidateObject = candidateValue.toObject();
        TopologyCandidateDto candidate;
        candidate.candidate_id = ReadString(candidateObject, QStringLiteral("candidate_id"));
        candidate.title = ReadString(candidateObject, QStringLiteral("title"));
        candidate.template_id = ReadString(candidateObject, QStringLiteral("template_id"));
        candidate.score = ReadDouble(candidateObject, QStringLiteral("score"));
        candidate.matches_requirement =
            ReadBool(candidateObject, QStringLiteral("matches_requirement"));
        candidate.recommendation_reason = ReadStringArray(
            candidateObject.value(QStringLiteral("recommendation_reason")).toArray());
        candidate.model = FromModelObject(candidateObject.value(QStringLiteral("model")).toObject());
        state.candidates.push_back(candidate);
    }

    const QJsonObject recommendationObject =
        jsonObject.value(QStringLiteral("recommendation")).toObject();
    state.recommendation.recommended_candidate_id =
        ReadString(recommendationObject, QStringLiteral("recommended_candidate_id"));
    state.recommendation.recommended_topology_id =
        ReadString(recommendationObject, QStringLiteral("recommended_topology_id"));
    state.recommendation.recommended_template_id =
        ReadString(recommendationObject, QStringLiteral("recommended_template_id"));
    state.recommendation.combined_score =
        ReadDouble(recommendationObject, QStringLiteral("combined_score"));
    state.recommendation.requirement_loaded =
        ReadBool(recommendationObject, QStringLiteral("requirement_loaded"));
    state.recommendation.requirement_constraints_applied =
        ReadBool(recommendationObject, QStringLiteral("requirement_constraints_applied"));
    state.recommendation.recommendation_reason = ReadStringArray(
        recommendationObject.value(QStringLiteral("recommendation_reason")).toArray());

    return state;
}

QJsonObject TopologyJsonStorage::ToModelObject(
    const RoboSDP::Topology::Dto::RobotTopologyModelDto& model) const
{
    QJsonObject rootObject;

    QJsonObject metaObject;
    metaObject.insert(QStringLiteral("topology_id"), model.meta.topology_id);
    metaObject.insert(QStringLiteral("name"), model.meta.name);
    metaObject.insert(QStringLiteral("version"), model.meta.version);
    metaObject.insert(QStringLiteral("source"), model.meta.source);
    metaObject.insert(QStringLiteral("status"), model.meta.status);
    metaObject.insert(QStringLiteral("remarks"), model.meta.remarks);
    metaObject.insert(QStringLiteral("template_id"), model.meta.template_id);
    metaObject.insert(QStringLiteral("requirement_ref"), model.meta.requirement_ref);
    rootObject.insert(QStringLiteral("meta"), metaObject);

    QJsonObject definitionObject;
    definitionObject.insert(QStringLiteral("robot_type"), model.robot_definition.robot_type);
    definitionObject.insert(QStringLiteral("joint_count"), model.robot_definition.joint_count);
    definitionObject.insert(
        QStringLiteral("application_tags"),
        ToStringArray(model.robot_definition.application_tags));
    definitionObject.insert(
        QStringLiteral("base_mount_type"),
        model.robot_definition.base_mount_type);
    definitionObject.insert(QStringLiteral("base_height_m"), model.robot_definition.base_height_m);
    definitionObject.insert(
        QStringLiteral("base_orientation"),
        ToJsonArray3(model.robot_definition.base_orientation));
    definitionObject.insert(
        QStringLiteral("j1_rotation_range_deg"),
        ToJsonArray2(model.robot_definition.j1_rotation_range_deg));
    rootObject.insert(QStringLiteral("robot_definition"), definitionObject);

    QJsonObject layoutObject;
    layoutObject.insert(QStringLiteral("shoulder_type"), model.layout.shoulder_type);
    layoutObject.insert(QStringLiteral("elbow_type"), model.layout.elbow_type);
    layoutObject.insert(QStringLiteral("wrist_type"), model.layout.wrist_type);
    layoutObject.insert(
        QStringLiteral("wrist_intersection"),
        model.layout.wrist_intersection);
    layoutObject.insert(
        QStringLiteral("wrist_offset"),
        model.layout.wrist_offset);
    layoutObject.insert(
        QStringLiteral("internal_routing_required"),
        model.layout.internal_routing_required);
    layoutObject.insert(
        QStringLiteral("hollow_joint_ids"),
        ToStringArray(model.layout.hollow_joint_ids));
    layoutObject.insert(
        QStringLiteral("hollow_wrist_required"),
        model.layout.hollow_wrist_required);
    layoutObject.insert(
        QStringLiteral("reserved_channel_diameter_mm"),
        model.layout.reserved_channel_diameter_mm);
    layoutObject.insert(
        QStringLiteral("seventh_axis_reserved"),
        model.layout.seventh_axis_reserved);
    rootObject.insert(QStringLiteral("layout"), layoutObject);

    QJsonArray axisRelationsArray;
    for (const auto& relation : model.axis_relations)
    {
        QJsonObject relationObject;
        QJsonArray jointPairArray;
        jointPairArray.append(relation.joint_pair[0]);
        jointPairArray.append(relation.joint_pair[1]);
        relationObject.insert(QStringLiteral("joint_pair"), jointPairArray);
        relationObject.insert(QStringLiteral("relation_type"), relation.relation_type);
        axisRelationsArray.append(relationObject);
    }
    rootObject.insert(QStringLiteral("axis_relations"), axisRelationsArray);

    QJsonArray jointsArray;
    for (const auto& joint : model.joints)
    {
        QJsonObject jointObject;
        jointObject.insert(QStringLiteral("joint_id"), joint.joint_id);
        jointObject.insert(QStringLiteral("axis_index"), joint.axis_index);
        jointObject.insert(QStringLiteral("role"), joint.role);
        jointObject.insert(QStringLiteral("joint_type"), joint.joint_type);
        jointObject.insert(QStringLiteral("parent_link_id"), joint.parent_link_id);
        jointObject.insert(QStringLiteral("child_link_id"), joint.child_link_id);
        jointObject.insert(
            QStringLiteral("axis_direction"),
            ToJsonArray3(joint.axis_direction));
        jointObject.insert(
            QStringLiteral("motion_range_deg"),
            ToJsonArray2(joint.motion_range_deg));
        jointsArray.append(jointObject);
    }
    rootObject.insert(QStringLiteral("joints"), jointsArray);

    QJsonObject topologyGraphObject;
    QJsonArray linksArray;
    for (const auto& link : model.topology_graph.links)
    {
        QJsonObject linkObject;
        linkObject.insert(QStringLiteral("link_id"), link.link_id);
        linkObject.insert(QStringLiteral("parent_joint_id"), link.parent_joint_id);
        linkObject.insert(QStringLiteral("name"), link.name);
        linksArray.append(linkObject);
    }

    QJsonArray jointsGraphArray;
    for (const auto& graphJoint : model.topology_graph.joints_graph)
    {
        QJsonObject graphJointObject;
        graphJointObject.insert(QStringLiteral("joint_id"), graphJoint.joint_id);
        graphJointObject.insert(QStringLiteral("parent_link_id"), graphJoint.parent_link_id);
        graphJointObject.insert(QStringLiteral("child_link_id"), graphJoint.child_link_id);
        jointsGraphArray.append(graphJointObject);
    }

    topologyGraphObject.insert(QStringLiteral("links"), linksArray);
    topologyGraphObject.insert(QStringLiteral("joints_graph"), jointsGraphArray);
    rootObject.insert(QStringLiteral("topology_graph"), topologyGraphObject);

    return rootObject;
}

RoboSDP::Topology::Dto::RobotTopologyModelDto TopologyJsonStorage::FromModelObject(
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
    model.robot_definition.base_height_m =
        ReadDouble(definitionObject, QStringLiteral("base_height_m"), model.robot_definition.base_height_m);
    model.robot_definition.base_orientation = ReadArray3(
        definitionObject.value(QStringLiteral("base_orientation")).toArray());
    model.robot_definition.j1_rotation_range_deg = ReadArray2(
        definitionObject.value(QStringLiteral("j1_rotation_range_deg")).toArray());

    const QJsonObject layoutObject = jsonObject.value(QStringLiteral("layout")).toObject();
    model.layout.shoulder_type =
        ReadString(layoutObject, QStringLiteral("shoulder_type"), model.layout.shoulder_type);
    model.layout.elbow_type =
        ReadString(layoutObject, QStringLiteral("elbow_type"), model.layout.elbow_type);
    model.layout.wrist_type =
        ReadString(layoutObject, QStringLiteral("wrist_type"), model.layout.wrist_type);
    model.layout.wrist_intersection =
        ReadBool(layoutObject, QStringLiteral("wrist_intersection"), model.layout.wrist_intersection);
    model.layout.wrist_offset =
        ReadBool(layoutObject, QStringLiteral("wrist_offset"), model.layout.wrist_offset);
    model.layout.internal_routing_required =
        ReadBool(
            layoutObject,
            QStringLiteral("internal_routing_required"),
            model.layout.internal_routing_required);
    model.layout.hollow_joint_ids = ReadStringArray(
        layoutObject.value(QStringLiteral("hollow_joint_ids")).toArray());
    model.layout.hollow_wrist_required =
        ReadBool(
            layoutObject,
            QStringLiteral("hollow_wrist_required"),
            model.layout.hollow_wrist_required);
    model.layout.reserved_channel_diameter_mm =
        ReadDouble(
            layoutObject,
            QStringLiteral("reserved_channel_diameter_mm"),
            model.layout.reserved_channel_diameter_mm);
    model.layout.seventh_axis_reserved =
        ReadBool(
            layoutObject,
            QStringLiteral("seventh_axis_reserved"),
            model.layout.seventh_axis_reserved);

    model.axis_relations.clear();
    const QJsonArray axisRelationsArray = jsonObject.value(QStringLiteral("axis_relations")).toArray();
    for (const QJsonValue& relationValue : axisRelationsArray)
    {
        const QJsonObject relationObject = relationValue.toObject();
        const QJsonArray jointPairArray = relationObject.value(QStringLiteral("joint_pair")).toArray();
        RoboSDP::Topology::Dto::TopologyAxisRelationDto relation;
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

    const QJsonArray jointsGraphArray =
        topologyGraphObject.value(QStringLiteral("joints_graph")).toArray();
    for (const QJsonValue& graphJointValue : jointsGraphArray)
    {
        const QJsonObject graphJointObject = graphJointValue.toObject();
        model.topology_graph.joints_graph.push_back({
            ReadString(graphJointObject, QStringLiteral("joint_id")),
            ReadString(graphJointObject, QStringLiteral("parent_link_id")),
            ReadString(graphJointObject, QStringLiteral("child_link_id"))});
    }

    return model;
}

} // namespace RoboSDP::Topology::Persistence
