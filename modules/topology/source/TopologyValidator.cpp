#include "modules/topology/validator/TopologyValidator.h"

#include <QSet>

namespace RoboSDP::Topology::Validation
{

namespace
{

void AddIssue(
    TopologyValidationResult& result,
    const QString& field,
    const QString& code,
    const QString& message,
    ValidationSeverity severity = ValidationSeverity::Error)
{
    result.issues.push_back({field, code, message, severity});
}

bool IsSupportedSource(const QString& value)
{
    static const QSet<QString> kSupportedValues {
        QStringLiteral("manual"),
        QStringLiteral("template"),
        QStringLiteral("imported")
    };
    return kSupportedValues.contains(value);
}

bool IsSupportedStatus(const QString& value)
{
    static const QSet<QString> kSupportedValues {
        QStringLiteral("draft"),
        QStringLiteral("ready"),
        QStringLiteral("invalid"),
        QStringLiteral("archived")
    };
    return kSupportedValues.contains(value);
}

bool IsSupportedBaseMount(const QString& value)
{
    static const QSet<QString> kSupportedValues {
        QStringLiteral("floor"),
        QStringLiteral("wall"),
        QStringLiteral("ceiling"),
        QStringLiteral("pedestal")
    };
    return kSupportedValues.contains(value);
}

bool IsSupportedRelationType(const QString& value)
{
    static const QSet<QString> kSupportedValues {
        QStringLiteral("parallel"),
        QStringLiteral("perpendicular"),
        QStringLiteral("intersecting"),
        QStringLiteral("offset"),
        QStringLiteral("coplanar")
    };
    return kSupportedValues.contains(value);
}

} // namespace

bool TopologyValidationResult::IsValid() const
{
    return ErrorCount() == 0;
}

int TopologyValidationResult::ErrorCount() const
{
    int count = 0;
    for (const ValidationIssue& issue : issues)
    {
        if (issue.severity == ValidationSeverity::Error)
        {
            ++count;
        }
    }

    return count;
}

int TopologyValidationResult::WarningCount() const
{
    int count = 0;
    for (const ValidationIssue& issue : issues)
    {
        if (issue.severity == ValidationSeverity::Warning)
        {
            ++count;
        }
    }

    return count;
}

QString ToString(ValidationSeverity severity)
{
    switch (severity)
    {
    case ValidationSeverity::Info:
        return QStringLiteral("INFO");
    case ValidationSeverity::Warning:
        return QStringLiteral("WARNING");
    case ValidationSeverity::Error:
        return QStringLiteral("ERROR");
    }

    return QStringLiteral("ERROR");
}

TopologyValidationResult TopologyValidator::Validate(
    const RoboSDP::Topology::Dto::RobotTopologyModelDto& model) const
{
    TopologyValidationResult result;

    if (model.meta.topology_id.trimmed().isEmpty())
    {
        AddIssue(
            result,
            QStringLiteral("meta.topology_id"),
            QStringLiteral("TOPOLOGY_ID_REQUIRED"),
            QStringLiteral("构型 ID 不能为空。"));
    }

    if (model.meta.name.trimmed().isEmpty())
    {
        AddIssue(
            result,
            QStringLiteral("meta.name"),
            QStringLiteral("TOPOLOGY_NAME_REQUIRED"),
            QStringLiteral("构型名称不能为空。"));
    }

    if (!IsSupportedSource(model.meta.source))
    {
        AddIssue(
            result,
            QStringLiteral("meta.source"),
            QStringLiteral("TOPOLOGY_SOURCE_INVALID"),
            QStringLiteral("构型来源不在允许枚举范围内。"));
    }

    if (!IsSupportedStatus(model.meta.status))
    {
        AddIssue(
            result,
            QStringLiteral("meta.status"),
            QStringLiteral("TOPOLOGY_STATUS_INVALID"),
            QStringLiteral("构型状态不在允许枚举范围内。"));
    }

    if (model.meta.source == QStringLiteral("template") && model.meta.template_id.trimmed().isEmpty())
    {
        AddIssue(
            result,
            QStringLiteral("meta.template_id"),
            QStringLiteral("TOPOLOGY_TEMPLATE_ID_EMPTY"),
            QStringLiteral("模板生成的构型应保留模板 ID。"),
            ValidationSeverity::Warning);
    }

    if (model.robot_definition.robot_type != QStringLiteral("6R_serial"))
    {
        AddIssue(
            result,
            QStringLiteral("robot_definition.robot_type"),
            QStringLiteral("TOPOLOGY_ROBOT_TYPE_UNSUPPORTED"),
            QStringLiteral("当前阶段仅支持 6R_serial 模板化串联构型。"));
    }

    if (model.robot_definition.joint_count != 6)
    {
        AddIssue(
            result,
            QStringLiteral("robot_definition.joint_count"),
            QStringLiteral("TOPOLOGY_JOINT_COUNT_INVALID"),
            QStringLiteral("当前阶段关节数必须固定为 6。"));
    }

    if (!IsSupportedBaseMount(model.robot_definition.base_mount_type))
    {
        AddIssue(
            result,
            QStringLiteral("robot_definition.base_mount_type"),
            QStringLiteral("TOPOLOGY_BASE_MOUNT_INVALID"),
            QStringLiteral("基座安装方式不在允许枚举范围内。"));
    }

    if (model.robot_definition.base_height_m < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("robot_definition.base_height_m"),
            QStringLiteral("TOPOLOGY_BASE_HEIGHT_NEGATIVE"),
            QStringLiteral("基座高度不能为负数。"));
    }

    if (model.robot_definition.j1_rotation_range_deg[0] >= model.robot_definition.j1_rotation_range_deg[1])
    {
        AddIssue(
            result,
            QStringLiteral("robot_definition.j1_rotation_range_deg"),
            QStringLiteral("TOPOLOGY_J1_RANGE_INVALID"),
            QStringLiteral("J1 旋转范围必须满足最小值小于最大值。"));
    }
// ---------- 新增：DH关键尺寸校验 ----------
    if (model.robot_definition.shoulder_offset_m < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("robot_definition.shoulder_offset_m"),
            QStringLiteral("TOPOLOGY_SHOULDER_OFFSET_NEGATIVE"),
            QStringLiteral("肩部偏置不能为负数。"));
    }

    if (model.robot_definition.upper_arm_length_m <= 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("robot_definition.upper_arm_length_m"),
            QStringLiteral("TOPOLOGY_UPPER_ARM_INVALID"),
            QStringLiteral("大臂长度必须大于 0。"));
    }

    if (model.robot_definition.forearm_length_m <= 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("robot_definition.forearm_length_m"),
            QStringLiteral("TOPOLOGY_FOREARM_INVALID"),
            QStringLiteral("小臂长度必须大于 0。"));
    }

    if (model.robot_definition.wrist_offset_m < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("robot_definition.wrist_offset_m"),
            QStringLiteral("TOPOLOGY_WRIST_OFFSET_NEGATIVE"),
            QStringLiteral("腕部法兰偏置不能为负数。"));
    }

    if (model.layout.reserved_channel_diameter_mm < 0.0)
    {
        AddIssue(
            result,
            QStringLiteral("layout.reserved_channel_diameter_mm"),
            QStringLiteral("TOPOLOGY_RESERVED_CHANNEL_NEGATIVE"),
            QStringLiteral("预留通道直径不能为负数。"));
    }

    if (static_cast<int>(model.joints.size()) != model.robot_definition.joint_count)
    {
        AddIssue(
            result,
            QStringLiteral("joints"),
            QStringLiteral("TOPOLOGY_JOINT_SIZE_MISMATCH"),
            QStringLiteral("关节集合数量必须与 joint_count 一致。"));
    }

    if (model.topology_graph.links.size() < 2)
    {
        AddIssue(
            result,
            QStringLiteral("topology_graph.links"),
            QStringLiteral("TOPOLOGY_LINKS_TOO_FEW"),
            QStringLiteral("拓扑图中的连杆数量至少为 2。"));
    }

    if (static_cast<int>(model.topology_graph.joints_graph.size()) != model.robot_definition.joint_count)
    {
        AddIssue(
            result,
            QStringLiteral("topology_graph.joints_graph"),
            QStringLiteral("TOPOLOGY_GRAPH_JOINT_SIZE_MISMATCH"),
            QStringLiteral("拓扑图关节数量必须与 joint_count 一致。"));
    }

    QSet<QString> jointIds;
    for (std::size_t index = 0; index < model.joints.size(); ++index)
    {
        const auto& joint = model.joints.at(index);
        const QString prefix = QStringLiteral("joints[%1]").arg(index);

        if (joint.joint_id.trimmed().isEmpty())
        {
            AddIssue(
                result,
                prefix + QStringLiteral(".joint_id"),
                QStringLiteral("TOPOLOGY_JOINT_ID_REQUIRED"),
                QStringLiteral("关节 ID 不能为空。"));
        }
        else if (jointIds.contains(joint.joint_id))
        {
            AddIssue(
                result,
                prefix + QStringLiteral(".joint_id"),
                QStringLiteral("TOPOLOGY_JOINT_ID_DUPLICATED"),
                QStringLiteral("关节 ID 必须唯一。"));
        }
        else
        {
            jointIds.insert(joint.joint_id);
        }

        if (joint.axis_index <= 0)
        {
            AddIssue(
                result,
                prefix + QStringLiteral(".axis_index"),
                QStringLiteral("TOPOLOGY_AXIS_INDEX_INVALID"),
                QStringLiteral("关节序号必须大于 0。"));
        }

        if (joint.role.trimmed().isEmpty())
        {
            AddIssue(
                result,
                prefix + QStringLiteral(".role"),
                QStringLiteral("TOPOLOGY_JOINT_ROLE_REQUIRED"),
                QStringLiteral("关节功能角色不能为空。"));
        }

        if (joint.parent_link_id.trimmed().isEmpty() || joint.child_link_id.trimmed().isEmpty())
        {
            AddIssue(
                result,
                prefix + QStringLiteral(".parent_child"),
                QStringLiteral("TOPOLOGY_PARENT_CHILD_REQUIRED"),
                QStringLiteral("关节父子连杆 ID 不能为空。"));
        }

        if (joint.motion_range_deg[0] >= joint.motion_range_deg[1])
        {
            AddIssue(
                result,
                prefix + QStringLiteral(".motion_range_deg"),
                QStringLiteral("TOPOLOGY_MOTION_RANGE_INVALID"),
                QStringLiteral("关节运动范围必须满足最小值小于最大值。"));
        }
    }

    for (std::size_t index = 0; index < model.layout.hollow_joint_ids.size(); ++index)
    {
        const QString& jointId = model.layout.hollow_joint_ids.at(index);
        if (!jointId.trimmed().isEmpty() && !jointIds.contains(jointId))
        {
            AddIssue(
                result,
                QStringLiteral("layout.hollow_joint_ids[%1]").arg(index),
                QStringLiteral("TOPOLOGY_HOLLOW_JOINT_NOT_FOUND"),
                QStringLiteral("中空关节 ID 必须存在于 joints 集合中。"));
        }
    }

    for (std::size_t index = 0; index < model.axis_relations.size(); ++index)
    {
        const auto& relation = model.axis_relations.at(index);
        const QString prefix = QStringLiteral("axis_relations[%1]").arg(index);

        if (relation.joint_pair[0].trimmed().isEmpty() || relation.joint_pair[1].trimmed().isEmpty())
        {
            AddIssue(
                result,
                prefix + QStringLiteral(".joint_pair"),
                QStringLiteral("TOPOLOGY_AXIS_RELATION_JOINT_PAIR_REQUIRED"),
                QStringLiteral("轴线关系中的关节对不能为空。"));
        }
        else
        {
            if (!jointIds.contains(relation.joint_pair[0]))
            {
                AddIssue(
                    result,
                    prefix + QStringLiteral(".joint_pair[0]"),
                    QStringLiteral("TOPOLOGY_AXIS_RELATION_FIRST_JOINT_NOT_FOUND"),
                    QStringLiteral("轴线关系中的第一个关节必须存在于 joints 集合中。"));
            }
            if (!jointIds.contains(relation.joint_pair[1]))
            {
                AddIssue(
                    result,
                    prefix + QStringLiteral(".joint_pair[1]"),
                    QStringLiteral("TOPOLOGY_AXIS_RELATION_SECOND_JOINT_NOT_FOUND"),
                    QStringLiteral("轴线关系中的第二个关节必须存在于 joints 集合中。"));
            }
            if (relation.joint_pair[0] == relation.joint_pair[1])
            {
                AddIssue(
                    result,
                    prefix + QStringLiteral(".joint_pair"),
                    QStringLiteral("TOPOLOGY_AXIS_RELATION_JOINT_PAIR_DUPLICATED"),
                    QStringLiteral("轴线关系中的两个关节不能相同。"));
            }
        }

        if (!IsSupportedRelationType(relation.relation_type))
        {
            AddIssue(
                result,
                prefix + QStringLiteral(".relation_type"),
                QStringLiteral("TOPOLOGY_AXIS_RELATION_TYPE_INVALID"),
                QStringLiteral("轴线关系类型不在允许枚举范围内。"));
        }
    }

    QSet<QString> linkIds;
    for (std::size_t index = 0; index < model.topology_graph.links.size(); ++index)
    {
        const auto& link = model.topology_graph.links.at(index);
        const QString prefix = QStringLiteral("topology_graph.links[%1]").arg(index);

        if (link.link_id.trimmed().isEmpty())
        {
            AddIssue(
                result,
                prefix + QStringLiteral(".link_id"),
                QStringLiteral("TOPOLOGY_LINK_ID_REQUIRED"),
                QStringLiteral("连杆 ID 不能为空。"));
        }
        else if (linkIds.contains(link.link_id))
        {
            AddIssue(
                result,
                prefix + QStringLiteral(".link_id"),
                QStringLiteral("TOPOLOGY_LINK_ID_DUPLICATED"),
                QStringLiteral("连杆 ID 必须唯一。"));
        }
        else
        {
            linkIds.insert(link.link_id);
        }
    }

    for (std::size_t index = 0; index < model.topology_graph.joints_graph.size(); ++index)
    {
        const auto& graphJoint = model.topology_graph.joints_graph.at(index);
        const QString prefix = QStringLiteral("topology_graph.joints_graph[%1]").arg(index);

        if (!jointIds.contains(graphJoint.joint_id))
        {
            AddIssue(
                result,
                prefix + QStringLiteral(".joint_id"),
                QStringLiteral("TOPOLOGY_GRAPH_JOINT_NOT_FOUND"),
                QStringLiteral("拓扑图中的关节 ID 必须存在于 joints 集合中。"));
        }

        if (!linkIds.contains(graphJoint.parent_link_id) || !linkIds.contains(graphJoint.child_link_id))
        {
            AddIssue(
                result,
                prefix + QStringLiteral(".parent_child_link"),
                QStringLiteral("TOPOLOGY_GRAPH_LINK_NOT_FOUND"),
                QStringLiteral("拓扑图中的父子连杆 ID 必须存在于 links 集合中。"));
        }
    }

    return result;
}

} // namespace RoboSDP::Topology::Validation
