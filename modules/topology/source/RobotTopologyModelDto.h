#pragma once

#include <QString>

#include <array>
#include <vector>

namespace RoboSDP::Topology::Dto
{

/**
 * @brief Topology 元信息 DTO。
 * 保存构型对象的标识、来源、状态以及 Requirement 关联信息。
 */
struct TopologyMetaDto
{
    QString topology_id = QStringLiteral("topology_001");
    QString name = QStringLiteral("6R 串联构型");
    int version = 1;
    QString source = QStringLiteral("template");
    QString status = QStringLiteral("draft");
    QString remarks;
    QString template_id;
    QString requirement_ref;
};

/**
 * @brief 机器人定义与基座安装 DTO。
 * 第一阶段仅支持模板化 6R 串联机器人，新增核心运动学尺寸（DH参数基础）。
 */
struct RobotDefinitionDto
{
    QString robot_type = QStringLiteral("6R_serial");
    int joint_count = 6;
    std::vector<QString> application_tags;
    
    QString base_mount_type = QStringLiteral("floor");
    std::array<double, 3> base_orientation {0.0, 0.0, 0.0};
    std::array<double, 2> j1_rotation_range_deg {-185.0, 185.0};

    // --- 新增：核心运动学尺寸 (DH 关键参数) ---
    double base_height_m = 0.35;         // d1: 基座高度
    double shoulder_offset_m = 0.10;     // a1: 肩部偏置
    double upper_arm_length_m = 0.40;    // a2: 大臂长度
    double forearm_length_m = 0.35;      // d4: 小臂延伸长度
    double wrist_offset_m = 0.10;        // d6: 腕部法兰偏置
};

/**
 * @brief 单个关节定义 DTO。
 * 保存关节角色、父子连杆关系和基础运动范围，供 Kinematics 复用。
 */
struct TopologyJointDto
{
    QString joint_id;
    int axis_index = 0;
    QString role;
    QString joint_type = QStringLiteral("revolute");
    QString parent_link_id;
    QString child_link_id;
    std::array<double, 3> axis_direction {0.0, 0.0, 1.0};
    std::array<double, 2> motion_range_deg {-180.0, 180.0};
};

/**
 * @brief 走线预留与附加机械配置 DTO。
 * 已精简无用的拓扑文本描述字段，仅保留走线与轴扩展属性。
 */
struct TopologyLayoutDto
{
    bool internal_routing_required = false;
    std::vector<QString> hollow_joint_ids;
    bool hollow_wrist_required = false;
    double reserved_channel_diameter_mm = 0.0;
    bool seventh_axis_reserved = false;
};

/**
 * @brief 关节轴线关系 DTO。
 * 当前只保留关节对与关系类型，作为最小编辑入口的数据承载。
 */
struct TopologyAxisRelationDto
{
    std::array<QString, 2> joint_pair;
    QString relation_type;
};

/// 拓扑图中的连杆节点 DTO。
struct TopologyGraphLinkDto
{
    QString link_id;
    QString parent_joint_id;
    QString name;
};

/// 拓扑图中的关节边 DTO。
struct TopologyGraphJointDto
{
    QString joint_id;
    QString parent_link_id;
    QString child_link_id;
};

/// 拓扑图 DTO。
struct TopologyGraphDto
{
    std::vector<TopologyGraphLinkDto> links;
    std::vector<TopologyGraphJointDto> joints_graph;
};

/**
 * @brief Topology 模块主 DTO。
 * 第一阶段只覆盖模板化 6R 串联骨架的最小闭环。
 */
struct RobotTopologyModelDto
{
    TopologyMetaDto meta;
    RobotDefinitionDto robot_definition;
    TopologyLayoutDto layout;
    std::vector<TopologyJointDto> joints;
    std::vector<TopologyAxisRelationDto> axis_relations;
    TopologyGraphDto topology_graph;

/// 创建可直接进入页面编辑与保存的默认 6R 串联构型骨架。
    static RobotTopologyModelDto CreateDefault()
    {
        RobotTopologyModelDto dto;
        dto.topology_graph.links = {
            {QStringLiteral("base_link"), QString(), QStringLiteral("基座")},
            {QStringLiteral("link_1"), QStringLiteral("joint_1"), QStringLiteral("大臂回转连杆")},
            {QStringLiteral("link_2"), QStringLiteral("joint_2"), QStringLiteral("肩部连杆")},
            {QStringLiteral("link_3"), QStringLiteral("joint_3"), QStringLiteral("肘部连杆")},
            {QStringLiteral("link_4"), QStringLiteral("joint_4"), QStringLiteral("腕部连杆 1")},
            {QStringLiteral("link_5"), QStringLiteral("joint_5"), QStringLiteral("腕部连杆 2")},
            {QStringLiteral("tool_link"), QStringLiteral("joint_6"), QStringLiteral("法兰末端")}
        };

        // 默认关节与角色映射保持不变，后台直接使用，不再让用户在界面编辑
        dto.joints = {
            {QStringLiteral("joint_1"), 1, QStringLiteral("base"), QStringLiteral("revolute"), QStringLiteral("base_link"), QStringLiteral("link_1"), {0.0, 0.0, 1.0}, {-185.0, 185.0}},
            {QStringLiteral("joint_2"), 2, QStringLiteral("shoulder"), QStringLiteral("revolute"), QStringLiteral("link_1"), QStringLiteral("link_2"), {0.0, 1.0, 0.0}, {-135.0, 85.0}},
            {QStringLiteral("joint_3"), 3, QStringLiteral("elbow"), QStringLiteral("revolute"), QStringLiteral("link_2"), QStringLiteral("link_3"), {0.0, 1.0, 0.0}, {-150.0, 150.0}},
            {QStringLiteral("joint_4"), 4, QStringLiteral("wrist_roll"), QStringLiteral("revolute"), QStringLiteral("link_3"), QStringLiteral("link_4"), {1.0, 0.0, 0.0}, {-200.0, 200.0}},
            {QStringLiteral("joint_5"), 5, QStringLiteral("wrist_pitch"), QStringLiteral("revolute"), QStringLiteral("link_4"), QStringLiteral("link_5"), {0.0, 1.0, 0.0}, {-125.0, 125.0}},
            {QStringLiteral("joint_6"), 6, QStringLiteral("wrist_yaw"), QStringLiteral("revolute"), QStringLiteral("link_5"), QStringLiteral("tool_link"), {1.0, 0.0, 0.0}, {-360.0, 360.0}}
        };

        // 走线预留
        dto.layout.hollow_joint_ids = {
            QStringLiteral("joint_4"),
            QStringLiteral("joint_5"),
            QStringLiteral("joint_6")
        };

        // 轴线约束规则后台写死，不再通过 UI 输入
        dto.axis_relations = {
            {{QStringLiteral("joint_2"), QStringLiteral("joint_3")}, QStringLiteral("parallel")},
            {{QStringLiteral("joint_4"), QStringLiteral("joint_5")}, QStringLiteral("perpendicular")}
        };

        dto.topology_graph.joints_graph = {
            {QStringLiteral("joint_1"), QStringLiteral("base_link"), QStringLiteral("link_1")},
            {QStringLiteral("joint_2"), QStringLiteral("link_1"), QStringLiteral("link_2")},
            {QStringLiteral("joint_3"), QStringLiteral("link_2"), QStringLiteral("link_3")},
            {QStringLiteral("joint_4"), QStringLiteral("link_3"), QStringLiteral("link_4")},
            {QStringLiteral("joint_5"), QStringLiteral("link_4"), QStringLiteral("link_5")},
            {QStringLiteral("joint_6"), QStringLiteral("link_5"), QStringLiteral("tool_link")}
        };

        return dto;
    }
};

} // namespace RoboSDP::Topology::Dto
