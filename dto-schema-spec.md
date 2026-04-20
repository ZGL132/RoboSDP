# 机械臂设计与仿真验证软件 DTO / Schema 规格表

**文档类型**：字段级 DTO / Schema 规格表  
**适用对象**：架构师、Qt 开发、后台服务开发、Codex 代码生成、测试工程师  
**适用范围**：第一阶段主链模块，兼顾二期扩展预留

---

## 1. 编写说明

本文档用于把详细设计文档中的核心对象进一步展开为**字段级 DTO / Schema 规格表**，用于：

1. 指导 Codex 生成 `dto/`、`model/`、`schema/`、`validator/` 代码；
2. 指导 Qt 右侧属性面板与结果详情窗口的字段绑定；
3. 指导 JSON 持久化结构设计；
4. 指导单元测试、接口测试和数据迁移。

### 1.1 字段列说明

- **字段名**：建议的 JSON / DTO 字段名，统一英文命名；
- **类型**：建议类型；
- **单位**：若适用则给出标准单位；
- **必填**：Y / N；
- **来源**：manual / imported / estimated / computed；
- **去向**：主要消费模块；
- **校验规则**：字段合法性和范围规则；
- **中文说明**：必须体现在代码注释或 schema 描述中。

### 1.2 通用类型约定

| 类型 | 说明 |
|---|---|
| string | 字符串 |
| integer | 整数 |
| number | 浮点数 |
| boolean | 布尔 |
| enum[...] | 枚举 |
| object | 对象 |
| array[T] | T 数组 |
| vec3 | 三维向量 `[x, y, z]` |
| inertia6 | 惯量六元组 `[ixx, iyy, izz, ixy, ixz, iyz]` |
| pose6 | 六维位姿 `[x, y, z, rx, ry, rz]` |
| range2 | 范围 `[min, max]` |

### 1.3 通用字段基类建议

所有核心对象建议继承以下元字段：

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| id | string | - | Y | computed/manual | 全模块 | 唯一，不可为空 | 对象唯一标识 |
| name | string | - | Y | manual | UI/导出 | 长度 1~128 | 对象名称 |
| version | integer | - | Y | computed | 持久化/迁移 | `>=1` | 对象版本号 |
| created_at | string | ISO8601 | Y | computed | 审计/快照 | 合法时间格式 | 创建时间 |
| updated_at | string | ISO8601 | Y | computed | 审计/快照 | 合法时间格式 | 更新时间 |
| source | enum[manual,imported,estimated,computed] | - | Y | computed | 全模块 | 枚举合法 | 数据来源 |
| status | enum[draft,ready,invalid,archived] | - | Y | computed | UI/流程控制 | 枚举合法 | 数据状态 |
| remarks | string | - | N | manual | UI/导出 | 可为空 | 备注 |

---

## 2. RequirementModel 规格表

### 2.1 project_meta

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| project_name | string | - | Y | manual | 全模块 | 1~128 字符 | 项目名称 |
| scenario_type | enum[handling,welding,assembly,grinding,spraying,custom] | - | Y | manual | 构型/规划 | 枚举合法 | 应用场景类型 |
| description | string | - | N | manual | 导出 | 可为空 | 项目描述 |

### 2.2 load_requirements

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| rated_payload | number | kg | Y | manual | 动力学/选型 | `>=0` | 额定负载 |
| max_payload | number | kg | Y | manual | 动力学/选型 | `>= rated_payload` | 最大负载 |
| tool_mass | number | kg | Y | manual | 动力学/工具 | `>=0` | 工具质量 |
| fixture_mass | number | kg | N | manual | 动力学 | `>=0` | 工装质量 |
| payload_cog | vec3 | m | Y | manual | 动力学/偏载 | 长度=3 | 负载质心位置 |
| payload_inertia | inertia6 | kg·m² | N | manual/imported/estimated | 动力学 | 长度=6 | 负载惯量 |
| off_center_load | boolean | - | Y | manual | 动力学/风险 | - | 是否偏载 |
| cable_drag_load | number | N | N | manual | 动力学 | `>=0` | 线缆拖曳附加载荷 |
| load_variants | array[object] | - | N | manual/computed | 动力学 | 可为空 | 负载工况变体集合 |

#### load_variants.item

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| variant_id | string | - | Y | computed/manual | 动力学 | 唯一 | 负载变体编号 |
| name | string | - | Y | manual | UI | 非空 | 变体名称 |
| total_mass | number | kg | Y | computed/manual | 动力学 | `>=0` | 总质量 |
| cog | vec3 | m | Y | manual | 动力学 | 长度=3 | 总质心 |
| inertia | inertia6 | kg·m² | N | manual/estimated | 动力学 | 长度=6 | 总惯量 |
| remark | string | - | N | manual | UI | 可空 | 变体说明 |

### 2.3 workspace_requirements

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| max_radius | number | m | Y | manual | 构型/运动学 | `>0` | 最大工作半径 |
| min_radius | number | m | N | manual | 构型/运动学 | `>=0` 且 `< max_radius` | 最小工作半径 |
| max_height | number | m | Y | manual | 构型/运动学 | `> min_height` | 最大工作高度 |
| min_height | number | m | N | manual | 构型/运动学 | `< max_height` | 最小工作高度 |
| key_poses | array[object] | - | Y | manual | 运动学/规划 | 至少 1 项 | 关键工位集合 |
| forbidden_regions | array[object] | - | N | manual | 规划 | 可空 | 禁入区域 |
| obstacle_regions | array[object] | - | N | manual | 规划 | 可空 | 障碍物区域 |
| base_constraints | object | - | N | manual | 构型 | 可空 | 基座安装约束 |

#### key_poses.item

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| pose_id | string | - | Y | computed/manual | 全模块 | 唯一 | 工位 ID |
| name | string | - | Y | manual | UI | 非空 | 工位名称 |
| pose | pose6 | m/deg | Y | manual | 运动学/规划 | 长度=6 | 工位位姿 |
| position_tol | number | mm | N | manual | 精度/运动学 | `>=0` | 位置容限 |
| orientation_tol | number | deg | N | manual | 精度/运动学 | `>=0` | 姿态容限 |
| required_direction | vec3 | - | N | manual | 运动学 | 长度=3 | 工具方向要求 |

### 2.4 motion_requirements

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| max_linear_speed | number | m/s | Y | manual | 运动学/规划 | `>0` | 最大线速度 |
| max_angular_speed | number | deg/s | Y | manual | 运动学/规划 | `>0` | 最大角速度 |
| max_acceleration | number | m/s² | Y | manual | 动力学/规划 | `>0` | 最大线加速度 |
| max_angular_acceleration | number | deg/s² | Y | manual | 动力学/规划 | `>0` | 最大角加速度 |
| jerk_limit | number | m/s³ | N | manual | 规划 | `>0` | jerk 限制 |
| takt_time | number | s | Y | manual | 动力学/规划 | `>0` | 节拍时间 |

### 2.5 accuracy_requirements / reliability_requirements / derived_conditions

简化保留为主字段：

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| absolute_accuracy | number | mm | N | manual | 精度 | `>=0` | 绝对定位精度 |
| repeatability | number | mm | N | manual | 精度 | `>=0` | 重复定位精度 |
| tracking_accuracy | number | mm | N | manual | 精度/规划 | `>=0` | 轨迹跟踪精度 |
| orientation_accuracy | number | deg | N | manual | 精度 | `>=0` | 姿态精度 |
| tcp_position_tol | number | mm | N | manual | 精度/运动学 | `>=0` | TCP 位置容限 |
| tcp_orientation_tol | number | deg | N | manual | 精度/运动学 | `>=0` | TCP 姿态容限 |
| design_life | number | h | N | manual | 可靠性/选型 | `>0` | 设计寿命 |
| cycle_count | integer | 次 | N | manual | 可靠性 | `>=0` | 循环次数 |
| duty_cycle | number | 0~1 | N | manual | 动力学/可靠性 | `0<=x<=1` | 占空比 |
| operating_hours_per_day | number | h | N | manual | 可靠性 | `0<=x<=24` | 每日运行时长 |
| mtbf_target | number | h | N | manual | 可靠性 | `>0` | MTBF 目标 |
| rated_case | object | - | N | computed | 动力学 | - | 额定工况 |
| peak_case | object | - | N | computed | 动力学 | - | 峰值工况 |
| continuous_case | object | - | N | computed | 动力学 | - | 连续工况 |
| extreme_case | object | - | N | computed | 动力学 | - | 极限工况 |
| selection_benchmark_case | object | - | N | computed | 选型 | - | 选型基准工况 |

---

## 3. RobotTopologyModel 规格表

### 3.1 robot_definition / base_mount

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| robot_type | enum[6R_serial] | - | Y | computed/manual | 全模块 | 当前固定 6R 串联 | 机器人类型 |
| chain_type | enum[serial] | - | Y | computed | 全模块 | 当前固定 serial | 链型 |
| dof | integer | - | Y | computed/manual | 全模块 | 当前固定 6 | 自由度数量 |
| joint_count | integer | - | Y | computed | 全模块 | 与 joints 数一致 | 关节数 |
| application_tags | array[string] | - | N | manual/computed | UI/推荐 | 可空 | 适用场景标签 |
| base_mount_type | enum[floor,wall,ceiling,pedestal] | - | Y | manual | 构型/规划 | 枚举合法 | 基座安装方式 |
| base_height_m | number | m | N | manual | 构型/运动学 | `>=0` | 基座高度 |
| base_orientation | array[number] | deg | N | manual | 构型/运动学 | 长度=3 | 基座姿态 |
| j1_rotation_range_deg | range2 | deg | N | manual | 运动学 | min<max | J1 回转范围 |

### 3.2 joint_roles / joints

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| joint_id | string | - | Y | computed | 全模块 | 唯一 | 关节 ID |
| role | string | - | Y | manual/computed | UI/算法 | 非空 | 关节功能角色 |
| functional_group | enum[base,arm,wrist] | - | Y | computed | UI/规则 | 枚举合法 | 功能分组 |
| joint_type | enum[revolute,prismatic] | - | Y | manual/computed | 运动学 | 枚举合法 | 关节类型 |
| axis_direction | vec3 | - | Y | manual/computed | 运动学 | 长度=3，不能全零 | 关节轴方向 |
| range | range2 | deg/mm | Y | manual | 运动学 | min<max | 关节行程范围 |
| max_velocity | number | deg/s,mm/s | N | manual | 运动学/规划 | `>0` | 最大速度 |
| max_acceleration | number | deg/s²,mm/s² | N | manual | 运动学/规划 | `>0` | 最大加速度 |
| hollow | boolean | - | N | manual | 结构 | - | 是否中空 |
| parent_link_id | string | - | Y | computed | 运动学 | 非空 | 父连杆 ID |
| child_link_id | string | - | Y | computed | 运动学 | 非空 | 子连杆 ID |

### 3.3 layout / axis_relations / routing_reservation / topology_graph

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| shoulder_type | string | - | Y | manual/computed | 运动学/结构 | 非空 | 肩部形式 |
| elbow_type | string | - | Y | manual/computed | 运动学/结构 | 非空 | 肘部形式 |
| wrist_type | string | - | Y | manual/computed | 运动学/结构 | 非空 | 腕部形式 |
| wrist_intersection | boolean | - | N | manual/computed | 运动学 | - | 腕部交点式 |
| wrist_offset | boolean | - | N | manual/computed | 运动学 | - | 腕部偏置 |
| joint_pair | array[string] | - | Y | computed | 运动学 | 长度=2 | 关节对 |
| relation_type | enum[parallel,perpendicular,intersecting,offset,coplanar] | - | Y | manual/computed | 运动学 | 枚举合法 | 轴线关系 |
| internal_routing_required | boolean | - | N | manual | 结构/工具 | - | 是否需要内部走线 |
| hollow_joint_ids | array[string] | - | N | manual | 结构 | 可空 | 中空关节集合 |
| hollow_wrist_required | boolean | - | N | manual | 结构 | - | 是否中空腕 |
| reserved_channel_diameter_mm | number | mm | N | manual | 结构 | `>=0` | 预留通道直径 |
| links | array[object] | - | Y | computed | 运动学 | 至少 2 项 | 连杆拓扑集合 |
| joints_graph | array[object] | - | Y | computed | 运动学 | 与 joints 对应 | 拓扑图关节集合 |

---

## 4. KinematicModel 规格表

### 4.1 基础字段

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| modeling_mode | enum[DH,MDH,URDF] | - | Y | manual/imported | 全模块 | 枚举合法 | 建模模式 |
| base_frame | object | - | Y | manual/computed | 运动学/规划 | 必须存在 | 基坐标系 |
| flange_frame | object | - | Y | manual/computed | 工具/规划 | 必须存在 | 法兰坐标系 |
| tool_frame | object | - | N | manual | 工具/规划 | 可空 | 工具坐标系 |
| workpiece_frame | object | - | N | manual | 规划 | 可空 | 工件坐标系 |
| tcp_frame | object | - | Y | manual | 全模块 | 必须存在 | TCP 坐标系 |

### 4.2 links / joint_limits / solver / result refs

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| link_id | string | - | Y | computed | 全模块 | 唯一 | 连杆 ID |
| a | number | m | N | manual/computed | FK/IK | - | DH/MDH 参数 a |
| alpha | number | deg | N | manual/computed | FK/IK | - | 扭转角 |
| d | number | m | N | manual/computed | FK/IK | - | 偏距 |
| theta_offset | number | deg | N | manual/computed | FK/IK | - | 零位偏置 |
| soft_limit | range2 | deg/mm | N | manual | IK/规划 | min<max | 软限位 |
| hard_limit | range2 | deg/mm | Y | manual | IK/规划 | min<max | 硬限位 |
| max_velocity | number | deg/s | N | manual | 规划/动力学 | `>0` | 最大速度 |
| max_acceleration | number | deg/s² | N | manual | 规划/动力学 | `>0` | 最大加速度 |
| solver_type | string | - | N | manual | IK | 可空 | 求解器类型 |
| branch_policy | string | - | N | manual | IK | 可空 | 逆解分支策略 |
| jacobian_samples | array[object] | - | N | computed | 刚度/精度 | 可空 | 雅可比采样结果 |
| reachability_result | object | - | N | computed | UI/规划 | 可空 | 可达性结果引用 |
| singularity_map | object | - | N | computed | UI/规划 | 可空 | 奇异性地图引用 |

---

## 5. DynamicModel 规格表

### 5.1 基础字段

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| gravity | vec3 | m/s² | Y | manual/computed | 动力学 | 长度=3 | 重力向量 |
| installation_pose | object | - | N | manual | 动力学/规划 | 可空 | 安装姿态 |

### 5.2 links / joints / end_effector / trajectories / results

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| link_id | string | - | Y | computed | 动力学 | 唯一 | 连杆 ID |
| mass | number | kg | Y | manual/imported/estimated/computed | 动力学/选型 | `>=0` | 质量 |
| cog | vec3 | m | Y | manual/imported/estimated/computed | 动力学 | 长度=3 | 质心 |
| inertia_tensor | inertia6 | kg·m² | N | manual/imported/estimated/computed | 动力学 | 长度=6 | 惯量张量 |
| source_detail | string | - | N | computed | UI | 可空 | 物理参数来源细节 |
| transmission_ratio | number | - | N | manual/computed | 选型 | `>0` | 传动比 |
| efficiency | number | 0~1 | N | manual/computed | 选型 | `0<x<=1` | 效率 |
| friction.viscous | number | N·m·s/rad | N | manual | 动力学 | `>=0` | 粘性摩擦 |
| friction.coulomb | number | N·m | N | manual | 动力学 | `>=0` | 库仑摩擦 |
| friction.static | number | N·m | N | manual | 动力学 | `>=0` | 静摩擦 |
| end_effector.mass | number | kg | N | manual/imported | 动力学 | `>=0` | 末端工具质量 |
| trajectories | array[object] | - | N | manual/computed/imported | 动力学 | 可空 | 轨迹工况集合 |
| load_envelope | object | - | N | computed | 选型/结构 | 可空 | 负载包络 |
| torque_curves | array[object] | - | N | computed | UI/导出 | 可空 | 扭矩曲线 |
| power_curves | array[object] | - | N | computed | UI/导出 | 可空 | 功率曲线 |
| peak_stats | array[object] | - | N | computed | 选型 | 可空 | 峰值统计 |
| rms_stats | array[object] | - | N | computed | 选型 | 可空 | RMS 统计 |

---

## 6. MotorSelection / ReducerSelection / DriveTrain 规格表

### 6.1 MotorRequirementDto / MotorSelectionResult

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| joint_id | string | - | Y | computed | 电机选型 | 非空 | 关节 ID |
| output_peak_torque_nm | number | N·m | Y | computed | 电机选型 | `>=0` | 输出侧峰值扭矩 |
| output_rms_torque_nm | number | N·m | Y | computed | 电机选型 | `>=0` | 输出侧 RMS 扭矩 |
| reducer_ratio_init | number | - | N | manual/computed | 电机选型 | `>0` | 初始减速比 |
| transmission_efficiency | number | 0~1 | N | manual/computed | 电机选型 | `0<x<=1` | 传动效率 |
| motor_peak_torque_required_nm | number | N·m | Y | computed | 电机筛选 | `>=0` | 电机侧峰值需求 |
| motor_rms_torque_required_nm | number | N·m | Y | computed | 电机筛选 | `>=0` | 电机侧 RMS 需求 |
| motor_peak_speed_required_rpm | number | rpm | Y | computed | 电机筛选 | `>=0` | 电机侧峰值转速需求 |
| motor_peak_power_required_w | number | W | N | computed | 电机筛选 | `>=0` | 电机侧功率需求 |
| candidate_motors | array[object] | - | N | computed | UI/联合匹配 | 可空 | 候选电机 |
| recommended_motor | object | - | N | computed | 联合匹配/导出 | 可空 | 推荐电机 |
| peak_torque_check | enum[pass,warning,fail] | - | N | computed | UI | 枚举合法 | 峰值转矩校核 |
| rms_torque_check | enum[pass,warning,fail] | - | N | computed | UI | 枚举合法 | RMS 校核 |
| speed_check | enum[pass,warning,fail] | - | N | computed | UI | 枚举合法 | 转速校核 |
| power_check | enum[pass,warning,fail] | - | N | computed | UI | 枚举合法 | 功率校核 |
| inertia_match_check | enum[pass,warning,fail] | - | N | computed | UI | 枚举合法 | 惯量匹配校核 |
| thermal_check | enum[pass,warning,fail] | - | N | computed | UI | 枚举合法 | 热负荷校核 |
| brake_check | enum[pass,warning,fail,na] | - | N | computed | UI | 枚举合法 | 抱闸校核 |

### 6.2 ReducerRequirementDto / ReducerSelectionResult

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| joint_id | string | - | Y | computed | 减速器选型 | 非空 | 关节 ID |
| output_peak_torque_nm | number | N·m | Y | computed | 减速器选型 | `>=0` | 输出峰值扭矩 |
| output_rms_torque_nm | number | N·m | Y | computed | 减速器选型 | `>=0` | 输出 RMS 扭矩 |
| motor_max_speed_rpm | number | rpm | N | computed | 减速器选型 | `>=0` | 电机最大转速 |
| ratio_target_range | range2 | - | N | computed | 减速器选型 | min<max | 目标减速比区间 |
| backlash_limit_arcmin | number | arcmin | N | manual/computed | 减速器筛选 | `>=0` | 回差上限 |
| torsional_stiffness_min | number | N·m/arcmin | N | manual/computed | 减速器筛选 | `>=0` | 最小扭转刚度 |
| candidate_reducers | array[object] | - | N | computed | UI/联合匹配 | 可空 | 候选减速器 |
| recommended_reducer | object | - | N | computed | 联合匹配/导出 | 可空 | 推荐减速器 |
| rated_torque_check | enum[pass,warning,fail] | - | N | computed | UI | 枚举合法 | 额定转矩校核 |
| peak_torque_check | enum[pass,warning,fail] | - | N | computed | UI | 枚举合法 | 峰值转矩校核 |
| input_speed_check | enum[pass,warning,fail] | - | N | computed | UI | 枚举合法 | 输入转速校核 |
| backlash_check | enum[pass,warning,fail] | - | N | computed | UI | 枚举合法 | 回差校核 |
| stiffness_check | enum[pass,warning,fail] | - | N | computed | UI | 枚举合法 | 刚度校核 |
| life_check | enum[pass,warning,fail] | - | N | computed | UI | 枚举合法 | 寿命校核 |
| efficiency_check | enum[pass,warning,fail] | - | N | computed | UI | 枚举合法 | 效率校核 |
| shock_check | enum[pass,warning,fail] | - | N | computed | UI | 枚举合法 | 冲击校核 |

### 6.3 DriveTrainSelectionResult

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| joint_id | string | - | Y | computed | 导出/快照 | 非空 | 关节 ID |
| motor_selection_ref | string | - | Y | computed | 导出 | 非空 | 电机选型结果引用 |
| reducer_selection_ref | string | - | Y | computed | 导出 | 非空 | 减速器选型结果引用 |
| motor_model | string | - | Y | computed | UI/导出 | 非空 | 推荐电机型号 |
| reducer_model | string | - | Y | computed | UI/导出 | 非空 | 推荐减速器型号 |
| ratio | number | - | Y | computed | UI/导出 | `>0` | 推荐传动比 |
| brake_enabled | boolean | - | N | computed | UI/导出 | - | 是否带抱闸 |
| combined_mass_kg | number | kg | N | computed | 结构/导出 | `>=0` | 联合质量 |
| combined_score | number | 0~100 | N | computed | UI | `>=0` | 综合评分 |
| recommendation_reason | array[string] | - | N | computed | UI/导出 | 可空 | 推荐理由 |

---

## 7. PlanningScene / PlanningVerification 规格表

### 7.1 PlanningSceneModel

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| robot_collision_model_ref | string | - | Y | computed/imported | MoveIt | 非空 | 机器人碰撞模型引用 |
| environment_objects | array[object] | - | N | manual/imported | MoveIt | 可空 | 环境对象 |
| forbidden_regions | array[object] | - | N | manual | MoveIt/规则 | 可空 | 禁入区 |
| attached_objects | array[object] | - | N | manual/computed | MoveIt | 可空 | 附着物体 |
| path_constraints | array[object] | - | N | manual | MoveIt | 可空 | 路径约束 |
| orientation_constraints | array[object] | - | N | manual | MoveIt | 可空 | 姿态约束 |
| joint_constraints | array[object] | - | N | manual/computed | MoveIt | 可空 | 关节约束 |
| planner | string | - | N | manual | MoveIt | 可空 | 规划器名称 |
| max_time | number | s | N | manual | MoveIt | `>0` | 最大规划时间 |
| attempts | integer | 次 | N | manual | MoveIt | `>=1` | 规划尝试次数 |

### 7.2 PlanningVerificationResult

| 字段名 | 类型 | 单位 | 必填 | 来源 | 去向 | 校验规则 | 中文说明 |
|---|---|---:|---|---|---|---|---|
| verification_id | string | - | Y | computed | 快照/导出 | 唯一 | 验证结果 ID |
| planning_scene_ref | string | - | Y | computed | 快照/导出 | 非空 | 场景引用 |
| task_sequence_ref | string | - | N | manual/computed | 导出 | 可空 | 任务序列引用 |
| trajectory_results | array[object] | - | N | computed | UI/导出 | 可空 | 轨迹结果 |
| collision_results | array[object] | - | N | computed | UI/导出 | 可空 | 碰撞结果 |
| self_collision_results | array[object] | - | N | computed | UI/导出 | 可空 | 自碰撞结果 |
| singularity_risk_results | array[object] | - | N | computed | UI/导出 | 可空 | 奇异风险结果 |
| takt_evaluations | array[object] | - | N | computed | UI/导出 | 可空 | 节拍评估 |
| smoothness_results | array[object] | - | N | computed | UI/导出 | 可空 | 平滑性结果 |
| feasibility_summary | object | - | N | computed | UI/导出 | 可空 | 可规划性总结 |

---

## 8. JSON Schema 与验证器生成建议

### 8.1 Schema 文件建议目录

```text
core/schema/
├─ common/
│  ├─ base.schema.json
│  ├─ vector.schema.json
│  └─ enums.schema.json
├─ requirement/
│  └─ requirement-model.schema.json
├─ topology/
│  └─ robot-topology.schema.json
├─ kinematics/
│  └─ kinematic-model.schema.json
├─ dynamics/
│  └─ dynamic-model.schema.json
├─ selection/
│  ├─ motor-selection.schema.json
│  ├─ reducer-selection.schema.json
│  └─ drivetrain-selection.schema.json
└─ planning/
   ├─ planning-scene.schema.json
   └─ planning-verification.schema.json
```

### 8.2 Validator 生成要求

1. 每个 DTO 应有对应 Validator；
2. Validator 层只做字段合法性和结构合法性校验；
3. 跨字段业务规则放入 Rule Engine 或 Service 层；
4. 错误信息必须提供中文描述，便于 UI 直接展示；
5. 错误输出建议包含：`field`、`code`、`message_zh`、`severity`。

### 8.3 代码注释要求

Codex 生成 DTO / Schema / Validator 时必须满足：

- DTO 类头有中文说明；
- 每个字段有中文注释；
- 每个 validator 规则有中文说明；
- Schema 的 `description` 字段必须填写中文。

---

## 9. 面向 Codex 的生成顺序建议

1. 先生成 `common/base DTO + enum + vector schema`；
2. 再生成 `RequirementModel` 与 `RobotTopologyModel`；
3. 再生成 `KinematicModel` 与 `DynamicModel`；
4. 再生成 `Motor/Reducer/DriveTrain` 结果对象；
5. 最后生成 `PlanningScene` 与 `PlanningVerification`；
6. 每生成一个模块，就同步生成：
   - DTO
   - JSON Schema
   - Validator
   - Repository 序列化测试样例

---

## 10. 结论

这份 DTO / Schema 规格表的目标，不是替代详细设计文档，而是把详细设计文档中的核心对象进一步收敛为：

- 可生成代码的字段级定义；
- 可生成表单的字段级定义；
- 可生成 JSON Schema 的字段级定义；
- 可执行测试和校验的结构化依据。

后续若继续扩展二期模块，应沿用本文档的组织方式，继续补充：

- StructureModel
n- BearingSupportModel
- ToolInterfaceModel
- AccuracyAnalysisResult
- StiffnessStrengthModalResult
- SafetyReliabilityResult
- OptimizationResult

并保持“英文命名 + 中文注释 + 模块独立 + Schema 可验证”的统一规则。
