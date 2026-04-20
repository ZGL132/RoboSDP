# 机械臂设计与仿真验证软件：右侧属性面板字段 ↔ DTO 字段映射表 + JSON Schema 样例说明

**文件名**：`ui-dto-mapping-and-schema-samples.md`  
**文档类型**：界面字段绑定与 Schema 落地说明  
**适用对象**：Qt 前端开发、后端/服务层开发、Codex 代码生成  
**用途**：用于把界面右侧属性面板的字段组织方式，与 DTO / Schema / Validator 直接对应起来，便于生成表单、数据绑定、校验逻辑与项目文件读写逻辑。

---

## 1. 文档目标

本文档解决三个问题：

1. 右侧属性面板上的每个表单字段，最终绑定到哪个 DTO 字段；
2. 哪些字段是直接录入字段，哪些字段是派生结果或只读字段；
3. JSON Schema 应如何组织，便于前后端、持久化和验证逻辑保持一致。

本文档是 `design-document_3_updated_v6.md` 和 `dto-schema-spec.md` 的配套实现文档。

---

## 2. 统一映射规则

### 2.1 字段命名规则

- UI 标签：中文，面向用户；
- DTO 字段名：英文，驼峰或下划线风格需全局统一；
- JSON Schema 属性名：与 DTO 字段名保持一致；
- 项目存储 JSON：与 Schema 属性名保持一致；
- 中文注释：写在代码注释和字段说明中，不进入持久化键名。

### 2.2 字段类型规则

建议统一基础类型：

- `string`
- `integer`
- `number`
- `boolean`
- `array`
- `object`
- `enum`（Schema 中用 `type + enum` 表示）

### 2.3 字段状态规则

每个字段至少应归类为以下之一：

- **可编辑输入字段**：用户可直接修改；
- **只读展示字段**：由系统计算后展示；
- **派生字段**：依赖多个输入自动生成；
- **引用字段**：指向其他 DTO / 资源库对象；
- **内部字段**：不在 UI 暴露，但保存在 DTO 或项目文件中。

### 2.4 校验触发规则

- 字段级即时校验：数值范围、非空、格式；
- 面板级校验：必填项组合、逻辑一致性；
- 模块级校验：上游依赖是否满足；
- 计算前校验：阻止非法计算；
- 保存前校验：允许草稿保存，但需标记状态。

---

## 3. 任务需求定义模块：UI ↔ DTO 映射

### 3.1 表单分组建议

1. 项目信息
2. 负载需求
3. 工作空间需求
4. 运动性能需求
5. 精度需求
6. 可靠性需求
7. 派生工况摘要（只读）

### 3.2 字段映射表

| UI分组 | UI字段名称 | DTO路径 | 类型 | 单位 | 必填 | 编辑性 | 校验规则 | 说明 |
|---|---|---|---|---|---|---|---|---|
| 项目信息 | 项目名称 | `projectMeta.projectName` | string | - | 是 | 可编辑 | 非空，长度<=100 | 项目标识 |
| 项目信息 | 场景类型 | `projectMeta.scenarioType` | string | - | 是 | 可编辑 | 枚举检查 | 如搬运/焊接/装配 |
| 项目信息 | 描述 | `projectMeta.description` | string | - | 否 | 可编辑 | 长度<=1000 | 任务描述 |
| 负载需求 | 额定负载 | `loadRequirements.ratedPayload` | number | kg | 是 | 可编辑 | >=0 | 正常工况负载 |
| 负载需求 | 最大负载 | `loadRequirements.maxPayload` | number | kg | 是 | 可编辑 | >=额定负载 | 极限工况负载 |
| 负载需求 | 工具质量 | `loadRequirements.toolMass` | number | kg | 是 | 可编辑 | >=0 | 末端工具质量 |
| 负载需求 | 工装质量 | `loadRequirements.fixtureMass` | number | kg | 否 | 可编辑 | >=0 | 夹具/工装质量 |
| 负载需求 | 负载质心X | `loadRequirements.payloadCog[0]` | number | m | 是 | 可编辑 | 数值 | 质心坐标 |
| 负载需求 | 负载质心Y | `loadRequirements.payloadCog[1]` | number | m | 是 | 可编辑 | 数值 | 质心坐标 |
| 负载需求 | 负载质心Z | `loadRequirements.payloadCog[2]` | number | m | 是 | 可编辑 | 数值 | 质心坐标 |
| 负载需求 | 偏载 | `loadRequirements.offCenterLoad` | boolean | - | 是 | 可编辑 | 布尔 | 是否偏载 |
| 负载需求 | 电缆拖曳负载 | `loadRequirements.cableDragLoad` | number | N | 否 | 可编辑 | >=0 | 外附加载荷 |
| 工作空间 | 最大半径 | `workspaceRequirements.maxRadius` | number | m | 是 | 可编辑 | >0 | 工作空间边界 |
| 工作空间 | 最小半径 | `workspaceRequirements.minRadius` | number | m | 否 | 可编辑 | >=0 且 <=最大半径 | 近端边界 |
| 工作空间 | 最大高度 | `workspaceRequirements.maxHeight` | number | m | 是 | 可编辑 | 数值 | 工作高度 |
| 工作空间 | 最低高度 | `workspaceRequirements.minHeight` | number | m | 否 | 可编辑 | <=最大高度 | 最低工作高度 |
| 运动性能 | 最大线速度 | `motionRequirements.maxLinearSpeed` | number | m/s | 否 | 可编辑 | >0 | 轨迹速度约束 |
| 运动性能 | 最大角速度 | `motionRequirements.maxAngularSpeed` | number | deg/s | 否 | 可编辑 | >0 | 旋转速度约束 |
| 运动性能 | 最大加速度 | `motionRequirements.maxAcceleration` | number | m/s² | 否 | 可编辑 | >0 | 线加速度约束 |
| 运动性能 | 最大角加速度 | `motionRequirements.maxAngularAcceleration` | number | deg/s² | 否 | 可编辑 | >0 | 角加速度约束 |
| 运动性能 | jerk限制 | `motionRequirements.jerkLimit` | number | m/s³ 或 deg/s³ | 否 | 可编辑 | >0 | 平滑性约束 |
| 运动性能 | 节拍 | `motionRequirements.taktTime` | number | s | 是 | 可编辑 | >0 | 总节拍 |
| 精度需求 | 绝对定位精度 | `accuracyRequirements.absoluteAccuracy` | number | mm | 否 | 可编辑 | >=0 | 全局精度 |
| 精度需求 | 重复定位精度 | `accuracyRequirements.repeatability` | number | mm | 否 | 可编辑 | >=0 | 关键精度指标 |
| 精度需求 | 跟踪精度 | `accuracyRequirements.trackingAccuracy` | number | mm | 否 | 可编辑 | >=0 | 路径精度 |
| 精度需求 | 姿态精度 | `accuracyRequirements.orientationAccuracy` | number | deg | 否 | 可编辑 | >=0 | 姿态误差 |
| 精度需求 | TCP位置容限 | `accuracyRequirements.tcpPositionTol` | number | mm | 否 | 可编辑 | >=0 | TCP约束 |
| 精度需求 | TCP姿态容限 | `accuracyRequirements.tcpOrientationTol` | number | deg | 否 | 可编辑 | >=0 | TCP姿态约束 |
| 可靠性需求 | 设计寿命 | `reliabilityRequirements.designLife` | number | h/年 | 否 | 可编辑 | >0 | 统一规范单位 |
| 可靠性需求 | 循环次数 | `reliabilityRequirements.cycleCount` | integer | 次 | 否 | 可编辑 | >=0 | 生命周期 |
| 可靠性需求 | 占空比 | `reliabilityRequirements.dutyCycle` | number | 0~1 | 否 | 可编辑 | 0~1 | 连续工况使用 |
| 可靠性需求 | 日运行时长 | `reliabilityRequirements.operatingHoursPerDay` | number | h | 否 | 可编辑 | 0~24 | 使用时长 |
| 可靠性需求 | MTBF目标 | `reliabilityRequirements.mtbfTarget` | number | h | 否 | 可编辑 | >0 | 可靠性目标 |
| 派生工况摘要 | 额定工况 | `derivedConditions.ratedCase` | object | - | 否 | 只读 | 系统生成 | 计算后展示 |
| 派生工况摘要 | 峰值工况 | `derivedConditions.peakCase` | object | - | 否 | 只读 | 系统生成 | 计算后展示 |

---

## 4. 总体构型设计模块：UI ↔ DTO 映射

### 4.1 表单分组建议

1. 机器人定义
2. 基座安装
3. 关节角色定义
4. 肩肘腕布局
5. 轴线关系
6. 中空与走线预留
7. 候选构型与推荐结果（摘要）

### 4.2 字段映射表（核心字段）

| UI分组 | UI字段名称 | DTO路径 | 类型 | 单位 | 必填 | 编辑性 | 校验规则 | 说明 |
|---|---|---|---|---|---|---|---|---|
| 机器人定义 | 机器人类型 | `robotDefinition.robotType` | string | - | 是 | 可编辑 | 当前限定6R_serial | 首期限定 |
| 机器人定义 | 自由度 | `robotDefinition.dof` | integer | - | 是 | 可编辑 | =6 | 首期固定 |
| 基座安装 | 安装方式 | `baseMount.baseMountType` | string | - | 是 | 可编辑 | 枚举 | floor/wall/ceiling |
| 基座安装 | 基座高度 | `baseMount.baseHeightM` | number | m | 否 | 可编辑 | >=0 | 可选 |
| 基座安装 | J1范围最小 | `baseMount.j1RotationRangeDeg[0]` | number | deg | 否 | 可编辑 | min<max | 第一轴范围 |
| 基座安装 | J1范围最大 | `baseMount.j1RotationRangeDeg[1]` | number | deg | 否 | 可编辑 | max>min | 第一轴范围 |
| 肩肘腕布局 | 肩部类型 | `layout.shoulderType` | string | - | 是 | 可编辑 | 枚举/模板约束 | 构型布局 |
| 肩肘腕布局 | 肘部类型 | `layout.elbowType` | string | - | 是 | 可编辑 | 枚举/模板约束 | 构型布局 |
| 肩肘腕布局 | 腕部类型 | `layout.wristType` | string | - | 是 | 可编辑 | 枚举 | 球腕/偏置腕等 |
| 肩肘腕布局 | 腕部交点 | `layout.wristIntersection` | boolean | - | 否 | 可编辑 | 布尔 | 球腕特征 |
| 肩肘腕布局 | 腕部偏置 | `layout.wristOffset` | boolean | - | 否 | 可编辑 | 布尔 | 结构特征 |
| 走线预留 | 需要内部走线 | `routingReservation.internalRoutingRequired` | boolean | - | 否 | 可编辑 | 布尔 | 线缆/气管 |
| 走线预留 | 需要中空腕 | `routingReservation.hollowWristRequired` | boolean | - | 否 | 可编辑 | 布尔 | 中空需求 |
| 走线预留 | 通道直径 | `routingReservation.reservedChannelDiameterMm` | number | mm | 否 | 可编辑 | >=0 | 预留通道 |
| 外部轴 | 预留第七轴 | `externalAxis.seventhAxisReserved` | boolean | - | 否 | 可编辑 | 布尔 | 扩展预留 |

---

## 5. 运动学参数设计模块：UI ↔ DTO 映射

### 5.1 表单分组建议

1. 建模入口
2. 坐标系定义
3. DH/MDH 参数表
4. 关节限位与运动边界
5. IK 求解配置
6. 分析任务设置
7. 结果摘要（只读）

### 5.2 字段映射表（核心字段）

| UI分组 | UI字段名称 | DTO路径 | 类型 | 单位 | 必填 | 编辑性 | 校验规则 | 说明 |
|---|---|---|---|---|---|---|---|---|
| 建模入口 | 建模模式 | `modelingMode` | string | - | 是 | 可编辑 | DH/MDH/URDF | 二选一或三选一 |
| 坐标系 | 基坐标系 | `baseFrame` | object | - | 是 | 可编辑 | 完整姿态对象 | 结构化定义 |
| 坐标系 | 法兰坐标系 | `flangeFrame` | object | - | 否 | 可编辑 | 完整姿态对象 | 可由模板生成 |
| 坐标系 | 工具坐标系 | `toolFrame` | object | - | 否 | 可编辑 | 完整姿态对象 | 工具定义 |
| 坐标系 | 工件坐标系 | `workpieceFrame` | object | - | 否 | 可编辑 | 完整姿态对象 | 工艺定义 |
| 坐标系 | TCP 坐标系 | `tcpFrame` | object | - | 是 | 可编辑 | 完整姿态对象 | 关键字段 |
| DH参数 | link_id | `links[i].linkId` | string | - | 是 | 可编辑 | 非空唯一 | 连杆标识 |
| DH参数 | a | `links[i].a` | number | m | 是 | 可编辑 | 数值 | DH参数 |
| DH参数 | alpha | `links[i].alpha` | number | deg/rad | 是 | 可编辑 | 数值 | DH参数 |
| DH参数 | d | `links[i].d` | number | m | 是 | 可编辑 | 数值 | DH参数 |
| DH参数 | theta_offset | `links[i].thetaOffset` | number | deg/rad | 否 | 可编辑 | 数值 | 零位偏置 |
| 关节限位 | 软限位最小 | `jointLimits[i].softLimit[0]` | number | deg/mm | 否 | 可编辑 | min<max | 关节边界 |
| 关节限位 | 软限位最大 | `jointLimits[i].softLimit[1]` | number | deg/mm | 否 | 可编辑 | max>min | 关节边界 |
| 关节限位 | 硬限位最小 | `jointLimits[i].hardLimit[0]` | number | deg/mm | 否 | 可编辑 | min<max | 关节边界 |
| 关节限位 | 硬限位最大 | `jointLimits[i].hardLimit[1]` | number | deg/mm | 否 | 可编辑 | max>min | 关节边界 |
| 关节限位 | 最大速度 | `jointLimits[i].maxVelocity` | number | deg/s or mm/s | 否 | 可编辑 | >0 | 动态约束 |
| 关节限位 | 最大加速度 | `jointLimits[i].maxAcceleration` | number | deg/s² or mm/s² | 否 | 可编辑 | >0 | 动态约束 |
| 求解配置 | IK求解器类型 | `ikSolverConfig.solverType` | string | - | 否 | 可编辑 | 枚举 | 数值法/解析法 |
| 求解配置 | 分支策略 | `ikSolverConfig.branchPolicy` | string | - | 否 | 可编辑 | 枚举 | 推荐分支策略 |
| 结果摘要 | 可达性结果 | `reachabilityResult` | object | - | 否 | 只读 | 系统生成 | 结果摘要 |
| 结果摘要 | 奇异图 | `singularityMap` | object | - | 否 | 只读 | 系统生成 | 结果摘要 |

---

## 6. 动力学与负载分析模块：UI ↔ DTO 映射

### 6.1 表单分组建议

1. 重力与安装姿态
2. 连杆质量/质心/惯量
3. 关节传动参数
4. 末端负载参数
5. 轨迹工况配置
6. 统计输出摘要

### 6.2 字段映射表（核心字段）

| UI分组 | UI字段名称 | DTO路径 | 类型 | 单位 | 必填 | 编辑性 | 校验规则 | 说明 |
|---|---|---|---|---|---|---|---|---|
| 安装姿态 | 重力X | `gravity[0]` | number | m/s² | 是 | 可编辑 | 数值 | 重力向量 |
| 安装姿态 | 重力Y | `gravity[1]` | number | m/s² | 是 | 可编辑 | 数值 | 重力向量 |
| 安装姿态 | 重力Z | `gravity[2]` | number | m/s² | 是 | 可编辑 | 数值 | 重力向量 |
| 安装姿态 | 安装姿态 | `installationPose` | object | - | 是 | 可编辑 | 完整姿态对象 | 地装/侧装/顶装 |
| 连杆参数 | 质量 | `links[i].mass` | number | kg | 是 | 可编辑 | >=0 | 动力学基础字段 |
| 连杆参数 | 质心 | `links[i].cog` | array[number] | m | 是 | 可编辑 | 长度=3 | 质心 |
| 连杆参数 | 惯量张量 | `links[i].inertiaTensor` | array[number] | kg·m² | 是 | 可编辑 | 长度=6 | 惯量 |
| 连杆参数 | 来源 | `links[i].source` | string | - | 是 | 可编辑/只读 | 枚举 | urdf/manual/estimated |
| 关节参数 | 传动比 | `joints[i].transmissionRatio` | number | - | 否 | 可编辑 | >0 | 驱动链参数 |
| 关节参数 | 效率 | `joints[i].efficiency` | number | 0~1 | 否 | 可编辑 | 0~1 | 传动效率 |
| 关节参数 | 粘性摩擦 | `joints[i].friction.viscous` | number | - | 否 | 可编辑 | >=0 | 摩擦参数 |
| 关节参数 | 库仑摩擦 | `joints[i].friction.coulomb` | number | - | 否 | 可编辑 | >=0 | 摩擦参数 |
| 关节参数 | 静摩擦 | `joints[i].friction.static` | number | - | 否 | 可编辑 | >=0 | 摩擦参数 |
| 末端负载 | 末端质量 | `endEffector.mass` | number | kg | 否 | 可编辑 | >=0 | 工具/末端 |
| 末端负载 | 末端质心 | `endEffector.cog` | array[number] | m | 否 | 可编辑 | 长度=3 | 工具/末端 |
| 末端负载 | 末端惯量 | `endEffector.inertiaTensor` | array[number] | kg·m² | 否 | 可编辑 | 长度=6 | 工具/末端 |
| 轨迹工况 | 轨迹列表 | `trajectories` | array | - | 否 | 可编辑 | 结构合法 | 可导入或生成 |
| 结果摘要 | 负载包络 | `loadEnvelope` | object | - | 否 | 只读 | 系统生成 | 关键结果 |
| 结果摘要 | 峰值统计 | `results.peakStats` | array | - | 否 | 只读 | 系统生成 | 关键结果 |
| 结果摘要 | RMS统计 | `results.rmsStats` | array | - | 否 | 只读 | 系统生成 | 关键结果 |

---

## 7. 电机选型模块：UI ↔ DTO 映射

### 7.1 表单分组建议

1. 电机需求摘要（只读）
2. 电机筛选约束
3. 抱闸配置约束
4. 候选列表
5. 推荐结果摘要

### 7.2 字段映射表（核心字段）

| UI分组 | UI字段名称 | DTO路径 | 类型 | 单位 | 必填 | 编辑性 | 校验规则 | 说明 |
|---|---|---|---|---|---|---|---|---|
| 需求摘要 | 关节ID | `jointId` | string | - | 是 | 只读 | 非空 | 当前关节 |
| 需求摘要 | 需求引用 | `requirementRef` | string | - | 是 | 只读 | 非空 | 上游引用 |
| 候选筛选 | 品牌过滤 | `candidateMotors`（过滤条件单独DTO更佳） | array/string | - | 否 | 可编辑 | 可空 | 前端过滤条件 |
| 候选筛选 | 额定转矩下限 | `checkResults`上游需求，不建议写回结果DTO | number | Nm | 否 | 可编辑 | >=0 | 建议单独 Request DTO |
| 推荐结果 | 推荐电机 | `recommendedMotor.model` | string | - | 否 | 只读 | 系统生成 | 选型结果 |
| 推荐结果 | 额定转矩 | `recommendedMotor.ratedTorqueNm` | number | Nm | 否 | 只读 | 系统生成 | 选型结果 |
| 推荐结果 | 峰值转矩 | `recommendedMotor.peakTorqueNm` | number | Nm | 否 | 只读 | 系统生成 | 选型结果 |
| 推荐结果 | 额定转速 | `recommendedMotor.ratedSpeedRpm` | number | rpm | 否 | 只读 | 系统生成 | 选型结果 |
| 推荐结果 | 最高转速 | `recommendedMotor.maxSpeedRpm` | number | rpm | 否 | 只读 | 系统生成 | 选型结果 |
| 推荐结果 | 转子惯量 | `recommendedMotor.rotorInertia` | number | kg·m² | 否 | 只读 | 系统生成 | 选型结果 |
| 推荐结果 | 抱闸选项 | `recommendedMotor.brakeOption` | object | - | 否 | 只读 | 系统生成 | 抱闸参数 |
| 校核结果 | 峰值转矩校核 | `checkResults.peakTorqueCheck` | string | - | 否 | 只读 | pass/warn/fail | 结果字段 |
| 校核结果 | RMS转矩校核 | `checkResults.rmsTorqueCheck` | string | - | 否 | 只读 | pass/warn/fail | 结果字段 |
| 校核结果 | 速度校核 | `checkResults.speedCheck` | string | - | 否 | 只读 | pass/warn/fail | 结果字段 |
| 校核结果 | 功率校核 | `checkResults.powerCheck` | string | - | 否 | 只读 | pass/warn/fail | 结果字段 |
| 校核结果 | 惯量匹配校核 | `checkResults.inertiaMatchCheck` | string | - | 否 | 只读 | pass/warn/fail | 结果字段 |
| 校核结果 | 热负荷校核 | `checkResults.thermalCheck` | string | - | 否 | 只读 | pass/warn/fail | 结果字段 |
| 校核结果 | 抱闸校核 | `checkResults.brakeCheck` | string | - | 否 | 只读 | pass/warn/fail | 结果字段 |

> 注：电机选型界面建议额外定义 `MotorSelectionRequestDto`，承载前端筛选约束，而 `MotorSelectionResultDto` 只承载结果。

---

## 8. 减速器选型模块：UI ↔ DTO 映射

### 8.1 表单分组建议

1. 输出侧需求摘要（只读）
2. 传动比区间设置 / 推荐
3. 减速器筛选约束
4. 候选列表
5. 推荐结果摘要

### 8.2 字段映射表（核心字段）

| UI分组 | UI字段名称 | DTO路径 | 类型 | 单位 | 必填 | 编辑性 | 校验规则 | 说明 |
|---|---|---|---|---|---|---|---|---|
| 需求摘要 | 关节ID | `jointId` | string | - | 是 | 只读 | 非空 | 当前关节 |
| 需求摘要 | 需求引用 | `requirementRef` | string | - | 是 | 只读 | 非空 | 上游引用 |
| 推荐结果 | 推荐减速器 | `recommendedReducer.model` | string | - | 否 | 只读 | 系统生成 | 选型结果 |
| 推荐结果 | 类型 | `recommendedReducer.reducerType` | string | - | 否 | 只读 | 系统生成 | RV/谐波等 |
| 推荐结果 | 传动比 | `recommendedReducer.ratio` | number | - | 否 | 只读 | >0 | 核心结果 |
| 推荐结果 | 额定输出转矩 | `recommendedReducer.ratedOutputTorqueNm` | number | Nm | 否 | 只读 | 系统生成 | 结果字段 |
| 推荐结果 | 峰值输出转矩 | `recommendedReducer.peakOutputTorqueNm` | number | Nm | 否 | 只读 | 系统生成 | 结果字段 |
| 推荐结果 | 回差 | `recommendedReducer.backlashArcmin` | number | arcmin | 否 | 只读 | 系统生成 | 精度相关 |
| 推荐结果 | 扭转刚度 | `recommendedReducer.torsionalStiffness` | number | Nm/arcmin 或规定单位 | 否 | 只读 | 系统生成 | 刚度相关 |
| 推荐结果 | 额定寿命 | `recommendedReducer.nominalLifeH` | number | h | 否 | 只读 | 系统生成 | 寿命相关 |
| 校核结果 | 额定转矩校核 | `checkResults.ratedTorqueCheck` | string | - | 否 | 只读 | pass/warn/fail | 结果字段 |
| 校核结果 | 峰值转矩校核 | `checkResults.peakTorqueCheck` | string | - | 否 | 只读 | pass/warn/fail | 结果字段 |
| 校核结果 | 输入转速校核 | `checkResults.inputSpeedCheck` | string | - | 否 | 只读 | pass/warn/fail | 结果字段 |
| 校核结果 | 回差校核 | `checkResults.backlashCheck` | string | - | 否 | 只读 | pass/warn/fail | 结果字段 |
| 校核结果 | 刚度校核 | `checkResults.stiffnessCheck` | string | - | 否 | 只读 | pass/warn/fail | 结果字段 |
| 校核结果 | 寿命校核 | `checkResults.lifeCheck` | string | - | 否 | 只读 | pass/warn/fail | 结果字段 |
| 校核结果 | 效率校核 | `checkResults.efficiencyCheck` | string | - | 否 | 只读 | pass/warn/fail | 结果字段 |
| 校核结果 | 冲击校核 | `checkResults.shockCheck` | string | - | 否 | 只读 | pass/warn/fail | 结果字段 |

---

## 9. 联合驱动链匹配模块：UI ↔ DTO 映射

| UI字段名称 | DTO路径 | 类型 | 单位 | 必填 | 编辑性 | 说明 |
|---|---|---|---|---|---|---|
| 关节ID | `jointId` | string | - | 是 | 只读 | 当前关节 |
| 电机结果引用 | `motorSelectionRef` | string | - | 是 | 只读 | 上游电机结果 |
| 减速器结果引用 | `reducerSelectionRef` | string | - | 是 | 只读 | 上游减速器结果 |
| 推荐电机型号 | `recommendedCombination.motorModel` | string | - | 否 | 只读 | 联合结果 |
| 推荐减速器型号 | `recommendedCombination.reducerModel` | string | - | 否 | 只读 | 联合结果 |
| 推荐传动比 | `recommendedCombination.ratio` | number | - | 否 | 只读 | 联合结果 |
| 启用抱闸 | `recommendedCombination.brakeEnabled` | boolean | - | 否 | 只读 | 联合结果 |
| 联合质量 | `combinedMassKg` | number | kg | 否 | 只读 | 联合评估 |
| 联合评分 | `combinedScore` | number | - | 否 | 只读 | 综合评分 |
| 推荐理由 | `recommendationReason` | array[string] | - | 否 | 只读 | 说明性字段 |

---

## 10. 规划与仿真验证模块：UI ↔ DTO 映射

### 10.1 表单分组建议

1. 规划场景
2. 目标与约束
3. 规划参数
4. 结果摘要

### 10.2 字段映射表（核心字段）

| UI分组 | UI字段名称 | DTO路径 | 类型 | 单位 | 必填 | 编辑性 | 校验规则 | 说明 |
|---|---|---|---|---|---|---|---|---|
| 场景 | 机器人碰撞模型引用 | `robotCollisionModelRef` | string | - | 是 | 可编辑/只读 | 非空 | 模型引用 |
| 场景 | 环境对象 | `environmentObjects` | array | - | 否 | 可编辑 | 结构合法 | 障碍物集合 |
| 场景 | 禁入区 | `forbiddenRegions` | array | - | 否 | 可编辑 | 结构合法 | 安全区域 |
| 场景 | 附着物 | `attachedObjects` | array | - | 否 | 可编辑 | 结构合法 | 工具/工件 |
| 约束 | 路径约束 | `motionConstraints.pathConstraints` | array | - | 否 | 可编辑 | 结构合法 | 规划约束 |
| 约束 | 姿态约束 | `motionConstraints.orientationConstraints` | array | - | 否 | 可编辑 | 结构合法 | 规划约束 |
| 约束 | 关节约束 | `motionConstraints.jointConstraints` | array | - | 否 | 可编辑 | 结构合法 | 规划约束 |
| 规划参数 | 规划器 | `planningConfig.planner` | string | - | 是 | 可编辑 | 非空 | RRTConnect等 |
| 规划参数 | 最大规划时间 | `planningConfig.maxTime` | number | s | 否 | 可编辑 | >0 | 求解边界 |
| 规划参数 | 尝试次数 | `planningConfig.attempts` | integer | 次 | 否 | 可编辑 | >=1 | 求解参数 |
| 结果摘要 | 轨迹结果 | `trajectoryResults` | array | - | 否 | 只读 | 系统生成 | 结果字段 |
| 结果摘要 | 碰撞结果 | `collisionResults` | array | - | 否 | 只读 | 系统生成 | 结果字段 |
| 结果摘要 | 自碰撞结果 | `selfCollisionResults` | array | - | 否 | 只读 | 系统生成 | 结果字段 |
| 结果摘要 | 奇异风险结果 | `singularityRiskResults` | array | - | 否 | 只读 | 系统生成 | 结果字段 |
| 结果摘要 | 节拍评估 | `taktEvaluations` | array | - | 否 | 只读 | 系统生成 | 结果字段 |
| 结果摘要 | 平滑性结果 | `smoothnessResults` | array | - | 否 | 只读 | 系统生成 | 结果字段 |
| 结果摘要 | 总结 | `feasibilitySummary` | object | - | 否 | 只读 | 系统生成 | 统一结论 |

---

## 11. JSON Schema 组织建议

### 11.1 目录建议

```text
schemas/
├─ common/
│  ├─ base-entity.schema.json
│  ├─ vector3.schema.json
│  ├─ inertia-tensor.schema.json
│  └─ pose.schema.json
├─ requirement/
│  └─ requirement-model.schema.json
├─ topology/
│  └─ robot-topology-model.schema.json
├─ kinematics/
│  └─ kinematic-model.schema.json
├─ dynamics/
│  └─ dynamic-model.schema.json
├─ selection/
│  ├─ motor-selection-result.schema.json
│  ├─ reducer-selection-result.schema.json
│  └─ drivetrain-selection-result.schema.json
├─ planning/
│  ├─ planning-scene-model.schema.json
│  └─ planning-verification-result.schema.json
└─ snapshot/
   └─ scheme-snapshot.schema.json
```

### 11.2 通用 Schema 复用建议

#### `vector3.schema.json`
```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "array",
  "items": { "type": "number" },
  "minItems": 3,
  "maxItems": 3,
  "$comment": "三维向量，按 [x, y, z] 顺序表示。"
}
```

#### `inertia-tensor.schema.json`
```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "array",
  "items": { "type": "number" },
  "minItems": 6,
  "maxItems": 6,
  "$comment": "惯量张量，按 [ixx, iyy, izz, ixy, ixz, iyz] 顺序表示。"
}
```

### 11.3 RequirementModel Schema 样例（片段）

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "schemas/requirement/requirement-model.schema.json",
  "title": "RequirementModel",
  "type": "object",
  "required": ["projectMeta", "loadRequirements", "workspaceRequirements", "motionRequirements"],
  "properties": {
    "projectMeta": {
      "type": "object",
      "required": ["projectName", "scenarioType"],
      "properties": {
        "projectName": { "type": "string", "minLength": 1, "maxLength": 100 },
        "scenarioType": { "type": "string", "minLength": 1 },
        "description": { "type": "string" }
      }
    },
    "loadRequirements": {
      "type": "object",
      "required": ["ratedPayload", "maxPayload", "toolMass", "payloadCog"],
      "properties": {
        "ratedPayload": { "type": "number", "minimum": 0 },
        "maxPayload": { "type": "number", "minimum": 0 },
        "toolMass": { "type": "number", "minimum": 0 },
        "fixtureMass": { "type": "number", "minimum": 0 },
        "payloadCog": { "$ref": "../common/vector3.schema.json" },
        "offCenterLoad": { "type": "boolean" }
      }
    }
  },
  "allOf": [
    {
      "$comment": "最大负载不得小于额定负载",
      "if": {
        "properties": {
          "loadRequirements": {
            "properties": {
              "ratedPayload": { "type": "number" },
              "maxPayload": { "type": "number" }
            }
          }
        }
      },
      "then": true
    }
  ]
}
```

### 11.4 RobotTopologyModel Schema 样例（片段）

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "schemas/topology/robot-topology-model.schema.json",
  "title": "RobotTopologyModel",
  "type": "object",
  "required": ["robotDefinition", "joints", "layout"],
  "properties": {
    "robotDefinition": {
      "type": "object",
      "required": ["robotType", "dof"],
      "properties": {
        "robotType": { "type": "string", "enum": ["6R_serial"] },
        "dof": { "type": "integer", "const": 6 }
      }
    },
    "joints": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["jointId", "jointType", "axisDirection"],
        "properties": {
          "jointId": { "type": "string" },
          "jointType": { "type": "string", "enum": ["revolute", "prismatic"] },
          "axisDirection": { "$ref": "../common/vector3.schema.json" },
          "range": {
            "type": "array",
            "items": { "type": "number" },
            "minItems": 2,
            "maxItems": 2
          },
          "hollow": { "type": "boolean" }
        }
      }
    }
  }
}
```

---

## 12. Validator 生成要求

### 12.1 Validator 分类

建议为每个模块至少生成三类 Validator：

1. **字段级 Validator**：检查基础类型、范围、非空；
2. **模块级 Validator**：检查逻辑一致性；
3. **流程级 Validator**：检查上游依赖是否满足计算条件。

### 12.2 示例

#### RequirementValidator
- 最大负载 ≥ 额定负载
- 最小半径 ≤ 最大半径
- 最低高度 ≤ 最大高度
- 占空比在 0~1 之间
- 节拍 > 0

#### TopologyValidator
- dof 必须为 6（首期）
- jointId 唯一
- 轴方向向量长度合法
- parentLinkId / childLinkId 不得为空

#### KinematicsValidator
- DH/MDH 参数表完整
- 关节限位成对出现
- 坐标系对象合法
- IK 配置有效

#### DynamicsValidator
- 质量、质心、惯量齐全
- 重力向量有效
- 效率在 0~1 之间
- 传动比 > 0

---

## 13. Codex 生成顺序建议（字段层）

建议 Codex 按以下顺序生成：

### 第一步：生成 `common/` 基础 Schema 与 DTO
- BaseEntity
- Vector3
- Pose
- InertiaTensor

### 第二步：生成 `RequirementModel` 与 Validator
- Requirement DTO
- Requirement Schema
- Requirement Validator

### 第三步：生成 `RobotTopologyModel` 与 Validator

### 第四步：生成 `KinematicModel` 与 Validator

### 第五步：生成 `DynamicModel` 与 Validator

### 第六步：生成 Selection DTO / Schema / Validator
- Motor
- Reducer
- DriveTrain

### 第七步：生成 Planning DTO / Schema / Validator

### 第八步：生成 UI 表单绑定层
- 字段 → DTO 映射
- 校验错误提示
- 草稿保存与状态刷新

---

## 14. 建议的下一份配套文档

在本文件基础上，下一步最适合继续补的是：

1. **Ribbon / Toolbar 按钮级功能清单**  
   便于 Codex 生成主界面菜单、工具栏和命令绑定。  

2. **右侧属性面板字段组 JSON 配置样例**  
   便于做配置驱动表单，而不是手写全部 Qt 表单代码。  

3. **错误码与底部输出信息面板消息规范**  
   便于统一日志、校验错误、计算失败和规划失败的展示口径。

