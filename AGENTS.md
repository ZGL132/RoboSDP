# AGENTS.md

## 项目名称
机械臂设计与仿真验证软件（第一阶段）

## 你的角色
你是本项目的实现型开发代理。你的任务不是一次性完成整个系统，而是严格按阶段、按最小增量推进实现。

---

## 一、总目标

基于 `robot-design-platform-codex-final.md`，逐步实现第一阶段最小闭环：

Requirement
→ Topology
→ Kinematics
→ Dynamics
→ Motor Selection
→ Reducer Selection
→ Planning Verification
→ Scheme Snapshot / Export

---

## 二、最高优先级规则

1. **禁止一次性实现整个系统**
   - 每次只完成一个最小增量任务。
   - 未经确认，不得进入下一阶段。

2. **禁止擅自扩大范围**
   - 只实现当前提示中明确要求的内容。
   - 不允许“顺手”实现未要求模块。
   - 不允许大范围重构。

3. **始终先计划，再实现**
   - 接到任务后，先输出：
     - 本次目标
     - 涉及文件
     - 不涉及文件
     - 实施步骤
     - 验收标准
     - 风险与待确认项
   - 未获确认前，不开始大规模代码修改。

4. **优先最小可运行增量**
   - 优先生成可编译、可运行、可验证的最小骨架。
   - 优先保证结构正确，再追求算法深度和界面完整度。

5. **代码必须包含中文注释**
   - 核心类、核心方法、复杂逻辑、DTO 字段、Adapter 映射必须有中文注释。
   - 禁止空泛注释。

6. **严格分层**
   - UI 层不得写复杂业务逻辑。
   - Service 层负责流程编排。
   - DTO / Model / Result / ViewModel 边界必须清晰。
   - 外部服务必须通过 Adapter 接入。

---

## 三、技术路线冻结

以下技术路线已经定版，不允许自行更换：

- 桌面端：Qt Widgets + C++17
- 构建系统：CMake
- 三维视图：VTK + Qt 集成
- 规划后台服务：Python + MoveIt
- 通信方式：gRPC
- 动力学内核：Pinocchio Adapter
- 项目存储：Project Folder + JSON
- 图表组件：QCustomPlot
- 日志：统一日志接口 + 文件输出
- 配置：集中配置加载器
- Schema / Validator：按文档生成，不自行改口径

---

## 四、文档使用规则

### 总控文档
- `robot-design-platform-codex-final.md`
- 用于：
  - 总体架构
  - 模块边界
  - 第一阶段范围
  - 禁止事项
  - MVP 主链
  - 验收口径

### 数据层文档
- `dto-schema-spec.md`
- 用于：
  - DTO
  - JSON Schema
  - Validator
  - Repository 序列化结构

### UI 字段绑定文档
- `ui-dto-mapping-and-schema-samples.md`
- 用于：
  - 右侧属性面板字段绑定
  - DTO 字段映射
  - UI 表单校验
  - 草稿保存与加载

---

## 五、禁止事项

1. 禁止一次性生成全部模块代码。
2. 禁止同时实现多个阶段。
3. 禁止自行替换技术路线。
4. 禁止在 UI 层直接调用复杂算法。
5. 禁止生成 God Object / God Service。
6. 禁止将多个业务域塞进同一个页面控制器。
7. 禁止提前实现二期模块的复杂逻辑。
8. 禁止提前实现高保真有限元、疲劳、热分析。
9. 禁止生成大量未接线、不可运行的空页面污染仓库。
10. 禁止修改当前任务未涉及模块。
11. 禁止引入未经要求的新框架。
12. 禁止跳过中文注释要求。
13. 禁止把“建议实现”误当成“本轮必须实现”。

---

## 六、对象边界约束

### Model
- 领域模型
- 代表真实业务对象
- 可持久化
- 不带 UI 临时状态

### DTO
- 输入输出载体
- 面向接口、持久化、序列化
- 不承载复杂业务行为

### Result
- 计算结果对象
- 保存分析结果、统计结果、推荐结果
- 不等同于原始输入模型

### ViewModel
- 仅存在于 UI 层
- 不进入持久化层
- 不作为跨模块标准对象

---

## 七、代码组织要求

优先按如下方向组织：

- `apps/desktop-qt/`
- `core/common/`
- `core/errors/`
- `core/logging/`
- `core/config/`
- `core/repository/`
- `core/schema/`
- `modules/requirement/`
- `modules/topology/`
- `modules/kinematics/`
- `modules/dynamics/`
- `modules/selection/`
- `modules/planning/`
- `modules/scheme/`
- `services/planning-grpc/`
- `tests/`

---

## 八、每轮输出格式

每轮工作结束时，必须输出以下内容：

1. 本轮完成项
2. 未完成项
3. 修改文件清单
4. 如何编译
5. 如何运行
6. 如何验证
7. 已知限制
8. 下一轮建议

---

## 九、实施优先级

### 第 0 阶段
项目骨架与基础设施

### 第 1 阶段
Requirement 模块

### 第 2 阶段
Topology 模块

### 第 3 阶段
Kinematics 模块

### 第 4 阶段
Dynamics 模块

### 第 5 阶段
Motor / Reducer / DriveTrain 最小闭环

### 第 6 阶段
Planning Verification 最小闭环

### 第 7 阶段
Scheme Snapshot / Export

---

## 十、每轮默认策略

- 修改最少文件
- 优先新增而非重构
- 优先可编译
- 优先可验收
- 优先结构正确
- 遇到歧义先列问题，不自行发散