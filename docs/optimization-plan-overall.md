# 机械臂设计与仿真验证软件 —— 整体优化计划

**文档版本**：v0.2  
**创建日期**：2026-05-03  
**适用阶段**：第一阶段主链闭环已跑通后的整体优化  
**前置文档**：[`robot-design-platform-codex-final.md`](robot-design-platform-codex-final.md)、[`dto-schema-spec.md`](dto-schema-spec.md)

---

## 目录

1. [当前状态评估](#1-当前状态评估)
2. [优化目标与原则](#2-优化目标与原则)
3. [总体优化路线图](#3-总体优化路线图)
4. [架构优化](#4-架构优化)
5. [UI/UX 优化](#5-uiux-优化)
6. [代码质量优化](#6-代码质量优化)
7. [性能优化](#7-性能优化)
8. [测试优化](#8-测试优化)
9. [构建与工具链优化](#9-构建与工具链优化)
10. [文档优化](#10-文档优化)
11. [风险与依赖](#11-风险与依赖)
12. [各模块详细优化计划索引](#12-各模块详细优化计划索引)
13. [附录：优化优先级矩阵](#13-附录优化优先级矩阵)

---

## 1. 当前状态评估

### 1.1 已完成内容

| 模块 | 层 | 状态 |
|------|----|------|
| 项目基础设施 | Core (Config/Logging/Error/Repository/Schema) | ✅ 基本完成 |
| 项目骨架 | Core (ProjectManager/SaveCoordinator/DirtyGraph) | ✅ 基本完成 |
| 需求模块 | DTO/Service/Persistence/UI/Validator | ✅ 最小闭环 |
| 构型模块 | DTO/Service/Persistence/UI/Validator | ✅ 最小闭环 |
| 运动学模块 | DTO/Service/Persistence/UI/Adapter | ✅ 最小闭环 |
| 动力学模块 | DTO/Service/Persistence/UI/Adapter/Trajectory | ✅ 最小闭环 |
| 选型模块 | DTO/Service/Persistence/UI/Catalog | ✅ 最小闭环 |
| 规划模块 | DTO/Service/Persistence/UI/Adapter/gRPC | ✅ 最小闭环 |
| 方案模块 | DTO/Service/Persistence/UI | ✅ 最小闭环 |
| 主窗口 | Ribbon/VTK View/ProjectTree/LogDock | ✅ 基本完成 |
| 三维视图 | VTK Scene Builder/Actor Manager | ✅ 基本完成 |
| 测试 | Unit Tests / Integration Tests | ⚠️ 部分覆盖 |

### 1.2 当前主要问题

#### 1.2.1 架构层

1. **Widget 层过重**：每个 Widget 直接实例化 Service、Repository、Storage、Logger，无依赖注入容器，难以单元测试。
2. **存储模式重复**：7 个模块各自实现独立的 JsonStorage，存在大量重复的序列化/反序列化模板代码。
3. **模块间耦合偏紧**：Widget 跨模块引用其他模块的 Persistence 类（如 [`TopologyWidget`](modules/topology/ui/TopologyWidget.h:121) 直接引用 [`RequirementJsonStorage`](modules/requirement/persistence/RequirementJsonStorage.h) 和 [`KinematicsService`](modules/kinematics/service/KinematicsService.h)），违反最小知识原则。
4. **DTO 膨胀**：[`KinematicModelDto`](modules/kinematics/dto/KinematicModelDto.h:81) 包含大量与 DH 参数无直接关系的工程元字段（30+ 字段），Single Responsibility 偏弱。
5. **Dynamics source/ 目录遗留**：[`modules/dynamics/source/`](modules/dynamics/source/) 目录存在大量与正式目录重复的文件（`DynamicModelDto.h`、`DynamicsService.cpp` 等），疑似迁移/合并过程中的残留，应清理。
6. **日志接口使用不统一**：部分 Widget 直接持有 [`ConsoleLogger`](core/logging/ConsoleLogger.h) 具体类而非通过 [`ILogger`](core/logging/ILogger.h) 接口注入。

#### 1.2.2 UI/UX 层

1. **表单交互单一**：所有模块使用纯 `QGroupBox` + `QDoubleSpinBox` + `QPlainTextEdit` 文本摘要，缺少可视化交互（3D 点选、拖拽、实时反馈）。
2. **校验反馈不足**：字段校验仅后端返回文本列表，缺少内联红色高亮、Tooltip 提示和批量错误定位。
3. **右侧属性面板实际是页面切换器**：[`MainWindow`](apps/desktop-qt/MainWindow.h:164) 的 `m_propertyStack` 为 `QStackedWidget`，切换模块时实际切换的是完整的功能页面，非标准属性编辑器模式。
4. **KeyPose 编辑原始**：Requirement 的关键工位使用 `QListWidget` + 离散 SpinBox，无法在 3D 场景中点选/拖拽。
5. **Ribbon 上下文切换生硬**：切换模块时上下文标签卡出现/消失无动画过渡，状态衔接感弱。
6. **无多语言/i18n 支持**：界面文字硬编码，未使用 `tr()`。
7. **缺少统一的消息/通知系统**：操作结果通过 `QLabel` 文本更新显示，缺少 Toast/Notification 机制。

#### 1.2.3 代码质量层

1. **中文注释不一致**：部分文件（如 [`TopologyWidget.h`](modules/topology/ui/TopologyWidget.h)）注释完整规范，但部分文件（如 [`AppBootstrap.cpp`](apps/desktop-qt/AppBootstrap.cpp)）注释不够详细或存在空泛注释。
2. **结果类型各自为政**：每个 Service 有独立的 `*Result` 结构（`KinematicBuildResult`、`DynamicsSaveResult` 等），语义重叠，缺少统一的 `ServiceResult<T>` 泛型。
3. **错误码未充分利用**：[`ErrorCode`](core/errors/ErrorCode.h) 枚举已定义但部分模块仅返回 `Ok`/`GenericError`，未细化。
4. **Dynamics 模块 source/ 目录冗余**：`modules/dynamics/source/` 下存在 `DynamicModelDto.h`、`DynamicsService.cpp`、`DynamicsWidget.cpp` 等文件，与 `dto/`、`service/`、`ui/` 目录中的文件重复，属于构建/迁移残留。
5. **硬编码字符串散布**：界面文字、错误消息、日志字符串均硬编码在各 `.cpp` 文件中，不利于维护和国际化。

#### 1.2.4 性能层

1. **模块页面懒加载缺失**：所有 7 个 Widget 在 [`MainWindow`](apps/desktop-qt/MainWindow.cpp:39) 构造时全部实例化，内存开销持续存在。
2. **VTK 场景重建频繁**：每次参数变化都触发 Clear + 重新创建 Actor，未使用增量更新策略。
3. **JSON 同步 I/O**：保存/加载均为主线程同步操作，大项目可能阻塞 UI。
4. **无缓存策略**：模板加载、元件库加载每次重新解析 JSON，无内存缓存。

#### 1.2.5 测试层

1. **单元测试覆盖率不足**：部分模块（Topology、Selection、Scheme）只有 1 个 Smoke Test，缺少边界条件和错误路径覆盖。
2. **无 UI 测试**：Widget 交互逻辑无法自动化验证。
3. **集成测试仅覆盖主链 Happy Path**：未覆盖错误恢复、版本迁移、并发保存等场景。
4. **测试数据不可复用**：各测试自行构造数据，无统一 Test Fixture 工厂。

#### 1.2.6 构建与工具链

1. **VTK 依赖非可选**：主程序硬依赖 VTK，无 VTK 环境下无法编译。
2. **第三方库管理原始**：QCustomPlot 直接源码拷贝至 [`apps/desktop-qt/third_party/qcustomplot/`](apps/desktop-qt/third_party/qcustomplot/)。
3. **MinGW/MSVC 兼容层脆弱**：[`strip-msvc-bigobj.py`](tools/build/strip-msvc-bigobj.py) 包装器依赖 `.deps/pinocchio-env` 环境。

---

## 2. 优化目标与原则

### 2.1 优化目标

1. **提升代码可维护性**：减少重复、降低耦合、统一模式
2. **改善用户体验**：增强交互反馈、完善校验提示、优化视觉一致性
3. **保障质量**：提升测试覆盖率、完善错误处理
4. **优化性能**：减少不必要的重建、实现懒加载
5. **降低构建门槛**：解耦 VTK 依赖、简化第三方库管理

### 2.2 优化原则

1. **破坏性最小**：不重构已有功能的外部接口，优先增量优化
2. **向后兼容**：不破坏已有项目文件的加载
3. **可验收**：每步优化必须有明确的验证方法
4. **渐近实施**：按模块分批进行，每批独立可编译可运行
5. **先清理再优化**：先清除冗余文件/代码，再进行优化改造

---

## 3. 总体优化路线图

```text
阶段 A（基础设施优化）
├── A0. 代码清理（清除冗余文件、统一代码风格）
├── A1. 统一存储抽象层（重构重复的 JsonStorage）
├── A2. 依赖注入基础设施
├── A3. 日志系统增强（级别/过滤/格式化/接口统一）
├── A4. 错误码细化与统一错误处理
└── A5. 模块懒加载机制

阶段 B（UI/UX 优化）
├── B0. 统一消息/通知系统
├── B1. 统一表单校验框架（内联高亮 + Tooltip）
├── B2. 属性面板模式重构（从页面切换 → 属性编辑器）
├── B3. Ribbon 动效与上下文衔接优化
├── B4. 3D 交互增强（场景点选/位姿可视化）
├── B5. 图表可视化增强（力矩/轨迹/工作空间）
└── B6. i18n 基础设施

阶段 C（各模块代码优化）
├── C1. Requirement 模块（表单交互 + KeyPose 增强）
├── C2. Topology 模块（模板预览 + 参数可视化）
├── C3. Kinematics 模块（FK 实时拖拽 + IK 多解）
├── C4. Dynamics 模块（图表增强 + 负载分析可视化 + 清理 source/ 冗余）
├── C5. Selection 模块（库浏览 + 对比视图）
├── C6. Planning 模块（轨迹预览 + 碰撞可视化）
└── C7. Scheme 模块（方案对比 + 报告预览）

阶段 D（测试与质量）
├── D1. 单元测试补充（边界/错误路径）
├── D2. 集成测试增强（场景覆盖）
├── D3. Test Fixture 工厂
├── D4. UI 自动化测试框架接入
└── D5. 性能基准测试

阶段 E（构建与部署）
├── E1. VTK 依赖解耦（编译时可选）
├── E2. 第三方库管理标准化
├── E3. CI/CD 配置
├── E4. 安装包制作
└── E5. 开发者文档完善
```

---

## 4. 架构优化

### 4.0 代码清理（A0）

**当前问题**：
- [`modules/dynamics/source/`](modules/dynamics/source/) 目录存在大量冗余文件，与 `dto/`、`service/`、`ui/` 目录重复。
- 各模块存在 `.gitkeep` 文件在非空目录中。

**优化方案**：
- 清理 `modules/dynamics/source/` 目录中的所有冗余文件
- 移除无意义的 `.gitkeep` 文件
- 统一文件编码（部分文件 BOM、部分文件无 BOM）

**涉及文件**：
- [`modules/dynamics/source/`](modules/dynamics/source/) 全部文件
- 各模块 `.gitkeep` 文件（保留空目录的 `.gitkeep`）

**验收标准**：`modules/dynamics/source/` 目录被移除，编译无影响。

### 4.1 统一存储抽象层（A1）

**当前问题**：7 个模块各自实现 `*JsonStorage`，模式高度重复。

**优化方案**：
- 引入泛型模板基类 `AbstractJsonStorage<T>`，封装 `Save`/`Load`/`Exists`/`Backup` 通用逻辑
- 各模块仅需提供序列化/反序列化回调
- 合并 [`LocalJsonRepository`](core/repository/LocalJsonRepository.h) 与各 Storage 的职责边界

**涉及文件**：
- [`core/repository/LocalJsonRepository.h`](core/repository/LocalJsonRepository.h)
- 各模块 `persistence/*JsonStorage.h`（7 个文件）
- 新文件：`core/repository/AbstractJsonStorage.h`

**验收标准**：所有模块保存/加载功能回归正常，代码量减少 40%+

### 4.2 依赖注入基础设施（A2）

**当前问题**：Widget 直接 `new` Service/Repository/Storage，紧耦合。如 [`TopologyWidget`](modules/topology/ui/TopologyWidget.h:119-125) 直接持有 7 个依赖对象。

**优化方案**：
- 引入轻量 DI 容器（或手动 Service Locator 模式）
- Widget 通过构造函数或 setter 接收依赖
- Service 通过接口注入 Logger/Storage

**涉及文件**：
- 所有 Widget（7 个文件）
- 所有 Service（约 10 个文件）
- 新文件：`core/di/ServiceContainer.h`
- 修改：[`apps/desktop-qt/main.cpp`](apps/desktop-qt/main.cpp)、[`apps/desktop-qt/AppBootstrap.h`](apps/desktop-qt/AppBootstrap.h)

**验收标准**：Widget 可独立 Mock 测试，模块间解耦。

### 4.3 日志系统增强（A3）

**当前问题**：[`ConsoleLogger`](core/logging/ConsoleLogger.h) 无级别过滤，部分 Widget 直接持有具体类而非接口。

**优化方案**：
- 增加日志级别（DEBUG/INFO/WARN/ERROR）
- 增加日志格式化（时间戳/模块名/级别）
- 日志输出目标可配置（控制台 + 文件）
- 统一所有模块通过 `ILogger` 接口使用日志

**涉及文件**：
- [`core/logging/ConsoleLogger.h`](core/logging/ConsoleLogger.h)
- [`core/logging/ILogger.h`](core/logging/ILogger.h)
- 新文件：`core/logging/LoggerFactory.h`
- 所有直接持有 `ConsoleLogger` 的 Widget/Service（约 10 处）

**验收标准**：支持 `setLogLevel` 过滤，日志格式统一，所有日志通过接口调用。

### 4.4 错误码细化（A4）

**当前问题**：部分模块仅返回 `Ok` / `GenericError`，错误语义不足。

**优化方案**：
- 审查 [`core/errors/ErrorCode.h`](core/errors/ErrorCode.h) 现有枚举
- 为每个模块添加模块级错误码范围
- Service 层使用具体错误码替代通用错误
- 引入 `ServiceResult<T>` 泛型替换各模块独立的 `*Result` 结构

**涉及文件**：
- [`core/errors/ErrorCode.h`](core/errors/ErrorCode.h)
- 新文件：`core/common/ServiceResult.h`
- 所有 Service 的错误返回逻辑（约 20 个 `*Result` 结构）

**验收标准**：每个模块至少 5 个具体错误码，`ServiceResult<T>` 替换所有独立 Result。

### 4.5 模块懒加载（A5）

**当前问题**： [`MainWindow`](apps/desktop-qt/MainWindow.cpp:36-40) 构造时实例化全部 7 个 Widget，内存开销持续存在。

**优化方案**：
- 采用 `QStackedWidget` + 按需创建策略
- 切换到模块时首次创建，后续缓存
- 可选：超时未使用释放

**涉及文件**：
- [`apps/desktop-qt/MainWindow.cpp`](apps/desktop-qt/MainWindow.cpp)
- [`apps/desktop-qt/MainWindow.h`](apps/desktop-qt/MainWindow.h)

**验收标准**：内存占用在未使用模块时显著降低。

---

## 5. UI/UX 优化

### 5.0 统一消息/通知系统（B0）

**当前问题**：操作反馈通过 [`SetOperationMessage`](modules/topology/ui/TopologyWidget.h:104) 这样的 `QLabel` 显示，无统一的 Toast/Notification 机制。

**优化方案**：
- 引入 `NotificationManager` 单例，支持 Info/Success/Warning/Error 四级通知
- 主窗口右下角堆叠显示 Toast 通知
- 支持自动消失和手动关闭

**涉及文件**：
- 新文件：`apps/desktop-qt/widgets/common/NotificationManager.h`
- 新文件：`apps/desktop-qt/widgets/common/NotificationWidget.h`
- [`apps/desktop-qt/MainWindow.cpp`](apps/desktop-qt/MainWindow.cpp)

**验收标准**：各模块操作成功后显示 Toast 提示。

### 5.1 统一表单校验框架（B1）

**当前问题**：校验结果仅文本列表，用户需手动定位错误。

**优化方案**：
- 封装 `FormValidationDecorator`，绑定 Widget + 字段路径
- 校验失败时：红色边框 + Tooltip + 左侧指示图标
- 校验通过时：绿色对勾
- 自动滚动到第一个错误控件

**涉及文件**：
- 所有 Widget 的 `ApplyValidationResult` 逻辑
- 新文件：`apps/desktop-qt/widgets/common/FormValidationDecorator.h`

**验收标准**：校验反馈从文本列表升级为内联高亮。

### 5.2 属性面板模式重构（B2）

**当前问题**：[`MainWindow`](apps/desktop-qt/MainWindow.h:164) 的 `m_propertyStack` 为 `QStackedWidget`，右侧显示的是完整的模块页面，非标准属性编辑器。

**优化方案**：
- 将右侧面板拆分为真正的属性编辑器 + 中央内容区
- 属性编辑器采用 `QTreeView` 模式，分类折叠显示当前模块的可编辑字段
- 左侧项目树选中节点后，右侧显示该节点的属性，中央显示对应的完整页面

**涉及文件**：
- [`apps/desktop-qt/MainWindow.cpp`](apps/desktop-qt/MainWindow.cpp)
- 新文件：`apps/desktop-qt/widgets/property/PropertyPanelWidget.h`

**验收标准**：属性面板可折叠、可搜索、直观展示字段层级。

### 5.3 Ribbon 动效与上下文衔接优化（B3）

**当前问题**：切换模块时无过渡动画，上下文标签卡出现/消失生硬。

**优化方案**：
- 页面切换时添加淡入/滑入动画
- Ribbon 标签卡状态与当前模块联动（自动高亮）
- 切换模块时状态栏同步更新上下文信息

**涉及文件**：
- [`apps/desktop-qt/widgets/ribbon/RibbonBarWidget.cpp`](apps/desktop-qt/widgets/ribbon/RibbonBarWidget.cpp)
- [`apps/desktop-qt/MainWindow.cpp`](apps/desktop-qt/MainWindow.cpp)

**验收标准**：模块切换有平滑过渡动画。

### 5.4 3D 交互增强（B4）

**当前问题**：3D 场景仅展示骨架/Mesh，无交互操作。

**优化方案**：
- 支持场景中点选 Link → 高亮 + 显示属性
- 支持 KeyPose 在场景中显示为坐标系标记
- 支持 IK 目标位姿的 3D 拖拽操作（第二阶段）
- 工作空间点云渲染颜色映射

**涉及文件**：
- [`apps/desktop-qt/widgets/vtk/RobotVtkView.h`](apps/desktop-qt/widgets/vtk/RobotVtkView.h)
- [`apps/desktop-qt/widgets/vtk/VtkSceneBuilder.h`](apps/desktop-qt/widgets/vtk/VtkSceneBuilder.h)

**验收标准**：场景中点选 Link 可高亮并显示名称/坐标。

### 5.5 图表可视化增强（B5）

**当前问题**：QCustomPlot 仅用于扭矩曲线，功能简单。

**优化方案**：
- 增加多曲线叠加（各关节力矩对比）
- 增加工作空间散点图（可达点云）
- 增加负载包络雷达图/柱状图
- 图表交互：缩放/平移/数据点提示

**涉及文件**：
- [`modules/dynamics/ui/DynamicsWidget.cpp`](modules/dynamics/ui/DynamicsWidget.cpp)
- [`modules/kinematics/ui/KinematicsWidget.cpp`](modules/kinematics/ui/KinematicsWidget.cpp)

**验收标准**：各模块结果图表可交互查看。

### 5.6 i18n 基础设施（B6）

**当前问题**：所有界面文字硬编码。

**优化方案**：
- 引入 `QTranslator` + `.ts` / `.qm` 文件
- 提取所有用户可见字符串到 `tr()` 调用
- 首期仅支持 `zh_CN`，预留 `en_US` 扩展

**涉及文件**：
- 所有 Widget 的 `.cpp` 文件
- 新增 `resources/i18n/` 目录

**验收标准**：所有用户可见字符串通过 `tr()` 调用。

---

## 6. 代码质量优化

### 6.1 结果类型统一

**当前问题**：每个 Service 定义独立的 `*Result` 结构，语义重叠。

**优化方案**：
- 引入泛型 `ServiceResult<T>` 模板
- 统一 `error_code`、`message`、`IsSuccess()`
- 业务数据通过模板参数 `T data` 承载

**涉及文件**：
- 所有 Service 的 `*Result` 结构（约 20 个）
- 新文件：`core/common/ServiceResult.h`

### 6.2 DTO 瘦身

**当前问题**：[`KinematicModelDto`](modules/kinematics/dto/KinematicModelDto.h:81) 膨胀至 30+ 字段，Single Responsibility 偏弱。

**优化方案**：
- 将工程元信息抽到 [`KinematicMetaDto`](modules/kinematics/dto/KinematicModelDto.h:22)（已有）
- 将后端诊断状态抽到独立 `BackendDiagnosticDto`
- 将预览相关字段抽到 `PreviewContextDto`

**涉及文件**：
- [`modules/kinematics/dto/KinematicModelDto.h`](modules/kinematics/dto/KinematicModelDto.h)
- 使用方：[`KinematicsWidget`](modules/kinematics/ui/KinematicsWidget.h)、[`KinematicsService`](modules/kinematics/service/KinematicsService.h)

### 6.3 Dynamics source/ 目录清理

**当前问题**：[`modules/dynamics/source/`](modules/dynamics/source/) 目录包含大量重复文件，属于代码迁移/合并残留。

**优化方案**：
- 逐个对比 `source/` 与正式目录文件的差异
- 如有独特内容则合并到正式目录
- 确认无独特内容后删除 `source/` 目录

**涉及文件**：
- `modules/dynamics/source/DynamicModelDto.h`
- `modules/dynamics/source/DynamicsResultDto.h`
- `modules/dynamics/source/DynamicsService.cpp` / `.h`
- `modules/dynamics/source/DynamicsWidget.cpp` / `.h`
- `modules/dynamics/source/DynamicJsonStorage.cpp` / `.h`
- `modules/dynamics/source/IDynamicsBackendAdapter.h`
- `modules/dynamics/source/IDynamicsBackendDiagnosticsAdapter.h`
- `modules/dynamics/source/PinocchioDynamicsBackendAdapter.cpp` / `.h`
- `modules/dynamics/source/BenchmarkTrajectoryFactory.cpp` / `.h`
- `modules/dynamics/source/TimeParameterizationSkeleton.cpp` / `.h`

**验收标准**：`source/` 目录被安全移除，所有测试通过。

### 6.4 注释审查

**当前问题**：部分文件中文注释缺失或空泛。

**优化方案**：
- 审查所有 `.h` 和 `.cpp` 文件
- 补充缺失的核心类/方法注释
- 替换空泛注释（如"处理数据"→ 具体描述）

**涉及文件**：全部源文件

**验收标准**：代码审查中无空泛注释问题。

### 6.5 硬编码字符串抽取

**当前问题**：界面文字、错误消息、日志字符串均硬编码在 `.cpp` 文件中。

**优化方案**：
- 将 UI 显示字符串统一使用 `tr()` 包裹
- 将错误消息字符串集中到 `ErrorInfo.h` 或各模块的消息定义中
- 将日志格式字符串统一管理

**涉及文件**：所有 Widget 和 Service 的 `.cpp` 文件

**验收标准**：无未包裹 `tr()` 的用户可见字符串。

---

## 7. 性能优化

### 7.1 VTK 增量更新

**当前问题**：每次参数变化都触发 `ClearCache` + 重建 Actor。

**优化方案**：
- 区分结构变化（DH 参数）与姿态变化（关节角度）
- 结构变化：重建场景
- 姿态变化：仅更新 `vtkTransform`（已有 [`UpdatePreviewPoses`](apps/desktop-qt/widgets/vtk/RobotVtkView.h:66) 接口，需验证使用充分性）

**涉及文件**：
- [`apps/desktop-qt/widgets/vtk/RobotVtkView.cpp`](apps/desktop-qt/widgets/vtk/RobotVtkView.cpp)
- [`modules/kinematics/ui/KinematicsWidget.cpp`](modules/kinematics/ui/KinematicsWidget.cpp)

### 7.2 异步保存

**当前问题**：JSON 保存/加载在主线程同步阻塞。

**优化方案**：
- 使用 `QtConcurrent::run` 将文件 I/O 移至后台线程
- 保存期间显示进度条/禁用保存按钮
- 加载完成后通过信号通知 UI 刷新

**涉及文件**：
- [`core/repository/LocalJsonRepository.cpp`](core/repository/LocalJsonRepository.cpp)
- 各模块 Save/Load 逻辑

### 7.3 模板/元件库缓存

**当前问题**：每次调用重新解析 JSON 文件。

**优化方案**：
- 引入 `TemplateCache` + `ComponentCache` 单例
- 首次加载后缓存解析结果
- 文件修改时间检测自动失效

**涉及文件**：
- [`modules/topology/service/TopologyTemplateLoader.h`](modules/topology/service/TopologyTemplateLoader.h)
- [`modules/selection/catalog/JsonComponentCatalog.h`](modules/selection/catalog/JsonComponentCatalog.h)

---

## 8. 测试优化

### 8.1 单元测试补充（D1）

**当前问题**：各模块仅 1-2 个 Smoke Test，覆盖率低。

**优化方案**：

| 模块 | 现有测试数 | 目标测试数 | 补充场景 |
|------|-----------|-----------|---------|
| Requirement | 1 | 5+ | 边界值、必填校验、多工位、JSON 往返 |
| Topology | 1 | 5+ | 模板加载、尺寸校验、推荐逻辑 |
| Kinematics | 7 | 10+ | IK 边界、工作空间采样、URDF 导入 |
| Dynamics | 5 | 10+ | 逆动力学异常、轨迹生成、惯量校验 |
| Selection | 1 | 5+ | 电机/减速器/驱动链匹配逻辑 |
| Planning | 1 | 5+ | gRPC 通信、场景构建、结果解析 |
| Scheme | 1 | 3+ | 快照聚合、JSON 导出 |
| Schema | 3 | 6+ | 版本迁移、引用检查、一致性 |

**涉及文件**：`tests/unit/*/CMakeLists.txt` 及新增测试文件

### 8.2 集成测试增强（D2）

**当前问题**：集成测试仅覆盖主链 Happy Path。

**优化方案**：
- 新增场景：错误恢复（保存时磁盘满）、版本迁移（v0.1 → v0.2）、并发保存
- 新增场景：无上游数据时模块降级行为

**涉及文件**：
- [`tests/integration/mainchain/ProjectMainChainIntegrationTest.cpp`](tests/integration/mainchain/ProjectMainChainIntegrationTest.cpp)

### 8.3 Test Fixture 工厂（D3）

**当前问题**：各测试自行构造数据，不可复用。

**优化方案**：
- 创建 `TestModelFactory`，提供各模块默认 DTO 构建方法
- 提供 Builder 模式定制特定字段

**新文件**：`tests/fixtures/TestModelFactory.h`

---

## 9. 构建与工具链优化

### 9.1 VTK 依赖解耦（E1）

**当前问题**：主程序硬依赖 VTK，无 VTK 环境无法编译。

**优化方案**：
- `find_package(VTK)` 设为可选
- 无 VTK 时 `RobotVtkView` 退化为纯文本/2D 模式
- 通过 `#if defined(ROBOSDP_HAVE_VTK)` 条件编译（已有部分基础）

**涉及文件**：
- [`apps/desktop-qt/CMakeLists.txt`](apps/desktop-qt/CMakeLists.txt)
- [`apps/desktop-qt/widgets/vtk/RobotVtkView.h`](apps/desktop-qt/widgets/vtk/RobotVtkView.h)

### 9.2 第三方库管理（E2）

**当前问题**：QCustomPlot 直接源码拷贝。

**优化方案**：
- QCustomPlot 改为 git submodule 或 FetchContent 方式引入
- 第三方库统一目录：`third_party/`

**涉及文件**：
- 顶层 [`CMakeLists.txt`](CMakeLists.txt)
- [`apps/desktop-qt/third_party/qcustomplot/`](apps/desktop-qt/third_party/qcustomplot/)

---

## 10. 文档优化

### 10.1 开发者文档

- 补充 `CONTRIBUTING.md`：环境搭建、编译步骤、代码规范
- 补充模块交互图（module dependency graph）
- 补充 DTO 变更指南

### 10.2 API 文档

- Doxygen 配置，生成 API 参考文档
- 核心 Service 接口文档补全

---

## 11. 风险与依赖

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| 存储抽象重构可能破坏现有项目加载 | 高 | 充分的回归测试 + 版本兼容层 |
| VTK 解耦可能导致 3D 功能受限 | 中 | 保持现有接口不变，仅添加编译开关 |
| i18n 修改量巨大 | 中 | 分模块逐步迁移，不要求一次性完成 |
| 依赖注入引入过度设计 | 低 | 保持轻量，不引入第三方 DI 框架 |
| Dynamics source/ 清理可能丢失独有代码 | 中 | 逐文件 diff 对比后再删除，确保无内容丢失 |
| 测试补充耗时过长 | 中 | 优先覆盖核心路径 + 边界条件 |

---

## 12. 各模块详细优化计划索引

以下各模块的详细优化计划将在独立文档中描述：

| 文档 | 模块 | 主要内容 |
|------|------|----------|
| [`docs/optimization-plan-requirement.md`](docs/optimization-plan-requirement.md) | Requirement | KeyPose 3D 交互、多工况管理、表单增强 |
| [`docs/optimization-plan-topology.md`](docs/optimization-plan-topology.md) | Topology | 模板预览可视化、轴线关系可视化 |
| [`docs/optimization-plan-kinematics.md`](docs/optimization-plan-kinematics.md) | Kinematics | FK 实时拖拽、IK 多解展示、DTO 瘦身 |
| [`docs/optimization-plan-dynamics.md`](docs/optimization-plan-dynamics.md) | Dynamics | source/ 清理、图表增强、负载可视化 |
| [`docs/optimization-plan-selection.md`](docs/optimization-plan-selection.md) | Selection | 库浏览、对比视图、匹配逻辑可视化 |
| [`docs/optimization-plan-planning.md`](docs/optimization-plan-planning.md) | Planning | 轨迹预览、碰撞可视化、gRPC 通信增强 |
| [`docs/optimization-plan-scheme.md`](docs/optimization-plan-scheme.md) | Scheme | 方案对比、报告预览、导出增强 |

---

## 13. 附录：优化优先级矩阵

| 优先级 | 优化项 | 影响面 | 工作量 | 依赖 |
|--------|--------|--------|--------|------|
| P0 | 代码清理: Dynamics source/ 目录 (A0) | 1 模块 | S | 无 |
| P0 | 统一存储抽象层 (A1) | 全模块 | M | 无 |
| P0 | 统一表单校验框架 (B1) | 全模块 | M | 无 |
| P0 | 统一消息/通知系统 (B0) | 全模块 | S | 无 |
| P1 | 依赖注入基础设施 (A2) | 全模块 | L | A1 |
| P1 | 模块懒加载 (A5) | 主窗口 | S | 无 |
| P1 | 错误码细化 + ServiceResult (A4/C3) | 全模块 | M | 无 |
| P1 | 单元测试补充 (D1) | 全模块 | L | 无 |
| P2 | 属性面板重构 (B2) | 主窗口 | L | A2 |
| P2 | 3D 交互增强 (B4) | VTK | L | 无 |
| P2 | 图表可视化增强 (B5) | 动力学/运动学 | M | 无 |
| P2 | VTK 依赖解耦 (E1) | 构建 | M | 无 |
| P2 | DTO 瘦身 (C2) | Kinematics | M | 无 |
| P2 | 硬编码字符串抽取 (6.5) | 全模块 | L | B6 |
| P3 | i18n 基础设施 (B6) | 全模块 | XL | 无 |
| P3 | 异步保存 (7.2) | 全模块 | M | A1 |
| P3 | 模板/缓存优化 (7.3) | Topology/Selection | S | 无 |
| P3 | 集成测试增强 (D2) | 测试 | M | D1 |
| P3 | 第三方库管理 (E2) | 构建 | S | 无 |
| P3 | 注释审查 (6.4) | 全模块 | M | 无 |
| P3 | Ribbon 动效优化 (B3) | 主窗口 | M | 无 |

> **P0** = 必须优先处理，阻塞后续工作  
> **P1** = 重要，建议本迭代完成  
> **P2** = 有价值，可在后续迭代处理  
> **P3** = 锦上添花，资源充裕时处理  
> **S** = 小（< 1 天），**M** = 中（1-3 天），**L** = 大（1 周），**XL** = 超大（2 周+）

---

## 变更记录

| 版本 | 日期 | 变更内容 | 作者 |
|------|------|---------|------|
| v0.1 | 2026-05-03 | 初始版本，覆盖整体优化计划 | - |
| v0.2 | 2026-05-03 | 增补 Dynamics source/ 清理、统一消息系统、硬编码字符串抽取、代码清理 A0、各模块详细计划索引 | Roo |
