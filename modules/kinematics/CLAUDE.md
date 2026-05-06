# Kinematics 模块完善与优化计划

## 模块现状

**文件结构**：24 源文件（8 adapter + 5 dto + 5 service + 1 persistence + 3 ui + 2 bat）

**分层架构**：
- `dto/` — 6 个 DTO 结构体，涵盖模型参数、求解结果、预览场景、诊断上下文
- `adapter/` — 3 接口 + 2 Pinocchio 实现，IKinematicBackendAdapter / IIkSolverAdapter 隔离算法库
- `service/` — KinematicsService 核心业务，FK/IK/Workspace/URDF 导入/BuildDiagnostics
- `ui/` — KinematicsWidget (~1900 行)，5 页签：模型与坐标系 / DH 参数 / 关节限位 / 求解 / 结果摘要
- `persistence/` — KinematicJsonStorage，JSON 读写

**数据流**（典型 FK 操作）：
```
用户拖动 FK 滑块 → SyncPoseOnly() → Service.SolveFk() → Adapter.SolveFk() → Pinocchio FK → 
FkResultDto → map<link,pose> → emit PreviewPosesUpdated() → RobotVtkView.UpdatePreviewPoses()
```

**A/B 双通道**：
- 通道 A (SyncStructureAndPreview)：DH 表/坐标系结构变更 → 重建引擎 + 全场景发射
- 通道 B (SyncPoseOnly)：仅 FK 关节角度变更 → SolveFk → 只更新 Actor 矩阵

---

## 优化计划（按优先级）

### P0：基础设施清理 ✅

| # | 项 | 说明 | 影响 |
|---|-----|------|------|
| 0.1 | 删除 `mv1.bat` `mv2.bat` | ✅ 已删除 | 清理 |
| 0.2 | 添加 `tests/unit/kinematics` 到 CMakeLists | ✅ 已通过 `tests/unit/CMakeLists.txt` 正确纳入构建 | 构建 |

### P1：操作易用性（操作体验提升）✅

| # | 项 | 说明 | 涉及文件 |
|---|-----|------|---------|
| 1.1 | FK 滑块关节名显示限位范围 | ✅ `UpdateFkJointLimitLabels()` 从关节限位表读取 soft_limit 显示 `J1 [-170.0, 170.0]` 格式 | KinematicsWidget.h/.cpp |
| 1.2 | FK 滑块拖拽时实时联动 3D | ✅ 已在 `AdjustJointInputCount()` spinbox 创建时连接 `SyncPoseOnly()` 到 valueChanged | KinematicsWidget.cpp |
| 1.3 | FK 结果一键填入 IK 种子 | ✅ 结果摘要区新增 "⇥ 填入 IK 种子" 按钮，调用 `FillIkTargetFromFkResult()` | KinematicsWidget.cpp |
| 1.4 | 关节限位越界视觉提示 | ✅ `UpdateJointLimitWarningStyle()` 在 valueChanged 和 SyncPoseOnly 中检测越界并设置红底红字 | KinematicsWidget.h/.cpp |
| 1.5 | IK 求解成功后自动将结果填入 FK 滑块 | ✅ `OnRunIkClicked()` 成功后遍历 `last_ik_result.joint_positions_deg` 填入 FK spinbox | KinematicsWidget.cpp |

### P2：结果可视化（显示能力提升）✅

| # | 项 | 说明 | 涉及文件 |
|---|-----|------|---------|
| 2.1 | FK 结果 TCP 坐标系在 3D 视图显示 | ✅ 新增 `m_tcp_axes_actor`，FK 成功后 tcp_frame 位姿驱动红绿蓝三轴坐标指示器 | RobotVtkView.h/.cpp |
| 2.2 | Workspace 采样结果 3D 点云 | ✅ 新增 `WorkspacePointCloudGenerated` 信号 + `ShowWorkspacePointCloud` 渲染绿色半透明散点图 | KinematicsWidget.h/.cpp, RobotVtkView.h/.cpp, MainWindow.cpp |
| 2.3 | 结果摘要页签结构化展示 | ✅ 拆分为模型概要/诊断/FK/IK/工作空间 5 个分类 QGroupBox，保留底部文本详情（默认隐藏） | KinematicsWidget.h/.cpp |
| 2.4 | IK 多解浏览 | ⏳ 依赖后续 Pinocchio 接口扩展，暂缓 | - |

### P2.5：运动学分析能力增强（基于 Jacobian 与 IK 的深度分析）🆕

基于现有的 Pinocchio Jacobian 干跑接口和 IK 求解器，扩展以下 5 项分析能力：

| # | 项 | 说明 | 涉及文件 | 技术方案 |
|---|-----|------|---------|---------|
| 2.5.1 | **关键工位可达性检测** | 给定 TCP 目标位姿（位置+姿态），通过多种子 IK 求解判断是否可达，返回可达/不可达、误差、最佳种子等信息 | `KinematicSolverResultDto.h`(新 DTO) + `IKinematicBackendAdapter.h`(新接口) + `PinocchioKinematicBackendAdapter.cpp` + `KinematicsService.h/.cpp` + `KinematicsWidget.h/.cpp` | 对每个目标位姿，从 N 个随机种子（如 20 个）启动 IK 求解，任意一个收敛即判定可达；在结果摘要区新增"工位可达性" QGroupBox，含位姿输入框（位置 x/y/z + RPY）和"检测"按钮 |
| 2.5.2 | **姿态可达性分析** | 固定 TCP 位置，采样不同姿态（RPY 组合），统计在该位置下可实现的姿态比例和包络范围 | `KinematicSolverResultDto.h`(新 DTO) + `IKinematicBackendAdapter.h`(新接口) + `PinocchioKinematicBackendAdapter.cpp` + `KinematicsService.h/.cpp` + `KinematicsWidget.h/.cpp` | 在指定位置周围按 Euler 角步长（如每轴 7 步 → 343 个姿态）采样，每个姿态运行 IK 检查可达性；结果以"姿态可达率 %"呈现，3D 视图可显示可达/不可达姿态朝向线 |
| 2.5.3 | **奇异区域识别** | 在 workspace 采样或 FK 求解时同步计算 Jacobian 条件数，标记条件数过大的构型为奇异区 | `KinematicSolverResultDto.h`(新 DTO: SingularityInfoDto) + `IKinematicBackendAdapter.h`(新接口) + `PinocchioKinematicBackendAdapter.cpp` + `KinematicsService.h/.cpp` + `KinematicsWidget.h/.cpp` + `RobotVtkView.h/.cpp` | 复用 `computeJointJacobians` + SVD 分解求 max/min 奇异值 → 条件数 κ = σ_max/σ_min；在 workspace 采样循环中同步计算，将条件数 > 阈值(如 1000)的点标记为奇异；点云用红(奇异)→绿(正常)渐变色渲染 |
| 2.5.4 | **可操作度分析** | Yoshikawa 可操作度 w = sqrt(det(J·J^T))，在 FK/采样时实时计算并展示趋势 | `KinematicSolverResultDto.h`(新 DTO: ManipulabilityResultDto) + `IKinematicBackendAdapter.h`(新接口) + `PinocchioKinematicBackendAdapter.cpp` + `KinematicsService.h/.cpp` + `KinematicsWidget.h/.cpp` + `RobotVtkView.h/.cpp` | 在 FK 求解后调用 `computeJointJacobians` 计算 J，执行 Eigen SVD(BDCSVD) 提取奇异值，计算 w = σ₁·σ₂·…·σₙ；结果摘要区显示当前 w 值 + 趋势指示(↑/↓/—)；可叠加到 workspace 点云颜色映射 |
| 2.5.5 | **关节限位与裕量** | 对当前构型计算每个关节距离软限位的裕量（归一化百分比），并按裕量大小分级预警 | `KinematicSolverResultDto.h`(新 DTO: JointLimitMarginDto) + `KinematicsWidget.h/.cpp` | 纯 UI 层 + 已有数据计算，无需后端修改：margin_i = min(q_i - soft_min_i, soft_max_i - q_i) / (soft_max_i - soft_min_i) × 100%；在 FK 滑块区每个 spinbox 旁显示裕量进度条或百分比标签（≥30% 绿色、10~30% 黄色、<10% 红色） |

#### 实现依赖关系

```
2.5.5 关节限位裕量 ← 仅依赖已有 joint_limits + FK 输入值，无其他依赖
2.5.1 工位可达性  ← 依赖 IIkSolverAdapter 多种子调用
2.5.4 可操作度     ← 依赖 Jacobian 计算（已有 EvaluateNativeJacobianDryRun）
2.5.3 奇异区域     ← 依赖 Jacobian SVD 分解（在 2.5.4 基础上扩展）
2.5.2 姿态可达性   ← 依赖 2.5.1 的多种子 IK 能力
```

执行顺序：2.5.5 → 2.5.1 → 2.5.4 → 2.5.3 → 2.5.2（由简到繁）

#### 新增 DTO 设计

```cpp
/// @brief 单点可达性检测结果（2.5.1）
struct ReachabilityCheckResultDto {
    bool reachable = false;
    CartesianPoseDto target_pose;
    std::vector<IkResultDto> ik_attempts;  // 各种子 IK 结果
    int total_seeds = 20;
    int converged_count = 0;
    double best_position_error_mm = 0.0;
    double best_orientation_error_deg = 0.0;
};

/// @brief 姿态可达性分析结果（2.5.2）
struct OrientationReachabilityResultDto {
    std::array<double, 3> position_m;
    int total_samples = 0;
    int reachable_count = 0;
    double reachability_ratio = 0.0;   // 0~1
    std::vector<CartesianPoseDto> reachable_poses;
    std::vector<CartesianPoseDto> unreachable_poses;
};

/// @brief 奇异值/条件数汇总（2.5.3 + 2.5.4）
struct JacobianAnalysisDto {
    std::vector<double> singular_values;  // 从大到小排列
    double condition_number = 0.0;
    double manipulability = 0.0;           // Yoshikawa w
    double min_singular_value = 0.0;
    double max_singular_value = 0.0;
    bool is_singular = false;
};

/// @brief 关节限位裕量（2.5.5）
struct JointLimitMarginDto {
    QString joint_id;
    double current_deg = 0.0;
    double soft_min_deg = 0.0;
    double soft_max_deg = 0.0;
    double margin_ratio = 0.0;    // 归一化裕量 0~1
    double margin_percent = 0.0;  // 百分比
    QString warning_level;        // "ok" / "caution" / "danger"
};
```

### P3：功能完整性（能力补齐）

| # | 项 | 说明 | 涉及文件 |
|---|-----|------|---------|
| 3.1 | FK 关节插值动画 | 起始角 → 目标角的线性插值，以设定步数/时间播放动画，联动 3D 视图 | KinematicsWidget + RobotVtkView |
| 3.2 | 正/逆解对比模式 | FK 和 IK 结果并排显示，支持"应用 IK 解到 FK 滑块" | KinematicsWidget.cpp |
| 3.3 | 多目标 IK 序列求解 | 允许用户输入一组 TCP 目标位姿序列，批量求解并汇总成功率 | KinematicsService + Widget |
| 3.4 | 可达性热力图 | 对 workspace 采样结果按密度/成功率着色，展示到 3D 视图 | VtkSceneBuilder + RobotVtkView |

### P4：代码可读性与可维护性

| # | 项 | 说明 | 涉及文件 |
|---|-----|------|---------|
| 4.1 | `RenderResults()` 拆分 | 将 130 行的单函数拆分为 `RenderModelSummary()` / `RenderDiagnostics()` / `RenderFkResult()` / `RenderIkResult()` / `RenderWorkspaceResult()` | KinematicsWidget.cpp |
| 4.2 | FK/IK 连接代码简化 | `CreateSolverGroup()` 中 FK/IK spinbox 创建可以提取模板方法，消除重复 | KinematicsWidget.cpp |
| 4.3 | 提取 `CreateFkJointSpinBoxes()` | 将 FK spinbox 创建逻辑从 CreateSolverGroup 独立成方法 | KinematicsWidget.cpp |
| 4.4 | `CollectModelFromForm()` 精简 | 当前包含大量重复字段收集，提取 `CollectFrame()` `CollectSolverConfig()` 等辅助方法 | KinematicsWidget.cpp |
| 4.5 | 枚举类型替换魔法字符串 | `master_model_type` `preview_source_mode` 等字段使用 `enum class` 替代 `QString` | KinematicModelDto.h + 关联文件 |

### P5：可扩展性

| # | 项 | 说明 | 涉及文件 |
|---|-----|------|---------|
| 5.1 | 支持可变关节数 | 当前硬编码 6 DOF 的假设（std::array<QDoubleSpinBox*,6>），改为动态 vector 适配 | KinematicsWidget.h/.cpp |
| 5.2 | 求解器配置 UI 增强 | 允许保存/加载多组求解器预设（如"快速"/"精确"） | KinematicsWidget.cpp |
| 5.3 | DH 表可配置列 | 允许显示/隐藏列（如 a/alpha/d/theta_offset/offset/range） | KinematicsWidget.cpp |

---

## 执行策略

1. **按优先级顺序执行**，P0→P1→P2→P2.5→P3→P4→P5
2. 每项完成后编译验证
3. P2 之前先完成 P1，确保操作基础稳固后再增强可视化
4. P4 代码重构穿插在前 3 阶段中进行（改到哪处就清理哪处）
5. 所有 UI 变更保持与现有 Ribbon 按钮状态的兼容

## 关键设计约束

- **不改动 DTO/Service/Adapter 核心接口**：除非明确需要接口扩展（如 IK 多解），否则只在 UI 层和信号链路中增强
- **A/B 通道不破坏**：SyncStructureAndPreview / SyncPoseOnly 的双通道架构保持不变，只增强各自的输出效果
- **不引入新的第三方依赖**：所有可视化增强使用现有的 VTK + Qt 能力实现
- **保持中文注释规范**：新增代码必须附带中文注释说明
