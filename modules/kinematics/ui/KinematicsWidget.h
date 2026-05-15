#pragma once

#include "core/infrastructure/ProjectSaveCoordinator.h"
#include "core/logging/ILogger.h"
#include "core/repository/LocalJsonRepository.h"
#include "modules/kinematics/dto/UrdfPreviewSceneDto.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/kinematics/service/KinematicsService.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

#include <QStringList>
#include <QWidget>

#include <array>
#include <map>

class QComboBox;
class QCheckBox;
class QDoubleSpinBox;
class QGridLayout; 
class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSlider;
class QTextEdit;
class QSpinBox;
class QTableWidget;

namespace RoboSDP::Kinematics::Ui
{

/// @brief 中央预览高频刷新使用的轻量位姿表，使用别名避免 Qt moc 直接解析复杂模板签名。
using PreviewPoseMap = std::map<QString, RoboSDP::Kinematics::Dto::CartesianPoseDto>;
/// @brief 当前阶段的中央三维预览场景别名；底层仍复用既有 DTO，避免第二阶段扩大改动范围。
using PreviewSceneDto = RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto;

/**
 * @brief Kinematics 最小页面。
 *
 * 页面当前只覆盖第一阶段最小闭环所需字段：
 * 1. 从 Topology 生成 DH/MDH 参数；
 * 2. 编辑 base/flange/tcp 坐标系与最小 joint limits；
 * 3. 执行 FK / IK / 工作空间基础采样；
 * 4. 保存与加载 JSON 草稿；
 * 5. 用文本摘要展示结果。
 */
class KinematicsWidget : public QWidget, public RoboSDP::Infrastructure::IProjectSaveParticipant
{
    Q_OBJECT

public:
    explicit KinematicsWidget(RoboSDP::Logging::ILogger* logger, QWidget* parent = nullptr);

    /// @brief 返回全局保存日志使用的模块名称。
    QString ModuleName() const override;

    /// @brief 保存当前 Kinematics 草稿，供模块按钮和全局保存共用。
    RoboSDP::Infrastructure::ProjectSaveItemResult SaveCurrentDraft() override;

    /// @brief 返回当前 Kinematics 页面是否存在未保存变更。
    bool HasUnsavedChanges() const override;

    /// @brief 供顶部 Ribbon 调用的受控入口：复用页面现有"导入工程 URDF"流程，不在 MainWindow 中复制业务逻辑。
    void TriggerImportUrdf();

    // ── 以下方法由 RibbonBarWidget 工具按钮信号触发，委托到内部槽函数 ──
    /// @brief Ribbon 接口：从拓扑构型构建运动学。
    void TriggerBuildFromTopology() { OnBuildFromTopologyClicked(); }
    /// @brief Ribbon 接口：复制 URDF 诊断草案为 DH/MDH 参数化设计模型。
    void TriggerPromoteToDhMaster() { OnPromoteDhDraftToMasterClicked(); }
    /// @brief Ribbon 接口：回到原始导入的工程 URDF 参考视图。
    void TriggerSwitchToUrdfMaster() { OnSwitchToUrdfMasterClicked(); }
    /// @brief Ribbon 接口：执行正运动学求解。
    void TriggerRunFk() { OnRunFkClicked(); }
    /// @brief Ribbon 接口：执行逆运动学求解。
    void TriggerRunIk() { OnRunIkClicked(); }
    /// @brief Ribbon 接口：采样工作空间。
    void TriggerSampleWorkspace() { OnSampleWorkspaceClicked(); }
    /// @brief Ribbon 接口：保存运动学草稿。
    void TriggerSaveDraft() { OnSaveDraftClicked(); }

    // ── Ribbon 按钮状态查询 ──────────────────────────────────
    /// @brief 查询是否可导入工程 URDF（通常始终可用）。
    bool CanImportUrdf() const { return true; }
    /// @brief 查询是否可从拓扑构型构建运动学。
    bool CanBuildFromTopology() const;
    /// @brief 查询是否可复制 URDF 诊断草案为 DH/MDH 参数化设计模型。
    bool CanPromoteToDhMaster() const;
    /// @brief 查询是否可回到原始导入的工程 URDF 参考视图。
    bool CanSwitchToUrdfMaster() const;
    /// @brief 查询是否可执行正运动学（模型已加载）。
    bool CanRunFk() const;
    /// @brief 查询是否可执行逆运动学（模型已加载）。
    bool CanRunIk() const;
    /// @brief 查询是否可采样工作空间。
    bool CanSampleWorkspace() const;
    /// @brief 查询当前草稿是否可保存。
    bool CanSaveDraft() const;

    // ============================================================
    // 【逆向驱动】槽：由 VTK 3D 视图信号触发
    // ============================================================

    /// @brief 3D 视图中鼠标滚轮滚动关节角度时触发，修改 FK 滑块值并刷新姿态。
    void HandleJointAngleScrolled(int jointIndex, double deltaDeg);

    /// @brief TCP Gizmo 拖动时触发，填充 IK 目标位姿并执行逆运动学求解。
    void HandleTcpPoseDragged(const RoboSDP::Kinematics::Dto::CartesianPoseDto& newPose);

    /// @brief 3D 视图拾取 link 后，同步选中 DH 参数表中的对应行。
    void HandlePreviewLinkPicked(const QString& linkName);

signals:
    /// @brief 将 Kinematics 操作消息抛给主窗口底部日志面板。
    void LogMessageGenerated(const QString& message);

    /// @brief 将 Kinematics 当前引擎状态遥测抛给主窗口状态栏。
    void TelemetryStatusGenerated(const QString& message, bool warning);

    /// @brief 将中央三维主视图区所需的骨架预览场景发送给主窗口；当前可来自 URDF 或 DH/MDH 主链。
    void PreviewSceneGenerated(const RoboSDP::Kinematics::Ui::PreviewSceneDto& scene);

    /// @brief 将共享内核 FK 输出的 link 全局位姿发送给中央三维视图，用于只更新 Actor 矩阵。
    void PreviewPosesUpdated(const RoboSDP::Kinematics::Ui::PreviewPoseMap& linkWorldPoses);

    /// @brief 将工作空间采样点云数据发送给中央三维视图，用于渲染 3D 散点图。
    /// @param tcpPositions 所有可达采样点的 TCP 位置坐标列表，每个元素为 [x, y, z]。
    void WorkspacePointCloudGenerated(const std::vector<std::array<double, 3>>& tcpPositions);

    /// @brief 奇异区分析点云（带奇异标记），绿色=正常，红色=奇异。
    /// @param tcpPositions TCP 位置坐标列表
    /// @param isSingular 每个点是否奇异
    void SingularityPointCloudGenerated(
        const std::vector<std::array<double, 3>>& tcpPositions,
        const std::vector<bool>& isSingular);

    /// @brief Kinematics 页面内部状态变更信号，通知 MainWindow 重新查询按钮启用状态。
    void StatusChanged();

    /// @brief 用户调整滚轮灵敏度时发射，通知 3D 视图更新步长。
    void signalScrollStepChanged(double stepDeg);

private:
    void BuildUi();
    QWidget* CreateModelGroup();
    QWidget* CreateDesignInputPage();
    QWidget* CreateVerificationPage();
    QWidget* CreateDiagnosticsPage();
    QGroupBox* CreateDhTableGroup();
    QGroupBox* CreateJointLimitGroup();
    QGroupBox* CreateSolverGroup();
    QWidget* CreateSolverConfigPage();
    QWidget* CreateInteractivePage();
    QWidget* CreateAdvancedAnalysisPage();
    QGroupBox* CreateResultGroup();
    QWidget* CreateScrollableTab(QWidget* contentWidget);
    QDoubleSpinBox* CreateDoubleSpinBox(double minimum, double maximum, int decimals, double step);
    void SetupDhTableColumns();
    void SetupJointLimitTableColumns();
    void ConnectDirtyTracking();
    void MarkDirty();
    void MarkClean();

    RoboSDP::Kinematics::Dto::KinematicModelDto CollectModelFromForm() const;
    void PopulateForm(const RoboSDP::Kinematics::Dto::KinematicModelDto& model);
    void RefreshBackendDiagnostics();
    void RenderResults();
    void RefreshEditingState();
    void RefreshValidationState();
    void RefreshHeaderBadges();
    QStringList BuildValidationIssues(const RoboSDP::Kinematics::Dto::KinematicModelDto& model) const;
    void RefreshValidationHighlights(const RoboSDP::Kinematics::Dto::KinematicModelDto& model);
    void SetOperationMessage(const QString& message, bool success, bool warning = false);
    void EmitTelemetryStatus(const QString& engineName, const QString& message, bool warning);
    
    void ClearPreviewContext();
    void RefreshModelStatusLabels();
    QString ResolveOriginalImportedUrdfMasterPath() const;
    QString ResolveDerivedUrdfMasterPath() const;

    // 替换为新的签名，并增加一个动态适配方法的声明：
    std::vector<double> CollectJointInputs(const std::vector<QDoubleSpinBox*>& spinBoxes) const;
    void AdjustJointInputCount(int jointCount);

    /// @brief 将所有 FK 关节输入框的 singleStep 设置为指定值。
    void ApplyStepToAllSpinBoxes(double stepDeg);

    void FillIkTargetFromFkResult();

    /// @brief 从关节限位表读取 soft_limit 并更新 FK 滑块标签显示范围。
    void UpdateFkJointLimitLabels();

    /// @brief 检查 FK 关节角度是否超出 soft_limit，越界时设置红色背景提示。
    void UpdateJointLimitWarningStyle();

    /// @brief 计算每个 FK 关节距离软限位的归一化裕量，更新裕量标签颜色。
    void UpdateJointLimitMargins();

    // =========================================================================
    // 核心生命周期分流 (单向数据流核心)
    // =========================================================================
    /// @brief 通道 A (重量级)：当 DH 表、Base/TCP 等结构尺寸改变时调用。
    void SyncStructureAndPreview();

    /// @brief 通道 B (轻量级)：当用户仅拖动 FK 关节滑块时调用。
    void SyncPoseOnly();

    void OnImportUrdfClicked();
    void OnBuildFromTopologyClicked();
    void OnCopyUrdfDraftToDhClicked();
    void OnReturnToUrdfReferenceClicked();
    void OnPromoteDhDraftToMasterClicked();
    void OnSwitchToUrdfMasterClicked();
    void OnRunFkClicked();
    void OnRunIkClicked();
    void OnSampleWorkspaceClicked();
    void OnCheckReachabilityClicked();
    void OnSingularityAnalysisClicked();
    void OnOrientationReachabilityClicked();
    void OnSaveDraftClicked();
    void OnLoadClicked();

private:
    RoboSDP::Logging::ILogger* m_logger = nullptr;
    RoboSDP::Repository::LocalJsonRepository m_repository;
    RoboSDP::Topology::Persistence::TopologyJsonStorage m_topology_storage;
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage m_kinematic_storage;
    RoboSDP::Kinematics::Service::KinematicsService m_service;
    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto m_state;
    RoboSDP::Kinematics::Dto::JacobianAnalysisDto m_last_jacobian_analysis;
    PreviewSceneDto m_preview_scene;
    RoboSDP::Kinematics::Dto::KinematicModelDto m_preview_model;
    RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto m_backend_diagnostic;
    
    bool m_has_unsaved_changes = false;
    bool m_is_populating_form = false;
    QString m_preview_source_mode = QStringLiteral("none");

    QLabel* m_operation_label = nullptr;
    QLabel* m_validation_label = nullptr;
    QLabel* m_validation_badge_label = nullptr;
    QLabel* m_dirty_badge_label = nullptr;
    QLabel* m_sync_badge_label = nullptr;
    QLabel* m_source_badge_label = nullptr;

    QLineEdit* m_model_name_edit = nullptr;
    QComboBox* m_parameter_convention_combo = nullptr;
    QLineEdit* m_topology_ref_edit = nullptr;
    QLineEdit* m_requirement_ref_edit = nullptr;
    QLabel* m_master_model_mode_label = nullptr;
    QLabel* m_derived_model_state_label = nullptr;
    QLabel* m_preview_source_label = nullptr;
    QLabel* m_master_switch_state_label = nullptr;
    QLabel* m_urdf_source_type_label = nullptr;
    QLabel* m_dh_draft_level_label = nullptr;
    QLabel* m_dh_draft_status_label = nullptr;
    QLabel* m_dh_readonly_banner_label = nullptr;
    std::array<QDoubleSpinBox*, 6> m_base_frame_spins {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    std::array<QDoubleSpinBox*, 6> m_flange_frame_spins {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    QCheckBox* m_tool_frame_enabled_check = nullptr;
    std::array<QDoubleSpinBox*, 6> m_tool_frame_spins {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    QCheckBox* m_workpiece_frame_enabled_check = nullptr;
    std::array<QDoubleSpinBox*, 6> m_workpiece_frame_spins {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    std::array<QDoubleSpinBox*, 6> m_tcp_frame_spins {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    QTableWidget* m_dh_table = nullptr;
    QTableWidget* m_joint_limit_table = nullptr;

    QComboBox* m_solver_type_combo = nullptr;
    QComboBox* m_branch_policy_combo = nullptr;
    QSpinBox* m_max_iterations_spin = nullptr;
    QDoubleSpinBox* m_position_tolerance_spin = nullptr;
    QDoubleSpinBox* m_orientation_tolerance_spin = nullptr;
    QDoubleSpinBox* m_step_gain_spin = nullptr;

    /// @brief IK 多解浏览器：解索引下拉框
    QComboBox* m_ik_solution_combo = nullptr;
    /// @brief IK 多解浏览器：当前选中解的关节角只读 spinbox
    std::vector<QDoubleSpinBox*> m_ik_solution_spins;

    std::vector<QDoubleSpinBox*> m_fk_joint_spins;
    std::vector<QSlider*> m_fk_joint_sliders;
    std::vector<QLabel*> m_fk_joint_labels;
    std::vector<QLabel*> m_fk_margin_labels;  // 关节限位裕量百分比标签
    QGridLayout* m_fk_grid = nullptr;

    /// @brief 滚轮灵敏度（度/滚轮格），默认 1.0°；联动控制 3D 视图滚轮步长与 FK 输入框 singleStep。
    QDoubleSpinBox* m_scroll_step_spin = nullptr;

    std::vector<QDoubleSpinBox*> m_ik_seed_joint_spins;
    std::vector<QLabel*> m_ik_seed_joint_labels;
    QGridLayout* m_ik_seed_grid = nullptr;
    std::array<QDoubleSpinBox*, 6> m_ik_target_pose_spins {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    QSpinBox* m_workspace_sample_count_spin = nullptr;
    QDoubleSpinBox* m_singularity_threshold_spin = nullptr;

    QLabel* m_singularity_result_label = nullptr;

    /// @brief 姿态可达性分析组件
    std::array<QDoubleSpinBox*, 3> m_orient_pos_spins {nullptr, nullptr, nullptr};
    QSpinBox* m_orient_steps_spin = nullptr;
    QDoubleSpinBox* m_orient_range_spin = nullptr;
    QLabel* m_orient_result_label = nullptr;

    /// @brief 关键工位可达性检测组件
    std::array<QDoubleSpinBox*, 6> m_reach_target_spins {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    QSpinBox* m_reach_seed_count_spin = nullptr;
    QLabel* m_reach_result_label = nullptr;

    QPlainTextEdit* m_result_summary_edit = nullptr;

    /// @brief 结构化结果展示：各分类 QGroupBox 内的信息标签
    QLabel* m_result_model_label = nullptr;      // 模型概要
    QLabel* m_result_diag_label = nullptr;       // 诊断信息
    QLabel* m_result_validation_label = nullptr; // 模型校验明细
    QLabel* m_result_fk_label = nullptr;         // FK 结果
    QLabel* m_result_fk_manipulability_label = nullptr; // 可操作度指标
    QLabel* m_result_ik_label = nullptr;         // IK 结果
    QLabel* m_result_ws_label = nullptr;         // 工作空间结果
};

} // namespace RoboSDP::Kinematics::Ui
