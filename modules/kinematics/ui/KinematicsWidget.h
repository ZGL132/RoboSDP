#pragma once

#include "core/infrastructure/ProjectSaveCoordinator.h"
#include "core/logging/ConsoleLogger.h"
#include "core/repository/LocalJsonRepository.h"
#include "modules/kinematics/dto/UrdfPreviewSceneDto.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/kinematics/service/KinematicsService.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

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
    explicit KinematicsWidget(QWidget* parent = nullptr);

    /// @brief 返回全局保存日志使用的模块名称。
    QString ModuleName() const override;

    /// @brief 保存当前 Kinematics 草稿，供模块按钮和全局保存共用。
    RoboSDP::Infrastructure::ProjectSaveItemResult SaveCurrentDraft() override;

    /// @brief 返回当前 Kinematics 页面是否存在未保存变更。
    bool HasUnsavedChanges() const override;

    /// @brief 供顶部 Ribbon 调用的受控入口：复用页面现有"导入 URDF"流程，不在 MainWindow 中复制业务逻辑。
    void TriggerImportUrdf();

    // ── 以下方法由 RibbonBarWidget 工具按钮信号触发，委托到内部槽函数 ──
    /// @brief Ribbon 接口：从拓扑构型构建运动学。
    void TriggerBuildFromTopology() { OnBuildFromTopologyClicked(); }
    /// @brief Ribbon 接口：提升为 DH 主模型。
    void TriggerPromoteToDhMaster() { OnPromoteDhDraftToMasterClicked(); }
    /// @brief Ribbon 接口：切换回 URDF 主模型。
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
    /// @brief 查询是否可导入 URDF（通常始终可用）。
    bool CanImportUrdf() const { return true; }
    /// @brief 查询是否可从拓扑构型构建运动学。
    bool CanBuildFromTopology() const;
    /// @brief 查询是否可提升为 DH 主模型。
    bool CanPromoteToDhMaster() const;
    /// @brief 查询是否可切换回 URDF 主模型。
    bool CanSwitchToUrdfMaster() const;
    /// @brief 查询是否可执行正运动学（模型已加载）。
    bool CanRunFk() const;
    /// @brief 查询是否可执行逆运动学（模型已加载）。
    bool CanRunIk() const;
    /// @brief 查询是否可采样工作空间。
    bool CanSampleWorkspace() const;
    /// @brief 查询当前草稿是否可保存。
    bool CanSaveDraft() const;

signals:
    /// @brief 将 Kinematics 操作消息抛给主窗口底部日志面板。
    void LogMessageGenerated(const QString& message);

    /// @brief 将 Kinematics 当前引擎状态遥测抛给主窗口状态栏。
    void TelemetryStatusGenerated(const QString& message, bool warning);

    /// @brief 将中央三维主视图区所需的骨架预览场景发送给主窗口；当前可来自 URDF 或 DH/MDH 主链。
    void PreviewSceneGenerated(const RoboSDP::Kinematics::Ui::PreviewSceneDto& scene);

    /// @brief 将共享内核 FK 输出的 link 全局位姿发送给中央三维视图，用于只更新 Actor 矩阵。
    void PreviewPosesUpdated(const RoboSDP::Kinematics::Ui::PreviewPoseMap& linkWorldPoses);

    /// @brief Kinematics 页面内部状态变更信号，通知 MainWindow 重新查询按钮启用状态。
    void StatusChanged();

private:
    void BuildUi();
    QGroupBox* CreateModelGroup();
    QGroupBox* CreateDhTableGroup();
    QGroupBox* CreateJointLimitGroup();
    QGroupBox* CreateSolverGroup();
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
    void SetOperationMessage(const QString& message, bool success, bool warning = false);
    void EmitTelemetryStatus(const QString& engineName, const QString& message, bool warning);
    
    void ClearPreviewContext();
    void RefreshModelStatusLabels();
    QString ResolveOriginalImportedUrdfMasterPath() const;
    QString ResolveDerivedUrdfMasterPath() const;

    // 替换为新的签名，并增加一个动态适配方法的声明：
    std::vector<double> CollectJointInputs(const std::vector<QDoubleSpinBox*>& spinBoxes) const;
    void AdjustJointInputCount(int jointCount);
    void FillIkTargetFromFkResult();

    // =========================================================================
    // 核心生命周期分流 (单向数据流核心)
    // =========================================================================
    /// @brief 通道 A (重量级)：当 DH 表、Base/TCP 等结构尺寸改变时调用。
    void SyncStructureAndPreview();

    /// @brief 通道 B (轻量级)：当用户仅拖动 FK 关节滑块时调用。
    void SyncPoseOnly();

    void OnImportUrdfClicked();
    void OnBuildFromTopologyClicked();
    void OnPromoteDhDraftToMasterClicked();
    void OnSwitchToUrdfMasterClicked();
    void OnRunFkClicked();
    void OnRunIkClicked();
    void OnSampleWorkspaceClicked();
    void OnSaveDraftClicked();
    void OnLoadClicked();

private:
    RoboSDP::Logging::ConsoleLogger m_logger;
    RoboSDP::Repository::LocalJsonRepository m_repository;
    RoboSDP::Topology::Persistence::TopologyJsonStorage m_topology_storage;
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage m_kinematic_storage;
    RoboSDP::Kinematics::Service::KinematicsService m_service;
    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto m_state;
    PreviewSceneDto m_preview_scene;
    RoboSDP::Kinematics::Dto::KinematicModelDto m_preview_model;
    RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto m_backend_diagnostic;
    
    bool m_has_unsaved_changes = false;
    bool m_is_populating_form = false;
    QString m_preview_source_mode = QStringLiteral("none");

    QLabel* m_operation_label = nullptr;

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

    std::vector<QDoubleSpinBox*> m_fk_joint_spins;
    std::vector<QLabel*> m_fk_joint_labels;
    QGridLayout* m_fk_grid = nullptr;

    std::vector<QDoubleSpinBox*> m_ik_seed_joint_spins;
    std::vector<QLabel*> m_ik_seed_joint_labels;
    QGridLayout* m_ik_seed_grid = nullptr;
    std::array<QDoubleSpinBox*, 6> m_ik_target_pose_spins {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    QSpinBox* m_workspace_sample_count_spin = nullptr;

    QPlainTextEdit* m_result_summary_edit = nullptr;
};

} // namespace RoboSDP::Kinematics::Ui
