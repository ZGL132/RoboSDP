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
class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QTextEdit;
class QSpinBox;
class QTableWidget;
class QTimer;

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

    /// @brief 供顶部 Ribbon 调用的受控入口：复用页面现有“导入 URDF”流程，不在 MainWindow 中复制业务逻辑。
    void TriggerImportUrdf();

signals:
    /// @brief 将 Kinematics 操作消息抛给主窗口底部日志面板。
    void LogMessageGenerated(const QString& message);

    /// @brief 将 Kinematics 当前引擎状态遥测抛给主窗口状态栏。
    void TelemetryStatusGenerated(const QString& message, bool warning);

    /// @brief 将中央三维主视图区所需的骨架预览场景发送给主窗口；当前可来自 URDF 或 DH/MDH 主链。
    void PreviewSceneGenerated(const RoboSDP::Kinematics::Ui::PreviewSceneDto& scene);

    /// @brief 将共享内核 FK 输出的 link 全局位姿发送给中央三维视图，用于只更新 Actor 矩阵。
    void PreviewPosesUpdated(const RoboSDP::Kinematics::Ui::PreviewPoseMap& linkWorldPoses);

    void KinematicsPreviewGenerated(const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& scene);
private:
    //界面的总入口。它负责创建主体的布局、顶部的操作按钮条，并组合下方的各个标签页（Tabs）。
    void BuildUi();
    //创建“模型与坐标系”面板，包含模型名称输入、坐标系（Base、TCP、Flange 等）的 6 自由度输入框。
    QGroupBox* CreateModelGroup();
    // 创建用于展示连杆参数的“DH/MDH 参数表”面板。
    QGroupBox* CreateDhTableGroup();
    // 创建用于设置关节软/硬限位、速度等参数的“关节限位表”面板。
    QGroupBox* CreateJointLimitGroup();
    // 创建包含正解（FK）滑块、逆解（IK）目标输入框以及求解算法设置的面板。
    QGroupBox* CreateSolverGroup();
    // 创建底部的“结果摘要”只读文本框面板，用于输出日志。
    QGroupBox* CreateResultGroup();
    // 一个 UI 工具函数，把上面创建的那些面板套进一个可滚动的区域中，防止界面内容过多导致显示不全。
    QWidget* CreateScrollableTab(QWidget* contentWidget);
    // 工厂函数，用来快速生成具有统一格式（如小数位数、步长）的浮点数输入框。
    QDoubleSpinBox* CreateDoubleSpinBox(double minimum, double maximum, int decimals, double step);
    void SetupDhTableColumns();
    void SetupJointLimitTableColumns();
    // 脏检查机制。监听所有输入框的改动，一旦用户修改了任何值，就调用 MarkDirty() 标记当前草稿“已修改、未保存”。
    void ConnectDirtyTracking();
    void MarkDirty();
    void MarkClean();

// 这些函数负责在“界面（UI）”和“后台数据模型（Model）”之间来回搬运数据，并控制界面的只读/可编辑状态，以及根据后台计算结果更新界面显示。
    // 读 UI。遍历当前页面所有的表格和输入框，把用户填写的数值打包成一个完整的数据结构（KinematicModelDto），供后台计算使用。
    RoboSDP::Kinematics::Dto::KinematicModelDto CollectModelFromForm() const;
    // 写 UI。把后台传来的机器人模型数据填充到各个页面的表格和输入框中。
    void PopulateForm(const RoboSDP::Kinematics::Dto::KinematicModelDto& model);
    /// @brief 检查后台计算引擎的状态，判断目前的模型是否准备好进行 FK/IK 计算，并生成诊断信息。
    void RefreshBackendDiagnostics();
    // 把当前模型的参数、FK/IK 的计算结果、诊断信息拼接成一段长文本，显示在“结果摘要”面板中。
    void RenderResults();
    /// @brief 根据当前主模型归属刷新参数编辑区可用性；URDF 主模型下 DH 草案默认只读。
    // 根据当前的“主模型”是 URDF 还是 DH 参数，来控制 DH 表格是否允许用户修改（URDF 模式下 DH 表格是锁死只读的）。
    void RefreshEditingState();
    /// @brief 根据执行结果更新顶部操作提示；warning 为 true 时使用警告色而不是成功色。
    void SetOperationMessage(const QString& message, bool success, bool warning = false);
    /// @brief 将当前操作状态同步给主窗口状态栏，避免用户在 UI 中处于“盲飞”状态。
    void EmitTelemetryStatus(const QString& engineName, const QString& message, bool warning);
    /// @brief 按当前中央预览来源调度刷新：URDF 预览走位姿更新，DH 主链走骨架重建。
    void ScheduleActivePreviewUpdate();
    /// @brief 真正执行一次活跃预览刷新，统一托管 URDF 姿态更新与 DH 骨架重建。
    void FlushActivePreviewUpdate();
    /// @brief 将高频关节输入合并为低频刷新请求，避免拖动 SpinBox 时阻塞 UI 线程。
    void SchedulePreviewPoseUpdate();
    /// @brief 真正执行一次预览姿态刷新：调用 Service 计算 FK，并通过信号交给中央 VTK 视图。
    void FlushPreviewPoseUpdate();
    /// @brief 在 DH/MDH 主链下排队刷新骨架场景，避免表格连续编辑时阻塞 UI。
    void ScheduleDhPreviewSceneUpdate();
    /// @brief 基于当前表单数据重建 DH/MDH 骨架预览，并同步给中央三维主视图区。
    void FlushDhPreviewSceneUpdate();
    /// @brief 清空当前中央预览上下文，模型结构切换时避免中央 VTK 继续复用旧骨架或 Mesh Actor。
    void ClearPreviewContext();
    /// @brief 刷新“草稿主模型 / 派生状态 / 预览来源”标签，帮助用户判断当前哪条链路在驱动三维结果。
    void RefreshModelStatusLabels();
    /// @brief 解析当前保留的原始导入 URDF 文件，供回切选择器展示“原始导入”来源。
    QString ResolveOriginalImportedUrdfMasterPath() const;
    /// @brief 解析当前项目中已生成的派生 URDF 文件，供回切选择器展示“项目派生”来源。
    QString ResolveDerivedUrdfMasterPath() const;

    std::vector<double> CollectJointInputs(const std::array<QDoubleSpinBox*, 6>& spinBoxes) const;
    void FillIkTargetFromFkResult();

    void OnImportUrdfClicked();
    void OnBuildFromTopologyClicked();
    void OnPromoteDhDraftToMasterClicked();
    void OnSwitchToUrdfMasterClicked();
    void OnRunFkClicked();
    void OnRunIkClicked();
    void OnSampleWorkspaceClicked();
    void OnSaveDraftClicked();
    void OnLoadClicked();

    void UpdateKinematicsPreview();

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
    QTimer* m_preview_pose_update_timer = nullptr;
    bool m_has_unsaved_changes = false;
    bool m_is_populating_form = false;
    QString m_preview_source_mode = QStringLiteral("none");

    QPushButton* m_import_urdf_button = nullptr;
    QPushButton* m_build_from_topology_button = nullptr;
    QPushButton* m_promote_dh_draft_button = nullptr;
    QPushButton* m_switch_to_urdf_master_button = nullptr;
    QPushButton* m_run_fk_button = nullptr;
    QPushButton* m_run_ik_button = nullptr;
    QPushButton* m_sample_workspace_button = nullptr;
    QPushButton* m_save_button = nullptr;
    QPushButton* m_load_button = nullptr;
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

    std::array<QDoubleSpinBox*, 6> m_fk_joint_spins {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    std::array<QDoubleSpinBox*, 6> m_ik_seed_joint_spins {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    std::array<QDoubleSpinBox*, 6> m_ik_target_pose_spins {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    QSpinBox* m_workspace_sample_count_spin = nullptr;

    QPlainTextEdit* m_result_summary_edit = nullptr;
};

} // namespace RoboSDP::Kinematics::Ui
