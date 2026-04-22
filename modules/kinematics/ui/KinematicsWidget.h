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
class QSpinBox;
class QTableWidget;
class QTimer;

namespace RoboSDP::Kinematics::Ui
{

/// @brief 中央预览高频刷新使用的轻量位姿表，使用别名避免 Qt moc 直接解析复杂模板签名。
using PreviewPoseMap = std::map<QString, RoboSDP::Kinematics::Dto::CartesianPoseDto>;

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

    /// @brief 将 URDF 骨架预览场景发送给中央三维视图。
    void UrdfPreviewSceneGenerated(const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& scene);

    /// @brief 将共享内核 FK 输出的 link 全局位姿发送给中央三维视图，用于只更新 Actor 矩阵。
    void PreviewPosesUpdated(const RoboSDP::Kinematics::Ui::PreviewPoseMap& linkWorldPoses);

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
    /// @brief 刷新统一模型语义与共享内核 build-context 诊断摘要，供结果面板显示。
    void RefreshBackendDiagnostics();
    void RenderResults();
    /// @brief 根据执行结果更新顶部操作提示；warning 为 true 时使用警告色而不是成功色。
    void SetOperationMessage(const QString& message, bool success, bool warning = false);
    /// @brief 将当前操作状态同步给主窗口状态栏，避免用户在 UI 中处于“盲飞”状态。
    void EmitTelemetryStatus(const QString& engineName, const QString& message, bool warning);
    /// @brief 将高频关节输入合并为低频刷新请求，避免拖动 SpinBox 时阻塞 UI 线程。
    void SchedulePreviewPoseUpdate();
    /// @brief 真正执行一次预览姿态刷新：调用 Service 计算 FK，并通过信号交给中央 VTK 视图。
    void FlushPreviewPoseUpdate();
    /// @brief 清空当前 URDF 预览上下文，模型结构切换时避免中央 VTK 继续复用旧 Mesh Actor。
    void ClearUrdfPreviewContext();

    std::vector<double> CollectJointInputs(const std::array<QDoubleSpinBox*, 6>& spinBoxes) const;
    void FillIkTargetFromFkResult();

    void OnImportUrdfClicked();
    void OnBuildFromTopologyClicked();
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
    RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto m_urdf_preview_scene;
    RoboSDP::Kinematics::Dto::KinematicModelDto m_urdf_preview_model;
    RoboSDP::Kinematics::Dto::KinematicBackendBuildContextResultDto m_backend_diagnostic;
    QTimer* m_preview_pose_update_timer = nullptr;
    bool m_has_unsaved_changes = false;

    QPushButton* m_import_urdf_button = nullptr;
    QPushButton* m_build_from_topology_button = nullptr;
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
