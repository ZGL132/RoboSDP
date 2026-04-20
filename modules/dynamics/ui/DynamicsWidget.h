#pragma once

#include "core/infrastructure/ProjectSaveCoordinator.h"
#include "core/logging/ConsoleLogger.h"
#include "core/repository/LocalJsonRepository.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/dynamics/service/DynamicsService.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"

#include <QString>
#include <QWidget>

class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QCustomPlot;
class QTableWidget;
class QTableWidgetItem;

namespace RoboSDP::Dynamics::Ui
{

/**
 * @brief Dynamics 最小页面骨架。
 * @details
 * 页面只负责：
 * 1. 从 Kinematics 生成一份最小 DynamicModel；
 * 2. 编辑质量、惯量和传动参数；
 * 3. 触发共享 Pinocchio 内核逆动力学主链并展示摘要；
 * 4. 保存与加载 Dynamics JSON 草稿；
 * 5. 将后端状态遥测同步到主窗口状态栏。
 */
class DynamicsWidget : public QWidget, public RoboSDP::Infrastructure::IProjectSaveParticipant
{
    Q_OBJECT

public:
    explicit DynamicsWidget(QWidget* parent = nullptr);

    /// @brief 返回全局保存日志使用的模块名称。
    QString ModuleName() const override;

    /// @brief 保存当前 Dynamics 草稿，供模块按钮和全局保存共用。
    RoboSDP::Infrastructure::ProjectSaveItemResult SaveCurrentDraft() override;

    /// @brief 供顶部 Ribbon 调用的受控入口：复用页面现有“执行逆动力学”流程，不在 MainWindow 中复制业务逻辑。
    void TriggerRunAnalysis();

signals:
    /// @brief 将 Dynamics 操作消息抛给主窗口日志面板。
    void LogMessageGenerated(const QString& message);

    /// @brief 将 Dynamics 当前引擎状态遥测同步给主窗口状态栏。
    void TelemetryStatusGenerated(const QString& message, bool warning);

private:
    void BuildUi();

    QGroupBox* CreateModelGroup();
    QGroupBox* CreateLinkTableGroup();
    QGroupBox* CreateJointDriveTableGroup();
    QGroupBox* CreateTrajectoryGroup();
    QGroupBox* CreateBackendStatusGroup();
    QGroupBox* CreatePlotGroup();
    QGroupBox* CreateResultGroup();

    QDoubleSpinBox* CreateDoubleSpinBox(double minimum, double maximum, int decimals, double step);
    void SetupLinkTableColumns();
    void SetupJointDriveTableColumns();
    void SetupTrajectoryTableColumns();
    bool ValidateTablesAndHighlight(QString* message = nullptr);
    void HighlightItem(QTableWidgetItem* item, bool isValid, const QString& tooltip);

    RoboSDP::Dynamics::Dto::DynamicModelDto CollectModelFromForm() const;
    void PopulateForm(const RoboSDP::Dynamics::Dto::DynamicModelDto& model);
    void RenderBackendStatus();
    void RenderTorquePlot();
    void RenderResults();
    void SetOperationMessage(const QString& message, bool success, bool warning = false);
    void EmitTelemetryStatus(const QString& engineName, const QString& message, bool warning);

    void OnBrowseProjectRootClicked();
    void OnBuildFromKinematicsClicked();
    void OnRunAnalysisClicked();
    void OnSaveDraftClicked();
    void OnLoadClicked();

private:
    RoboSDP::Logging::ConsoleLogger m_logger;
    RoboSDP::Repository::LocalJsonRepository m_repository;
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage m_dynamic_storage;
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage m_kinematic_storage;
    RoboSDP::Dynamics::Service::DynamicsService m_service;
    RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto m_state;

    QLineEdit* m_project_root_edit = nullptr;
    QPushButton* m_browse_button = nullptr;
    QPushButton* m_build_from_kinematics_button = nullptr;
    QPushButton* m_run_analysis_button = nullptr;
    QPushButton* m_save_button = nullptr;
    QPushButton* m_load_button = nullptr;
    QLabel* m_operation_label = nullptr;

    QLineEdit* m_model_name_edit = nullptr;
    QLineEdit* m_kinematic_ref_edit = nullptr;
    QLineEdit* m_topology_ref_edit = nullptr;
    QLineEdit* m_requirement_ref_edit = nullptr;
    QDoubleSpinBox* m_gravity_z_spin = nullptr;
    QDoubleSpinBox* m_end_effector_mass_spin = nullptr;

    QTableWidget* m_link_table = nullptr;
    QTableWidget* m_joint_drive_table = nullptr;
    QTableWidget* m_trajectory_table = nullptr;
    QLineEdit* m_solver_backend_edit = nullptr;
    QLineEdit* m_used_fallback_edit = nullptr;
    QLabel* m_backend_message_label = nullptr;
    QCustomPlot* m_torque_plot = nullptr;
    QLabel* m_torque_plot_status_label = nullptr;
    QPlainTextEdit* m_result_summary_edit = nullptr;
};

} // namespace RoboSDP::Dynamics::Ui
