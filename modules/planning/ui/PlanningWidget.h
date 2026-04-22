#pragma once

#include "core/infrastructure/ProjectSaveCoordinator.h"
#include "core/logging/ConsoleLogger.h"
#include "core/repository/LocalJsonRepository.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/planning/persistence/PlanningJsonStorage.h"
#include "modules/planning/service/PlanningVerificationService.h"
#include "modules/selection/persistence/SelectionJsonStorage.h"

#include <QString>
#include <QWidget>

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QTableWidget;
class QTableWidgetItem;

namespace RoboSDP::Planning::Ui
{

/**
 * @brief Planning Verification 最小页面骨架。
 *
 * 页面只负责：
 * 1. 从上游草稿构建最小 PlanningScene；
 * 2. 触发点到点规划验证；
 * 3. 用表格与文本摘要展示轨迹、碰撞、自碰撞和节拍结果；
 * 4. 支持 Planning JSON 保存与加载。
 */
class PlanningWidget : public QWidget, public RoboSDP::Infrastructure::IProjectSaveParticipant
{
    Q_OBJECT

public:
    explicit PlanningWidget(QWidget* parent = nullptr);

    /// @brief 返回全局保存日志使用的模块名称。
    QString ModuleName() const override;

    /// @brief 保存当前 Planning 草稿，供模块按钮和全局保存共用。
    RoboSDP::Infrastructure::ProjectSaveItemResult SaveCurrentDraft() override;

    /// @brief 返回当前 Planning 页面是否存在未保存变更。
    bool HasUnsavedChanges() const override;

signals:
    void LogMessageGenerated(const QString& message);

private:
    void BuildUi();
    QWidget* CreateScrollableTab(QWidget* contentWidget);
    void SetupJointTableColumns();
    void SetupCollisionTableColumns();
    void SetupSelfCollisionTableColumns();
    void ConnectDirtyTracking();
    void MarkDirty();
    void MarkClean();
    void PopulateForm(const RoboSDP::Planning::Dto::PlanningSceneDto& scene);
    RoboSDP::Planning::Dto::PlanningSceneDto CollectSceneFromForm() const;
    void RenderResults();
    void SetOperationMessage(const QString& message, bool success);
    bool ValidateJointTableAndHighlight(QString* message = nullptr);
    void HighlightItem(QTableWidgetItem* item, bool isValid, const QString& tooltip);

    void OnBuildSceneClicked();
    void OnRunVerificationClicked();
    void OnSaveDraftClicked();
    void OnLoadClicked();

private:
    RoboSDP::Logging::ConsoleLogger m_logger;
    RoboSDP::Repository::LocalJsonRepository m_repository;
    RoboSDP::Planning::Persistence::PlanningJsonStorage m_planning_storage;
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage m_kinematic_storage;
    RoboSDP::Selection::Persistence::SelectionJsonStorage m_selection_storage;
    RoboSDP::Planning::Service::PlanningVerificationService m_service;
    RoboSDP::Planning::Dto::PlanningWorkspaceStateDto m_state;
    bool m_has_unsaved_changes = false;

    QPushButton* m_build_scene_button = nullptr;
    QPushButton* m_run_verification_button = nullptr;
    QPushButton* m_save_button = nullptr;
    QPushButton* m_load_button = nullptr;
    QLabel* m_operation_label = nullptr;

    QLineEdit* m_kinematic_ref_edit = nullptr;
    QLineEdit* m_dynamic_ref_edit = nullptr;
    QLineEdit* m_selection_ref_edit = nullptr;
    QLineEdit* m_service_endpoint_edit = nullptr;
    QLineEdit* m_planner_id_edit = nullptr;
    QDoubleSpinBox* m_allowed_time_spin = nullptr;
    QDoubleSpinBox* m_target_cycle_time_spin = nullptr;

    QTableWidget* m_joint_table = nullptr;
    QTableWidget* m_collision_table = nullptr;
    QTableWidget* m_self_collision_table = nullptr;
    QPlainTextEdit* m_result_summary_edit = nullptr;
};

} // namespace RoboSDP::Planning::Ui
