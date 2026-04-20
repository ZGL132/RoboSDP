#pragma once

#include "core/infrastructure/ProjectSaveCoordinator.h"
#include "core/logging/ConsoleLogger.h"
#include "core/repository/LocalJsonRepository.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/selection/persistence/SelectionJsonStorage.h"
#include "modules/selection/service/DriveTrainMatchingService.h"

#include <QString>
#include <QWidget>

class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QTableWidget;

namespace RoboSDP::Selection::Ui
{

/**
 * @brief Selection 最小页面骨架。
 *
 * 页面只负责当前阶段的三件事：
 * 1. 指定项目目录与样例目录；
 * 2. 触发联合驱动链最小选型；
 * 3. 展示每关节推荐链路与推荐理由，并支持保存/加载。
 */
class SelectionWidget : public QWidget, public RoboSDP::Infrastructure::IProjectSaveParticipant
{
    Q_OBJECT

public:
    explicit SelectionWidget(QWidget* parent = nullptr);

    /// @brief 返回全局保存日志使用的模块名称。
    QString ModuleName() const override;

    /// @brief 保存当前 Selection 草稿，供模块按钮和全局保存共用。
    RoboSDP::Infrastructure::ProjectSaveItemResult SaveCurrentDraft() override;

signals:
    void LogMessageGenerated(const QString& message);

private:
    void BuildUi();
    void SetupResultTableColumns();
    void RenderState();
    void SetOperationMessage(const QString& message, bool success);

    void OnBrowseProjectRootClicked();
    void OnBrowseCatalogRootClicked();
    void OnRunSelectionClicked();
    void OnSaveDraftClicked();
    void OnLoadClicked();

private:
    RoboSDP::Logging::ConsoleLogger m_logger;
    RoboSDP::Repository::LocalJsonRepository m_repository;
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage m_dynamic_storage;
    RoboSDP::Selection::Persistence::SelectionJsonStorage m_selection_storage;
    RoboSDP::Selection::Service::DriveTrainMatchingService m_service;
    RoboSDP::Selection::Dto::SelectionWorkspaceStateDto m_state;

    QLineEdit* m_project_root_edit = nullptr;
    QLineEdit* m_catalog_root_edit = nullptr;
    QPushButton* m_browse_project_button = nullptr;
    QPushButton* m_browse_catalog_button = nullptr;
    QPushButton* m_run_button = nullptr;
    QPushButton* m_save_button = nullptr;
    QPushButton* m_load_button = nullptr;
    QLabel* m_operation_label = nullptr;
    QTableWidget* m_result_table = nullptr;
    QPlainTextEdit* m_summary_edit = nullptr;
};

} // namespace RoboSDP::Selection::Ui
