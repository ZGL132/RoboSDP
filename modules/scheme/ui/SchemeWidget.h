#pragma once

#include "core/infrastructure/ProjectSaveCoordinator.h"
#include "core/logging/ConsoleLogger.h"
#include "core/repository/LocalJsonRepository.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/planning/persistence/PlanningJsonStorage.h"
#include "modules/requirement/persistence/RequirementJsonStorage.h"
#include "modules/scheme/persistence/SchemeJsonStorage.h"
#include "modules/scheme/service/SchemeExportService.h"
#include "modules/scheme/service/SchemeSnapshotService.h"
#include "modules/selection/persistence/SelectionJsonStorage.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

#include <QString>
#include <QWidget>

class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;

namespace RoboSDP::Scheme::Ui
{

/**
 * @brief Scheme 最小页面骨架。
 *
 * 本页面只负责：
 * 1. 触发 SchemeSnapshot 生成、加载和 JSON 导出；
 * 2. 展示 Scheme 聚合摘要；
 * 3. 通过日志信号把最小操作结果回传给主窗口。
 * 不承载复杂方案对比、图表和上游原始对象浏览能力。
 */
class SchemeWidget : public QWidget, public RoboSDP::Infrastructure::IProjectSaveParticipant
{
    Q_OBJECT

public:
    explicit SchemeWidget(QWidget* parent = nullptr);

    /// @brief 返回全局保存日志使用的模块名称。
    QString ModuleName() const override;

    /// @brief 保存当前 Scheme 快照，供模块按钮和全局保存共用。
    RoboSDP::Infrastructure::ProjectSaveItemResult SaveCurrentDraft() override;

    /// @brief 返回当前 Scheme 快照是否存在未保存变更。
    bool HasUnsavedChanges() const override;

signals:
    /// 向主窗口输出一条简短日志消息。
    void LogMessageGenerated(const QString& message);

private:
    /// 构建最小界面控件，不下沉业务逻辑。
    void BuildUi();

    /// 把当前快照摘要渲染到只读文本区。
    void RenderSnapshotSummary();

    /// 统一设置操作提示文本和颜色。
    void SetOperationMessage(const QString& message, bool success);
    void ConnectDirtyTracking();
    void MarkDirty();
    void MarkClean();

    /// 生成一个跨模块聚合后的 SchemeSnapshot。
    void OnGenerateSnapshotClicked();

    /// 重新生成快照并立即保存，减少用户手动分两步操作。
    void OnRegenerateAndSaveSnapshotClicked();

    /// 从项目目录加载已保存的 SchemeSnapshot。
    void OnLoadSnapshotClicked();

    /// 导出当前快照为 JSON 交付摘要。
    void OnExportJsonClicked();

private:
    RoboSDP::Logging::ConsoleLogger m_logger;
    RoboSDP::Repository::LocalJsonRepository m_repository;
    RoboSDP::Requirement::Persistence::RequirementJsonStorage m_requirement_storage;
    RoboSDP::Topology::Persistence::TopologyJsonStorage m_topology_storage;
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage m_kinematic_storage;
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage m_dynamic_storage;
    RoboSDP::Selection::Persistence::SelectionJsonStorage m_selection_storage;
    RoboSDP::Planning::Persistence::PlanningJsonStorage m_planning_storage;
    RoboSDP::Scheme::Persistence::SchemeJsonStorage m_scheme_storage;
    RoboSDP::Scheme::Service::SchemeSnapshotService m_snapshot_service;
    RoboSDP::Scheme::Service::SchemeExportService m_export_service;

    RoboSDP::Scheme::Dto::SchemeSnapshotDto m_snapshot;
    QString m_last_snapshot_file_path;
    QString m_last_export_file_path;
    bool m_has_snapshot = false;
    bool m_has_unsaved_changes = false;

    QPushButton* m_generate_snapshot_button = nullptr;
    QPushButton* m_regenerate_save_snapshot_button = nullptr;
    QPushButton* m_load_snapshot_button = nullptr;
    QPushButton* m_export_json_button = nullptr;
    QLabel* m_operation_label = nullptr;
    QLineEdit* m_scheme_id_edit = nullptr;
    QLineEdit* m_snapshot_file_edit = nullptr;
    QLineEdit* m_snapshot_status_edit = nullptr;
    QLineEdit* m_available_module_count_edit = nullptr;
    QLineEdit* m_snapshot_updated_at_edit = nullptr;
    QLineEdit* m_export_file_edit = nullptr;
    QLineEdit* m_export_format_edit = nullptr;
    QLineEdit* m_export_relative_path_edit = nullptr;
    QLineEdit* m_exported_at_edit = nullptr;
    QPlainTextEdit* m_summary_edit = nullptr;
};

} // namespace RoboSDP::Scheme::Ui
