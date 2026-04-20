#include "modules/scheme/ui/SchemeWidget.h"

#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

namespace RoboSDP::Scheme::Ui
{

SchemeWidget::SchemeWidget(QWidget* parent)
    : QWidget(parent)
    , m_requirement_storage(m_repository)
    , m_topology_storage(m_repository)
    , m_kinematic_storage(m_repository)
    , m_dynamic_storage(m_repository)
    , m_selection_storage(m_repository)
    , m_planning_storage(m_repository)
    , m_scheme_storage(m_repository)
    , m_snapshot_service(
          m_scheme_storage,
          m_requirement_storage,
          m_topology_storage,
          m_kinematic_storage,
          m_dynamic_storage,
          m_selection_storage,
          m_planning_storage,
          &m_logger)
    , m_export_service(m_scheme_storage, &m_logger)
    , m_snapshot(m_snapshot_service.CreateDefaultSnapshot())
{
    BuildUi();
    RenderSnapshotSummary();
}

void SchemeWidget::BuildUi()
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(8, 8, 8, 8);
    rootLayout->setSpacing(8);

    auto* projectPathLayout = new QHBoxLayout();
    projectPathLayout->addWidget(new QLabel(QStringLiteral("项目目录"), this));
    m_project_root_edit = new QLineEdit(QDir::currentPath(), this);
    m_browse_button = new QPushButton(QStringLiteral("选择目录"), this);
    projectPathLayout->addWidget(m_project_root_edit, 1);
    projectPathLayout->addWidget(m_browse_button);

    auto* actionLayout = new QHBoxLayout();
    m_generate_snapshot_button = new QPushButton(QStringLiteral("生成 Snapshot"), this);
    m_regenerate_save_snapshot_button = new QPushButton(QStringLiteral("重新生成并保存"), this);
    m_load_snapshot_button = new QPushButton(QStringLiteral("加载 Snapshot"), this);
    m_export_json_button = new QPushButton(QStringLiteral("导出 JSON"), this);
    actionLayout->addWidget(m_generate_snapshot_button);
    actionLayout->addWidget(m_regenerate_save_snapshot_button);
    actionLayout->addWidget(m_load_snapshot_button);
    actionLayout->addWidget(m_export_json_button);
    actionLayout->addStretch();

    m_operation_label = new QLabel(
        QStringLiteral("就绪：请选择项目目录，然后生成或加载 SchemeSnapshot。"),
        this);
    m_operation_label->setWordWrap(true);

    auto* scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);

    auto* contentWidget = new QWidget(scrollArea);
    auto* contentLayout = new QVBoxLayout(contentWidget);

    auto* snapshotGroup = new QGroupBox(QStringLiteral("快照状态"), contentWidget);
    auto* snapshotFormLayout = new QFormLayout(snapshotGroup);
    m_scheme_id_edit = new QLineEdit(snapshotGroup);
    m_scheme_id_edit->setReadOnly(true);
    m_snapshot_file_edit = new QLineEdit(snapshotGroup);
    m_snapshot_file_edit->setReadOnly(true);
    m_snapshot_status_edit = new QLineEdit(snapshotGroup);
    m_snapshot_status_edit->setReadOnly(true);
    m_available_module_count_edit = new QLineEdit(snapshotGroup);
    m_available_module_count_edit->setReadOnly(true);
    m_snapshot_updated_at_edit = new QLineEdit(snapshotGroup);
    m_snapshot_updated_at_edit->setReadOnly(true);
    snapshotFormLayout->addRow(QStringLiteral("Scheme ID"), m_scheme_id_edit);
    snapshotFormLayout->addRow(QStringLiteral("Snapshot 文件"), m_snapshot_file_edit);
    snapshotFormLayout->addRow(QStringLiteral("状态"), m_snapshot_status_edit);
    snapshotFormLayout->addRow(QStringLiteral("已聚合模块数"), m_available_module_count_edit);
    snapshotFormLayout->addRow(QStringLiteral("最近更新时间"), m_snapshot_updated_at_edit);

    auto* exportGroup = new QGroupBox(QStringLiteral("导出结果"), contentWidget);
    auto* exportFormLayout = new QFormLayout(exportGroup);
    m_export_file_edit = new QLineEdit(exportGroup);
    m_export_file_edit->setReadOnly(true);
    m_export_format_edit = new QLineEdit(exportGroup);
    m_export_format_edit->setReadOnly(true);
    m_export_relative_path_edit = new QLineEdit(exportGroup);
    m_export_relative_path_edit->setReadOnly(true);
    m_exported_at_edit = new QLineEdit(exportGroup);
    m_exported_at_edit->setReadOnly(true);
    exportFormLayout->addRow(QStringLiteral("Export 文件"), m_export_file_edit);
    exportFormLayout->addRow(QStringLiteral("导出格式"), m_export_format_edit);
    exportFormLayout->addRow(QStringLiteral("导出相对路径"), m_export_relative_path_edit);
    exportFormLayout->addRow(QStringLiteral("最近导出时间"), m_exported_at_edit);

    auto* summaryGroup = new QGroupBox(QStringLiteral("Scheme 摘要"), contentWidget);
    auto* summaryLayout = new QVBoxLayout(summaryGroup);
    m_summary_edit = new QPlainTextEdit(summaryGroup);
    m_summary_edit->setReadOnly(true);
    summaryLayout->addWidget(m_summary_edit);

    contentLayout->addWidget(snapshotGroup);
    contentLayout->addWidget(exportGroup);
    contentLayout->addWidget(summaryGroup);
    contentLayout->addStretch();
    scrollArea->setWidget(contentWidget);

    rootLayout->addLayout(projectPathLayout);
    rootLayout->addLayout(actionLayout);
    rootLayout->addWidget(m_operation_label);
    rootLayout->addWidget(scrollArea, 1);

    connect(m_browse_button, &QPushButton::clicked, this, [this]() { OnBrowseProjectRootClicked(); });
    connect(
        m_generate_snapshot_button,
        &QPushButton::clicked,
        this,
        [this]() { OnGenerateSnapshotClicked(); });
    connect(
        m_regenerate_save_snapshot_button,
        &QPushButton::clicked,
        this,
        [this]() { OnRegenerateAndSaveSnapshotClicked(); });
    connect(
        m_load_snapshot_button,
        &QPushButton::clicked,
        this,
        [this]() { OnLoadSnapshotClicked(); });
    connect(
        m_export_json_button,
        &QPushButton::clicked,
        this,
        [this]() { OnExportJsonClicked(); });
}

void SchemeWidget::RenderSnapshotSummary()
{
    m_scheme_id_edit->setText(m_has_snapshot ? m_snapshot.meta.scheme_id : QString());
    m_snapshot_file_edit->setText(m_last_snapshot_file_path);
    m_snapshot_status_edit->setText(m_has_snapshot ? m_snapshot.meta.status : QString());
    m_available_module_count_edit->setText(
        m_has_snapshot ? QString::number(m_snapshot.aggregate.available_module_count) : QString());
    m_snapshot_updated_at_edit->setText(m_has_snapshot ? m_snapshot.meta.updated_at : QString());
    m_export_file_edit->setText(m_last_export_file_path);
    m_export_format_edit->setText(
        m_has_snapshot ? m_snapshot.aggregate.export_meta.default_export_format : QString());
    m_export_relative_path_edit->setText(
        m_has_snapshot ? m_snapshot.aggregate.export_meta.last_export_relative_path : QString());
    m_exported_at_edit->setText(
        m_has_snapshot ? m_snapshot.aggregate.export_meta.last_exported_at : QString());

    if (!m_has_snapshot)
    {
        m_summary_edit->setPlainText(
            QStringLiteral(
                "当前尚未生成或加载 SchemeSnapshot。\n"
                "页面仅展示 Scheme 聚合摘要，不展示上游完整原始对象。"));
        return;
    }

    const auto& aggregate = m_snapshot.aggregate;
    QStringList lines;
    lines.push_back(QStringLiteral("Scheme ID：%1").arg(m_snapshot.meta.scheme_id));
    lines.push_back(QStringLiteral("名称：%1").arg(m_snapshot.meta.name));
    lines.push_back(QStringLiteral("状态：%1").arg(m_snapshot.meta.status));
    lines.push_back(QStringLiteral("项目目录：%1").arg(m_snapshot.meta.source_project_root));
    lines.push_back(QStringLiteral("创建时间：%1").arg(m_snapshot.meta.created_at));
    lines.push_back(QStringLiteral("更新时间：%1").arg(m_snapshot.meta.updated_at));
    lines.push_back(QString());
    lines.push_back(QStringLiteral("已聚合模块数：%1").arg(aggregate.available_module_count));
    lines.push_back(QStringLiteral("完整性摘要：%1").arg(aggregate.completeness_summary));
    lines.push_back(QString());
    lines.push_back(QStringLiteral("[Requirement] %1").arg(aggregate.requirement_summary.summary_text));
    lines.push_back(QStringLiteral("[Topology] %1").arg(aggregate.topology_summary.summary_text));
    lines.push_back(QStringLiteral("[Kinematics] %1").arg(aggregate.kinematics_summary.summary_text));
    lines.push_back(QStringLiteral("[Dynamics] %1").arg(aggregate.dynamics_summary.summary_text));
    lines.push_back(QStringLiteral("[Selection] %1").arg(aggregate.selection_summary.summary_text));
    lines.push_back(QStringLiteral("[Planning] %1").arg(aggregate.planning_summary.summary_text));
    lines.push_back(QString());
    lines.push_back(QStringLiteral("默认导出格式：%1").arg(aggregate.export_meta.default_export_format));
    lines.push_back(
        QStringLiteral("默认导出相对路径：%1").arg(aggregate.export_meta.default_export_relative_path));
    lines.push_back(
        QStringLiteral("最近导出相对路径：%1").arg(aggregate.export_meta.last_export_relative_path));
    lines.push_back(QStringLiteral("最近导出时间：%1").arg(aggregate.export_meta.last_exported_at));

    m_summary_edit->setPlainText(lines.join(QLatin1Char('\n')));
}

void SchemeWidget::SetOperationMessage(const QString& message, bool success)
{
    m_operation_label->setText(message);
    m_operation_label->setStyleSheet(
        success ? QStringLiteral("color: #1b7f3b;") : QStringLiteral("color: #b42318;"));
}

void SchemeWidget::OnGenerateSnapshotClicked()
{
    const auto buildResult = m_snapshot_service.BuildSnapshot(m_project_root_edit->text().trimmed());
    if (buildResult.IsSuccess())
    {
        m_snapshot = buildResult.snapshot;
        m_last_snapshot_file_path = buildResult.snapshot_file_path;
        m_has_snapshot = true;
        RenderSnapshotSummary();
    }

    SetOperationMessage(buildResult.message, buildResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Scheme] %1").arg(buildResult.message));
}

void SchemeWidget::OnRegenerateAndSaveSnapshotClicked()
{
    const QString projectRootPath = m_project_root_edit->text().trimmed();
    const auto buildResult = m_snapshot_service.BuildSnapshot(projectRootPath);
    if (!buildResult.IsSuccess())
    {
        SetOperationMessage(buildResult.message, false);
        emit LogMessageGenerated(QStringLiteral("[Scheme] %1").arg(buildResult.message));
        return;
    }

    const auto saveResult = m_snapshot_service.SaveSnapshot(projectRootPath, buildResult.snapshot);
    if (saveResult.IsSuccess())
    {
        m_snapshot = buildResult.snapshot;
        m_snapshot.meta.updated_at = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
        m_last_snapshot_file_path = saveResult.snapshot_file_path;
        m_has_snapshot = true;
        RenderSnapshotSummary();
    }

    const QString message = saveResult.IsSuccess()
        ? QStringLiteral("SchemeSnapshot 已重新生成并保存。")
        : saveResult.message;
    SetOperationMessage(message, saveResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Scheme] %1").arg(message));
}

void SchemeWidget::OnLoadSnapshotClicked()
{
    const auto loadResult = m_snapshot_service.LoadSnapshot(m_project_root_edit->text().trimmed());
    if (loadResult.IsSuccess())
    {
        m_snapshot = loadResult.snapshot;
        m_last_snapshot_file_path = loadResult.snapshot_file_path;
        m_has_snapshot = true;
        RenderSnapshotSummary();
    }

    SetOperationMessage(loadResult.message, loadResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Scheme] %1").arg(loadResult.message));
}

void SchemeWidget::OnExportJsonClicked()
{
    if (!m_has_snapshot)
    {
        const QString message = QStringLiteral("请先生成或加载 SchemeSnapshot，再执行 JSON 导出。");
        SetOperationMessage(message, false);
        emit LogMessageGenerated(QStringLiteral("[Scheme] %1").arg(message));
        return;
    }

    const auto exportResult =
        m_export_service.ExportAsJson(m_project_root_edit->text().trimmed(), m_snapshot);
    if (exportResult.IsSuccess())
    {
        m_last_export_file_path = exportResult.export_file_path;
        m_snapshot.aggregate.export_meta.last_export_relative_path =
            QStringLiteral("exports/scheme-export.json");
        m_snapshot.aggregate.export_meta.last_exported_at = exportResult.export_dto.generated_at;
        m_snapshot.aggregate.export_meta.default_export_format = exportResult.export_dto.export_format;
        RenderSnapshotSummary();
    }

    SetOperationMessage(exportResult.message, exportResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Scheme] %1").arg(exportResult.message));
}

void SchemeWidget::OnBrowseProjectRootClicked()
{
    const QString selectedDirectory = QFileDialog::getExistingDirectory(
        this,
        QStringLiteral("选择 Scheme 项目目录"),
        m_project_root_edit->text().trimmed());

    if (!selectedDirectory.isEmpty())
    {
        m_project_root_edit->setText(QDir::toNativeSeparators(selectedDirectory));
    }
}

} // namespace RoboSDP::Scheme::Ui
