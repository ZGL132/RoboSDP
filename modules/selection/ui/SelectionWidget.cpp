#include "modules/selection/ui/SelectionWidget.h"

#include "core/infrastructure/ProjectManager.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileDialog>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

namespace RoboSDP::Selection::Ui
{

namespace
{

QString FindDefaultCatalogRootPath()
{
    const QString currentRoot = QDir::current().absolutePath();
    const QString applicationRoot = QCoreApplication::applicationDirPath();
    const QString relativeSelectionPath = QStringLiteral("resources/selection");

    const QStringList candidates {
        QDir(currentRoot).absoluteFilePath(relativeSelectionPath),
        QDir(applicationRoot).absoluteFilePath(relativeSelectionPath),
        QDir(applicationRoot).absoluteFilePath(QStringLiteral("../resources/selection")),
        QDir(applicationRoot).absoluteFilePath(QStringLiteral("../../resources/selection")),
        QDir(applicationRoot).absoluteFilePath(QStringLiteral("../../../resources/selection")),
        QDir(applicationRoot).absoluteFilePath(QStringLiteral("../../../../resources/selection"))};

    for (const QString& candidate : candidates)
    {
        if (QDir(candidate).exists())
        {
            return QDir(candidate).absolutePath();
        }
    }

    return QDir(currentRoot).absoluteFilePath(relativeSelectionPath);
}

QTableWidgetItem* EnsureItem(QTableWidget* table, int row, int column)
{
    QTableWidgetItem* item = table->item(row, column);
    if (item == nullptr)
    {
        item = new QTableWidgetItem();
        table->setItem(row, column, item);
    }
    return item;
}

void SetReadOnlyItem(QTableWidgetItem* item)
{
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
}

} // namespace

SelectionWidget::SelectionWidget(QWidget* parent)
    : QWidget(parent)
    , m_dynamic_storage(m_repository)
    , m_selection_storage(m_repository)
    , m_service(m_selection_storage, m_dynamic_storage, &m_logger)
    , m_state(m_service.CreateDefaultState())
{
    BuildUi();
    RenderState();
    ConnectDirtyTracking();
    MarkClean();
}

QString SelectionWidget::ModuleName() const
{
    return QStringLiteral("Selection");
}

bool SelectionWidget::HasUnsavedChanges() const
{
    return m_has_unsaved_changes;
}

RoboSDP::Infrastructure::ProjectSaveItemResult SelectionWidget::SaveCurrentDraft()
{
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto saveResult = m_service.SaveDraft(projectRootPath, m_state);
    SetOperationMessage(saveResult.message, saveResult.IsSuccess());
    if (saveResult.IsSuccess())
    {
        MarkClean();
    }
    return {ModuleName(), saveResult.IsSuccess(), saveResult.message};
}

void SelectionWidget::ConnectDirtyTracking()
{
    for (QLineEdit* editor : findChildren<QLineEdit*>())
    {
        connect(editor, &QLineEdit::textEdited, this, [this]() { MarkDirty(); });
    }
}

void SelectionWidget::MarkDirty()
{
    m_has_unsaved_changes = true;
}

void SelectionWidget::MarkClean()
{
    m_has_unsaved_changes = false;
}

void SelectionWidget::BuildUi()
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(8, 8, 8, 8);
    rootLayout->setSpacing(8);

    auto* catalogLayout = new QHBoxLayout();
    catalogLayout->addWidget(new QLabel(QStringLiteral("样例目录"), this));
    m_catalog_root_edit = new QLineEdit(FindDefaultCatalogRootPath(), this);
    m_browse_catalog_button = new QPushButton(QStringLiteral("选择目录"), this);
    catalogLayout->addWidget(m_catalog_root_edit, 1);
    catalogLayout->addWidget(m_browse_catalog_button);

    m_operation_label = new QLabel(QStringLiteral("就绪：请先通过顶部功能区新建或打开项目，再选择样例目录后执行驱动选型。"), this);
    m_operation_label->setWordWrap(true);

    auto* scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);
    auto* content = new QWidget(scrollArea);
    auto* contentLayout = new QVBoxLayout(content);

    auto* resultGroup = new QGroupBox(QStringLiteral("每关节推荐链路"), content);
    auto* resultLayout = new QVBoxLayout(resultGroup);
    m_result_table = new QTableWidget(resultGroup);
    SetupResultTableColumns();
    resultLayout->addWidget(m_result_table);

    auto* summaryGroup = new QGroupBox(QStringLiteral("推荐摘要"), content);
    auto* summaryLayout = new QVBoxLayout(summaryGroup);
    m_summary_edit = new QPlainTextEdit(summaryGroup);
    m_summary_edit->setReadOnly(true);
    summaryLayout->addWidget(m_summary_edit);

    contentLayout->addWidget(resultGroup);
    contentLayout->addWidget(summaryGroup);
    contentLayout->addStretch();
    scrollArea->setWidget(content);

    rootLayout->addLayout(catalogLayout);
    rootLayout->addWidget(m_operation_label);
    rootLayout->addWidget(scrollArea, 1);

    connect(m_browse_catalog_button, &QPushButton::clicked, this, [this]() { OnBrowseCatalogRootClicked(); });
}

void SelectionWidget::SetupResultTableColumns()
{
    m_result_table->setColumnCount(7);
    m_result_table->setHorizontalHeaderLabels({
        QStringLiteral("joint_id"),
        QStringLiteral("motor"),
        QStringLiteral("reducer"),
        QStringLiteral("ratio"),
        QStringLiteral("efficiency"),
        QStringLiteral("brake"),
        QStringLiteral("reason")});
    m_result_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

void SelectionWidget::RenderState()
{
    const auto& joints = m_state.drive_train_result.joint_selections;
    m_result_table->setRowCount(static_cast<int>(joints.size()));

    QStringList lines;
    lines.push_back(QStringLiteral("整体结果：%1").arg(m_state.drive_train_result.message));
    lines.push_back(QStringLiteral("电机结果数：%1").arg(m_state.motor_results.size()));
    lines.push_back(QStringLiteral("减速器结果数：%1").arg(m_state.reducer_results.size()));

    for (int row = 0; row < static_cast<int>(joints.size()); ++row)
    {
        const auto& joint = joints.at(static_cast<std::size_t>(row));
        EnsureItem(m_result_table, row, 0)->setText(joint.joint_id);
        SetReadOnlyItem(EnsureItem(m_result_table, row, 0));

        if (joint.has_recommendation)
        {
            EnsureItem(m_result_table, row, 1)->setText(joint.recommended.motor_name);
            EnsureItem(m_result_table, row, 2)->setText(joint.recommended.reducer_name);
            EnsureItem(m_result_table, row, 3)->setText(QString::number(joint.recommended.combined_ratio, 'f', 2));
            EnsureItem(m_result_table, row, 4)->setText(QString::number(joint.recommended.total_efficiency, 'f', 3));
            EnsureItem(m_result_table, row, 5)->setText(joint.recommended.brake_check_passed
                                                            ? QStringLiteral("通过")
                                                            : QStringLiteral("未通过"));
            EnsureItem(m_result_table, row, 6)->setText(joint.recommended.recommendation_reason);
        }
        else
        {
            EnsureItem(m_result_table, row, 1)->setText(QStringLiteral("无推荐"));
            EnsureItem(m_result_table, row, 2)->setText(QString());
            EnsureItem(m_result_table, row, 3)->setText(QString());
            EnsureItem(m_result_table, row, 4)->setText(QString());
            EnsureItem(m_result_table, row, 5)->setText(QString());
            EnsureItem(m_result_table, row, 6)->setText(joint.message);
        }

        for (int column = 1; column < m_result_table->columnCount(); ++column)
        {
            SetReadOnlyItem(EnsureItem(m_result_table, row, column));
        }

        lines.push_back(QString());
        lines.push_back(QStringLiteral("[%1] %2").arg(joint.joint_id, joint.message));
        if (joint.has_recommendation)
        {
            lines.push_back(
                QStringLiteral("推荐链路：%1 + %2，效率 %3，理由：%4")
                    .arg(joint.recommended.motor_name)
                    .arg(joint.recommended.reducer_name)
                    .arg(joint.recommended.total_efficiency, 0, 'f', 3)
                    .arg(joint.recommended.recommendation_reason));
        }
    }

    m_summary_edit->setPlainText(lines.join(QLatin1Char('\n')));
}

void SelectionWidget::SetOperationMessage(const QString& message, bool success)
{
    m_operation_label->setText(message);
    m_operation_label->setStyleSheet(success ? QStringLiteral("color: #1b7f3b;")
                                             : QStringLiteral("color: #b42318;"));
}

void SelectionWidget::OnBrowseCatalogRootClicked()
{
    const QString path = QFileDialog::getExistingDirectory(
        this,
        QStringLiteral("选择样例目录"),
        m_catalog_root_edit->text().trimmed());
    if (!path.isEmpty())
    {
        m_catalog_root_edit->setText(QDir::toNativeSeparators(path));
        MarkDirty();
    }
}

void SelectionWidget::OnRunSelectionClicked()
{
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto runResult = m_service.RunSelection(
        projectRootPath,
        m_catalog_root_edit->text().trimmed());
    if (runResult.IsSuccess())
    {
        m_state = runResult.state;
        RenderState();
        m_catalog_root_edit->setText(QDir::toNativeSeparators(runResult.catalog_directory_path));
        MarkDirty();
    }

    SetOperationMessage(runResult.message, runResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Selection] %1").arg(runResult.message));
}

void SelectionWidget::OnSaveDraftClicked()
{
    const auto saveResult = SaveCurrentDraft();
    emit LogMessageGenerated(QStringLiteral("[Selection] %1").arg(saveResult.message));
}

void SelectionWidget::OnLoadClicked()
{
    const QString projectRootPath =
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath();
    const auto loadResult = m_service.LoadDraft(projectRootPath);
    if (loadResult.IsSuccess())
    {
        m_state = loadResult.state;
        RenderState();
        MarkClean();
    }

    SetOperationMessage(loadResult.message, loadResult.IsSuccess());
    emit LogMessageGenerated(QStringLiteral("[Selection] %1").arg(loadResult.message));
}

} // namespace RoboSDP::Selection::Ui
