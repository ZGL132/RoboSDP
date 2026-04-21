#include "apps/desktop-qt/widgets/dialogs/GlobalSaveResultDialog.h"

#include <QAbstractItemView>
#include <QColor>
#include <QDialogButtonBox>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

namespace RoboSDP::Desktop::Dialogs
{

GlobalSaveResultDialog::GlobalSaveResultDialog(
    const RoboSDP::Infrastructure::ProjectSaveSummary& summary,
    QWidget* parent)
    : QDialog(parent)
{
    BuildUi(summary);
}

void GlobalSaveResultDialog::BuildUi(const RoboSDP::Infrastructure::ProjectSaveSummary& summary)
{
    setWindowTitle(QStringLiteral("全局保存结果"));
    resize(720, 420);

    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(12, 12, 12, 12);
    rootLayout->setSpacing(10);

    auto* summaryLabel = new QLabel(
        QStringLiteral("%1\n项目目录：%2")
            .arg(summary.message, summary.project_root_path),
        this);
    summaryLabel->setWordWrap(true);
    summaryLabel->setStyleSheet(
        summary.success
            ? QStringLiteral("color:#1b7f3b;font-weight:600;")
            : QStringLiteral("color:#b54708;font-weight:600;"));
    rootLayout->addWidget(summaryLabel);

    m_resultTable = new QTableWidget(this);
    m_resultTable->setColumnCount(3);
    m_resultTable->setHorizontalHeaderLabels(
        {QStringLiteral("模块"), QStringLiteral("状态"), QStringLiteral("消息")});
    m_resultTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_resultTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_resultTable->setAlternatingRowColors(true);
    m_resultTable->verticalHeader()->setVisible(false);
    m_resultTable->horizontalHeader()->setStretchLastSection(true);
    m_resultTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_resultTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    FillResultTable(summary);
    connect(m_resultTable, &QTableWidget::itemSelectionChanged, this, [this]() {
        if (m_navigateButton != nullptr)
        {
            m_navigateButton->setEnabled(!SelectedModuleName().trimmed().isEmpty());
        }
    });
    rootLayout->addWidget(m_resultTable, 1);

    auto* buttonBox = new QDialogButtonBox(QDialogButtonBox::Close, this);
    m_navigateButton = buttonBox->addButton(QStringLiteral("定位模块"), QDialogButtonBox::ActionRole);
    m_navigateButton->setEnabled(false);
    connect(m_navigateButton, &QPushButton::clicked, this, [this]() {
        const QString moduleName = SelectedModuleName();
        if (!moduleName.trimmed().isEmpty())
        {
            // 中文说明：对话框只发出模块名，不直接依赖 MainWindow 的项目树结构。
            emit ModuleNavigationRequested(moduleName);
        }
    });
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    rootLayout->addWidget(buttonBox);
}

void GlobalSaveResultDialog::FillResultTable(
    const RoboSDP::Infrastructure::ProjectSaveSummary& summary)
{
    if (m_resultTable == nullptr)
    {
        return;
    }

    m_resultTable->setRowCount(summary.item_results.size());
    for (int row = 0; row < summary.item_results.size(); ++row)
    {
        const auto& result = summary.item_results.at(row);
        auto* moduleItem = new QTableWidgetItem(result.module_name);
        auto* statusItem = new QTableWidgetItem(
            result.dependency_refresh_required
                ? QStringLiteral("需刷新")
                : (result.skipped
                ? QStringLiteral("跳过")
                : (result.success ? QStringLiteral("成功") : QStringLiteral("失败"))));
        auto* messageItem = new QTableWidgetItem(result.message);

        // 中文说明：颜色只作为结果提示，不改变保存汇总数据本身。
        statusItem->setForeground(
            result.dependency_refresh_required
                ? QColor(QStringLiteral("#b54708"))
                : (result.skipped
                ? QColor(QStringLiteral("#6b7280"))
                : (result.success ? QColor(QStringLiteral("#1b7f3b")) : QColor(QStringLiteral("#b42318")))));

        m_resultTable->setItem(row, 0, moduleItem);
        m_resultTable->setItem(row, 1, statusItem);
        m_resultTable->setItem(row, 2, messageItem);
    }
}

QString GlobalSaveResultDialog::SelectedModuleName() const
{
    if (m_resultTable == nullptr || m_resultTable->currentRow() < 0)
    {
        return QString();
    }

    const QTableWidgetItem* moduleItem = m_resultTable->item(m_resultTable->currentRow(), 0);
    return moduleItem != nullptr ? moduleItem->text().trimmed() : QString();
}

} // namespace RoboSDP::Desktop::Dialogs
