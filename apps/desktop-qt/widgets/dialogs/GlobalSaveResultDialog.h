#pragma once

#include "core/infrastructure/ProjectSaveCoordinator.h"

#include <QDialog>

class QPushButton;
class QTableWidget;

namespace RoboSDP::Desktop::Dialogs
{

/**
 * @brief 全局保存结果对话框。
 *
 * 该对话框只负责展示 ProjectSaveCoordinator 汇总后的保存结果，
 * 不触发任何模块保存动作，避免 UI 弹窗反向耦合业务流程。
 */
class GlobalSaveResultDialog final : public QDialog
{
    Q_OBJECT

public:
    explicit GlobalSaveResultDialog(
        const RoboSDP::Infrastructure::ProjectSaveSummary& summary,
        QWidget* parent = nullptr);

signals:
    /// @brief 请求主窗口定位到保存结果对应的业务模块。
    void ModuleNavigationRequested(const QString& moduleName);

private:
    void BuildUi(const RoboSDP::Infrastructure::ProjectSaveSummary& summary);
    void FillResultTable(const RoboSDP::Infrastructure::ProjectSaveSummary& summary);
    QString SelectedModuleName() const;

    QTableWidget* m_resultTable = nullptr;
    QPushButton* m_navigateButton = nullptr;
};

} // namespace RoboSDP::Desktop::Dialogs
