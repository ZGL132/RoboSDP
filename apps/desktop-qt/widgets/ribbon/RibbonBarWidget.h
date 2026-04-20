#pragma once

#include <QList>
#include <QWidget>

class QTabWidget;
class QToolButton;

namespace RoboSDP::Desktop::Ribbon
{

/**
 * @brief RoboSDP 顶部 Ribbon / Toolbar 控件。
 *
 * 控件职责：
 * 1. 按文档 16.1.2 的业务链条组织顶部页签；
 * 2. 只暴露轻量导航信号，不在顶部 UI 内直接执行复杂业务逻辑；
 * 3. 对暂未接入统一项目服务的按钮使用禁用态和中文 tooltip 标明边界。
 */
class RibbonBarWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RibbonBarWidget(QWidget* parent = nullptr);

signals:
    /// @brief 请求主窗口启动项目级新建流程。
    void signalCreateNewProject();

    /// @brief 请求主窗口启动项目级打开流程。
    void signalOpenProject();

    /// @brief 请求主窗口执行全局保存。
    void signalGlobalSaveRequested();

    /// @brief 请求主窗口切换到 Requirement 需求定义页面。
    void signalNavigateToRequirement();

    /// @brief 请求主窗口切换到 Topology 构型设计页面。
    void signalNavigateToTopology();

    /// @brief 请求主窗口切换到 Kinematics 运动学建模页面。
    void signalNavigateToKinematics();

    /// @brief 请求主窗口切换到 Dynamics 动力学分析页面。
    void signalNavigateToDynamics();

    /// @brief 请求主窗口切换到 Selection 驱动链选型页面，当前阶段仅预留。
    void signalNavigateToSelection();

    /// @brief 请求主窗口切换到 Planning 规划与分析页面。
    void signalNavigateToPlanning();

    /// @brief 请求主窗口切换到 Scheme 方案快照与导出页面。
    void signalNavigateToScheme();

private:
    void BuildUi();
    QWidget* CreateFileTab();
    QWidget* CreateModelingTab();
    QWidget* CreateDynamicsTab();
    QWidget* CreateDriveSelectionTab();
    QWidget* CreatePlanningAnalysisTab();
    QWidget* CreateResultExportTab();

    QToolButton* CreateActionButton(const QString& text, const QString& tooltip);
    QToolButton* CreateNewProjectButton();
    QToolButton* CreateOpenProjectButton();
    QToolButton* CreateGlobalSaveButton();
    QToolButton* CreateNavigationButton(
        const QString& text,
        const QString& tooltip,
        void (RibbonBarWidget::*navigationSignal)());
    QToolButton* CreateDisabledButton(const QString& text, const QString& reason);
    QWidget* CreateButtonGroup(const QString& title, const QList<QToolButton*>& buttons);

private:
    QTabWidget* m_tabs = nullptr;
};

} // namespace RoboSDP::Desktop::Ribbon
