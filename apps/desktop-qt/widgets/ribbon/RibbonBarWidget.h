#pragma once

#include <QWidget>

class QTabWidget;
class QToolButton;

namespace RoboSDP::Desktop::Ribbon
{

/**
 * @brief 顶部功能区业务目标枚举。
 *
 * 该枚举只表达“用户希望进入哪个业务域”，不直接承载具体算法或持久化流程，
 * 便于 MainWindow 继续负责页面编排，Service 层继续负责业务执行。
 */
enum class RibbonNavigationTarget
{
    Requirement,
    Topology,
    Kinematics,
    Dynamics,
    Selection,
    Planning,
    Scheme
};

/**
 * @brief RoboSDP 顶部 Ribbon / Toolbar 控件。
 *
 * 控件职责：
 * 1. 按文档 16.1.2 的业务链条组织顶部页签；
 * 2. 只暴露轻量导航/动作信号，不在顶部 UI 内直接执行复杂业务逻辑；
 * 3. 对暂未接入统一项目服务的按钮使用禁用态和中文 tooltip 标明边界。
 */
class RibbonBarWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RibbonBarWidget(QWidget* parent = nullptr);

signals:
    /// @brief 请求主窗口切换到指定业务模块页面。
    void NavigationRequested(RoboSDP::Desktop::Ribbon::RibbonNavigationTarget target);

    /// @brief 请求执行 URDF 导入；具体文件选择与导入流程由 Kinematics 页面负责。
    void ImportUrdfRequested();

    /// @brief 请求执行逆动力学分析；具体校验、计算与结果刷新由 Dynamics 页面负责。
    void RunDynamicsAnalysisRequested();

    /// @brief 请求记录一个暂未接线或轻量动作的顶部功能区事件。
    void RibbonActionRequested(const QString& actionName);

private:
    void BuildUi();
    QWidget* CreateFileTab();
    QWidget* CreateModelingTab();
    QWidget* CreateDynamicsTab();
    QWidget* CreateDriveSelectionTab();
    QWidget* CreatePlanningAnalysisTab();
    QWidget* CreateResultExportTab();

    QToolButton* CreateActionButton(const QString& text, const QString& tooltip);
    QToolButton* CreateNavigationButton(
        const QString& text,
        const QString& tooltip,
        RoboSDP::Desktop::Ribbon::RibbonNavigationTarget target);
    QToolButton* CreateImportUrdfButton(const QString& text, const QString& tooltip);
    QToolButton* CreateRunDynamicsAnalysisButton(const QString& text, const QString& tooltip);
    QToolButton* CreateDisabledButton(const QString& text, const QString& reason);
    QWidget* CreateButtonGroup(const QString& title, const QList<QToolButton*>& buttons);

private:
    QTabWidget* m_tabs = nullptr;
};

} // namespace RoboSDP::Desktop::Ribbon
