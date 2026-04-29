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
    /// @brief 请求设置中央三维视图中的地面网格显示状态。
    void signalSetGroundGridVisible(bool visible);

    /// @brief 请求设置中央三维视图中的屏幕角落方向坐标轴显示状态。
    void signalSetCornerAxesVisible(bool visible);

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

    /// @brief 请求设置中央三维视图中的骨架显示状态。
    void signalSetSkeletonVisible(bool visible);

    /// @brief 请求设置中央三维视图中的 visual mesh 显示状态。
    void signalSetVisualMeshVisible(bool visible);

    /// @brief 请求设置中央三维视图中的 collision mesh 显示状态。
    void signalSetCollisionMeshVisible(bool visible);

    /// @brief 请求设置中央三维视图中的关节轴显示状态。
    void signalSetJointAxesVisible(bool visible);

    /// @brief 请求设置中央三维视图中的坐标系显示状态。
    void signalSetAxesVisible(bool visible);

    /// @brief 请求设置中央三维视图中的 Link 标签显示状态。
    void signalSetLinkLabelsVisible(bool visible);

    /// @brief 请求设置中央三维视图中的 Joint 标签显示状态。
    void signalSetJointLabelsVisible(bool visible);

    /// @brief 请求重置中央三维视图相机。
    void signalResetCameraRequested();

    /// @brief 请求切换中央三维视图到正视图。
    void signalCameraFrontViewRequested();

    /// @brief 请求切换中央三维视图到侧视图。
    void signalCameraSideViewRequested();

    /// @brief 请求切换中央三维视图到俯视图。
    void signalCameraTopViewRequested();

    /// @brief 请求切换中央三维视图到等轴测视图。
    void signalCameraIsometricViewRequested();

    /// @brief 请求设置左侧项目树显示状态。
    void signalSetProjectTreeVisible(bool visible);

    /// @brief 请求设置右侧属性面板显示状态。
    void signalSetPropertyPanelVisible(bool visible);

    /// @brief 请求设置底部输出信息面板显示状态。
    void signalSetOutputPanelVisible(bool visible);

private:
    void BuildUi();
    QWidget* CreateFileTab();
    QWidget* CreateModelingTab();
    QWidget* CreateDynamicsTab();
    QWidget* CreateDriveSelectionTab();
    QWidget* CreatePlanningAnalysisTab();
    QWidget* CreateResultExportTab();
    QWidget* CreateViewTab();

    QToolButton* CreateActionButton(const QString& text, const QString& tooltip);
    QToolButton* CreateToggleButton(
        const QString& text,
        const QString& tooltip,
        bool checked,
        void (RibbonBarWidget::*toggleSignal)(bool));
    QToolButton* CreateNewProjectButton();
    QToolButton* CreateOpenProjectButton();
    QToolButton* CreateGlobalSaveButton();
    QToolButton* CreateResetCameraButton();
    QToolButton* CreateCameraPresetButton(
        const QString& text,
        const QString& tooltip,
        void (RibbonBarWidget::*cameraSignal)());
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
