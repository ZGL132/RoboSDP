#pragma once

#include <QList>
#include <QMap>
#include <QString>
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
 * 2. 根据左侧项目树选中的节点，切换到对应的工具页签（构型/运动学/动力学）；
 * 3. 只暴露具体的工具执行信号，不在顶部 UI 内直接执行复杂业务逻辑；
 * 4. 接收来自 MainWindow 的按钮启用/禁用状态更新，保持与业务页面状态同步。
 */
class RibbonBarWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RibbonBarWidget(QWidget* parent = nullptr);

    /**
     * @brief 根据模块名切换到对应的 Ribbon 工具页签。
     * @param moduleName  模块名，例如 "File"、"Topology"、"Kinematics"、"Dynamics"、"View"
     */
    void SwitchToContextTab(const QString& moduleName);

    // ── 按钮状态更新（由 MainWindow 连接业务页面 StatusChanged 信号后调用） ──

    /// @brief 更新构型工具按钮的启用状态。
    void SetTopologyButtonsEnabled(bool refreshEnabled, bool generateEnabled, bool validateEnabled, bool saveEnabled);

    /// @brief 更新运动学工具按钮的启用状态。
    void SetKinematicsButtonsEnabled(
        bool importUrdfEnabled,
        bool buildFromTopologyEnabled,
        bool promoteDhEnabled,
        bool switchUrdfEnabled,
        bool runFkEnabled,
        bool runIkEnabled,
        bool sampleWorkspaceEnabled,
        bool saveEnabled);

    /// @brief 更新动力学工具按钮的启用状态。
    void SetDynamicsButtonsEnabled(bool buildFromKinematicsEnabled, bool runAnalysisEnabled, bool saveEnabled);

signals:
    // ── 文件操作 ──────────────────────────────────────────────
    /// @brief 请求主窗口启动项目级新建流程。
    void signalCreateNewProject();

    /// @brief 请求主窗口启动项目级打开流程。
    void signalOpenProject();

    /// @brief 请求主窗口执行全局保存。
    void signalGlobalSaveRequested();

    // ── 需求校验工具 ──────────────────────────────────────────
    /// @brief 请求执行需求校验。
    void signalRequirementValidate();
    /// @brief 请求保存需求草稿。
    void signalRequirementSaveDraft();

    // ── 构型工具 ──────────────────────────────────────────────
    /// @brief 请求刷新构型模板列表。
    void signalTopologyRefreshTemplates();
    /// @brief 请求执行构型生成。
    void signalTopologyGenerate();
    /// @brief 请求校验当前构型。
    void signalTopologyValidate();
    /// @brief 请求保存构型草稿。
    void signalTopologySaveDraft();

    // ── 运动学工具 ────────────────────────────────────────────
    /// @brief 请求导入 URDF 文件。
    void signalKinematicsImportUrdf();
    /// @brief 请求从拓扑构型构建运动学。
    void signalKinematicsBuildFromTopology();
    /// @brief 请求提升为 DH 主模型。
    void signalKinematicsPromoteToDhMaster();
    /// @brief 请求切换回 URDF 主模型。
    void signalKinematicsSwitchToUrdfMaster();
    /// @brief 请求执行正运动学求解。
    void signalKinematicsRunFk();
    /// @brief 请求执行逆运动学求解。
    void signalKinematicsRunIk();
    /// @brief 请求采样工作空间。
    void signalKinematicsSampleWorkspace();
    /// @brief 请求保存运动学草稿。
    void signalKinematicsSaveDraft();

    // ── 动力学工具 ────────────────────────────────────────────
    /// @brief 请求从运动学模型构建动力学。
    void signalDynamicsBuildFromKinematics();
    /// @brief 请求执行动力学分析。
    void signalDynamicsRunAnalysis();
    /// @brief 请求保存动力学草稿。
    void signalDynamicsSaveDraft();

    // ── 选型工具 ──────────────────────────────────────────────
    /// @brief 请求执行驱动选型。
    void signalSelectionRun();
    /// @brief 请求保存选型草稿。
    void signalSelectionSaveDraft();

    // ── 规划工具 ──────────────────────────────────────────────
    /// @brief 请求构建 PlanningScene。
    void signalPlanningBuildScene();
    /// @brief 请求执行点到点规划验证。
    void signalPlanningRunVerification();
    /// @brief 请求保存规划草稿。
    void signalPlanningSaveDraft();

    // ── 方案导出工具 ──────────────────────────────────────────
    /// @brief 请求生成 SchemeSnapshot。
    void signalSchemeGenerateSnapshot();
    /// @brief 请求重新生成并保存 SchemeSnapshot。
    void signalSchemeRegenerateAndSave();
    /// @brief 请求加载 SchemeSnapshot。
    void signalSchemeLoadSnapshot();
    /// @brief 请求导出 JSON。
    void signalSchemeExportJson();

    // ── 视图/相机 ─────────────────────────────────────────────
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

    /// @brief 请求设置中央三维视图中的地面网格显示状态。
    void signalSetGroundGridVisible(bool visible);

    /// @brief 请求设置中央三维视图中的屏幕角落方向坐标轴显示状态。
    void signalSetCornerAxesVisible(bool visible);

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
    QWidget* CreateTopologyTab();
    QWidget* CreateKinematicsTab();
    QWidget* CreateDynamicsTab();
    QWidget* CreateRequirementTab();
    QWidget* CreateSelectionTab();
    QWidget* CreatePlanningTab();
    QWidget* CreateSchemeTab();
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
    QToolButton* CreateDisabledButton(const QString& text, const QString& reason);
    QWidget* CreateButtonGroup(const QString& title, const QList<QToolButton*>& buttons);

private:
    QTabWidget* m_tabs = nullptr;
    /// @brief 模块名 → Tab 页索引映射，供 SwitchToContextTab 快速定位。
    QMap<QString, int> m_moduleTabIndexMap;

    // ── 构型工具按钮指针 ──
    QToolButton* m_topologyRefreshBtn = nullptr;
    QToolButton* m_topologyGenerateBtn = nullptr;
    QToolButton* m_topologyValidateBtn = nullptr;
    QToolButton* m_topologySaveBtn = nullptr;

    // ── 运动学工具按钮指针 ──
    QToolButton* m_kinematicsImportUrdfBtn = nullptr;
    QToolButton* m_kinematicsBuildFromTopologyBtn = nullptr;
    QToolButton* m_kinematicsPromoteDhBtn = nullptr;
    QToolButton* m_kinematicsSwitchUrdfBtn = nullptr;
    QToolButton* m_kinematicsRunFkBtn = nullptr;
    QToolButton* m_kinematicsRunIkBtn = nullptr;
    QToolButton* m_kinematicsSampleWorkspaceBtn = nullptr;
    QToolButton* m_kinematicsSaveBtn = nullptr;

    // ── 动力学工具按钮指针 ──
    QToolButton* m_dynamicsBuildFromKinematicsBtn = nullptr;
    QToolButton* m_dynamicsRunAnalysisBtn = nullptr;
    QToolButton* m_dynamicsSaveBtn = nullptr;
};

} // namespace RoboSDP::Desktop::Ribbon
