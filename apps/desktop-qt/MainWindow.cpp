#include "apps/desktop-qt/MainWindow.h"

#include "apps/desktop-qt/widgets/dialogs/GlobalSaveResultDialog.h"
#include "apps/desktop-qt/widgets/empty/ProjectEmptyStateWidget.h"
#include "apps/desktop-qt/widgets/ribbon/RibbonBarWidget.h"
#include "apps/desktop-qt/widgets/vtk/RobotVtkView.h"
#include "core/infrastructure/ProjectManager.h"
#include "core/repository/ProjectService.h"
#include "modules/dynamics/ui/DynamicsWidget.h"
#include "modules/kinematics/ui/KinematicsWidget.h"
#include "modules/planning/ui/PlanningWidget.h"
#include "modules/requirement/ui/RequirementWidget.h"
#include "modules/scheme/ui/SchemeWidget.h"
#include "modules/selection/ui/SelectionWidget.h"
#include "modules/topology/ui/TopologyWidget.h"

#include <QDateTime>
#include <QDockWidget>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QStackedWidget>
#include <QStatusBar>
#include <QToolBar>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QMenu>   
#include <QCloseEvent> 

namespace RoboSDP::Desktop
{

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    BuildUi();
}

void MainWindow::BuildUi()
{
    setWindowTitle(QStringLiteral("RoboSDP Desktop"));
    resize(1440, 900);

    CreateRibbonBar();
    CreateCentralView();
    CreateProjectTreeDock();
    CreatePropertyDock();
    CreateLogDock();

    connect(
        &RoboSDP::Infrastructure::ProjectManager::instance(),
        &RoboSDP::Infrastructure::ProjectManager::projectPathChanged,
        this,
        [this](const QString& projectRootPath) {
            HandleProjectContextPathChanged(projectRootPath);
        });

    connect(
        m_projectTree,
        &QTreeWidget::currentItemChanged,
        this,
        [this](QTreeWidgetItem* current, QTreeWidgetItem*) { HandleProjectTreeSelectionChanged(current); });

    ShowEmptyProjectState();
}

void MainWindow::CreateRibbonBar()
{
    auto* toolbar = new QToolBar(QStringLiteral("顶部功能区"), this);
    toolbar->setObjectName(QStringLiteral("main_ribbon_toolbar"));
    toolbar->setMovable(false);
    toolbar->setFloatable(false);

    m_ribbonBar = new RoboSDP::Desktop::Ribbon::RibbonBarWidget(toolbar);
    toolbar->addWidget(m_ribbonBar);
    addToolBar(Qt::TopToolBarArea, toolbar);

    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalCreateNewProject,
        this,
        [this]() { HandleCreateNewProjectRequested(); });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalOpenProject,
        this,
        [this]() { HandleOpenProjectRequested(); });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalGlobalSaveRequested,
        this,
        [this]() { HandleGlobalSaveRequested(); });
    // ── 构型工具信号路由 ──────────────────────────────────────
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalTopologyRefreshTemplates,
            this, [this]() { if (m_topologyWidget) m_topologyWidget->TriggerRefreshTemplates(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalTopologyGenerate,
            this, [this]() { if (m_topologyWidget) m_topologyWidget->TriggerGenerate(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalTopologyValidate,
            this, [this]() { if (m_topologyWidget) m_topologyWidget->TriggerValidate(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalTopologySaveDraft,
            this, [this]() { if (m_topologyWidget) m_topologyWidget->TriggerSaveDraft(); });

    // ── 运动学工具信号路由 ────────────────────────────────────
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalKinematicsImportUrdf,
            this, [this]() { if (m_kinematicsWidget) m_kinematicsWidget->TriggerImportUrdf(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalKinematicsBuildFromTopology,
            this, [this]() { if (m_kinematicsWidget) m_kinematicsWidget->TriggerBuildFromTopology(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalKinematicsPromoteToDhMaster,
            this, [this]() { if (m_kinematicsWidget) m_kinematicsWidget->TriggerPromoteToDhMaster(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalKinematicsSwitchToUrdfMaster,
            this, [this]() { if (m_kinematicsWidget) m_kinematicsWidget->TriggerSwitchToUrdfMaster(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalKinematicsRunFk,
            this, [this]() { if (m_kinematicsWidget) m_kinematicsWidget->TriggerRunFk(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalKinematicsRunIk,
            this, [this]() { if (m_kinematicsWidget) m_kinematicsWidget->TriggerRunIk(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalKinematicsSampleWorkspace,
            this, [this]() { if (m_kinematicsWidget) m_kinematicsWidget->TriggerSampleWorkspace(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalKinematicsSaveDraft,
            this, [this]() { if (m_kinematicsWidget) m_kinematicsWidget->TriggerSaveDraft(); });

    // ── 动力学工具信号路由 ────────────────────────────────────
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalDynamicsBuildFromKinematics,
            this, [this]() { if (m_dynamicsWidget) m_dynamicsWidget->TriggerBuildFromKinematics(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalDynamicsRunAnalysis,
            this, [this]() { if (m_dynamicsWidget) m_dynamicsWidget->TriggerRunAnalysis(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalDynamicsSaveDraft,
            this, [this]() { if (m_dynamicsWidget) m_dynamicsWidget->TriggerSaveDraft(); });

    // ── 需求校验工具信号路由 ──────────────────────────────────
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalRequirementValidate,
            this, [this]() { if (m_requirementWidget) m_requirementWidget->TriggerValidate(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalRequirementSaveDraft,
            this, [this]() { if (m_requirementWidget) { m_requirementWidget->SaveCurrentDraft(); } });

    // ── 选型工具信号路由 ──────────────────────────────────────
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSelectionRun,
            this, [this]() { if (m_selectionWidget) m_selectionWidget->TriggerRunSelection(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSelectionSaveDraft,
            this, [this]() { if (m_selectionWidget) { m_selectionWidget->SaveCurrentDraft(); } });

    // ── 规划工具信号路由 ──────────────────────────────────────
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalPlanningBuildScene,
            this, [this]() { if (m_planningWidget) m_planningWidget->TriggerBuildScene(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalPlanningRunVerification,
            this, [this]() { if (m_planningWidget) m_planningWidget->TriggerRunVerification(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalPlanningSaveDraft,
            this, [this]() { if (m_planningWidget) { m_planningWidget->SaveCurrentDraft(); } });

    // ── 方案导出工具信号路由 ──────────────────────────────────
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSchemeGenerateSnapshot,
            this, [this]() { if (m_schemeWidget) m_schemeWidget->TriggerGenerateSnapshot(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSchemeRegenerateAndSave,
            this, [this]() { if (m_schemeWidget) m_schemeWidget->TriggerRegenerateAndSaveSnapshot(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSchemeLoadSnapshot,
            this, [this]() { if (m_schemeWidget) m_schemeWidget->TriggerLoadSnapshot(); });
    connect(m_ribbonBar, &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSchemeExportJson,
            this, [this]() { if (m_schemeWidget) m_schemeWidget->TriggerExportJson(); });

    const auto visibilityText = [](bool visible) {
        return visible ? QStringLiteral("开启") : QStringLiteral("关闭");
    };
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSetSkeletonVisible,
        this,
        [this, visibilityText](bool visible) {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->SetSkeletonVisible(visible);
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 显示骨架：%1").arg(visibilityText(visible)));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSetVisualMeshVisible,
        this,
        [this, visibilityText](bool visible) {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->SetVisualMeshVisible(visible);
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 显示 Visual Mesh：%1").arg(visibilityText(visible)));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSetCollisionMeshVisible,
        this,
        [this, visibilityText](bool visible) {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->SetCollisionMeshVisible(visible);
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 显示 Collision Mesh：%1").arg(visibilityText(visible)));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSetJointAxesVisible,
        this,
        [this, visibilityText](bool visible) {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->SetJointAxesVisible(visible);
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 显示关节轴：%1").arg(visibilityText(visible)));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSetAxesVisible,
        this,
        [this, visibilityText](bool visible) {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->SetAxesVisible(visible);
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 显示坐标系：%1").arg(visibilityText(visible)));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSetGroundGridVisible,
        this,
        [this, visibilityText](bool visible) {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->SetGroundGridVisible(visible);
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 显示地面网格：%1").arg(visibilityText(visible)));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSetCornerAxesVisible,
        this,
        [this, visibilityText](bool visible) {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->SetCornerAxesVisible(visible);
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 显示角落坐标轴：%1").arg(visibilityText(visible)));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSetLinkLabelsVisible,
        this,
        [this, visibilityText](bool visible) {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->SetLinkLabelsVisible(visible);
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 显示 Link 标签：%1").arg(visibilityText(visible)));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSetJointLabelsVisible,
        this,
        [this, visibilityText](bool visible) {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->SetJointLabelsVisible(visible);
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 显示 Joint 标签：%1").arg(visibilityText(visible)));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalResetCameraRequested,
        this,
        [this]() {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->ResetCameraToCurrentScene();
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 用户请求重置三维相机"));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalCameraFrontViewRequested,
        this,
        [this]() {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->SetFrontCameraView();
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 用户请求切换至正视图"));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalCameraSideViewRequested,
        this,
        [this]() {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->SetSideCameraView();
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 用户请求切换至侧视图"));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalCameraTopViewRequested,
        this,
        [this]() {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->SetTopCameraView();
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 用户请求切换至俯视图"));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalCameraIsometricViewRequested,
        this,
        [this]() {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->SetIsometricCameraView();
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 用户请求切换至等轴测视图"));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSetProjectTreeVisible,
        this,
        [this, visibilityText](bool visible) {
            if (m_projectTreeDock != nullptr)
            {
                m_projectTreeDock->setVisible(visible);
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 项目树面板：%1").arg(visibilityText(visible)));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSetPropertyPanelVisible,
        this,
        [this, visibilityText](bool visible) {
            if (m_propertyDock != nullptr)
            {
                m_propertyDock->setVisible(visible);
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 属性面板：%1").arg(visibilityText(visible)));
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalSetOutputPanelVisible,
        this,
        [this, visibilityText](bool visible) {
            if (m_logDock != nullptr)
            {
                m_logDock->setVisible(visible);
            }
            AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 输出信息面板：%1").arg(visibilityText(visible)));
        });
}

void MainWindow::CreateCentralView()
{
    // 本轮只把中央区域接到最小 VTK 视图，不改动项目树、属性栏和日志栏结构。
    m_robotVtkView = new RoboSDP::Desktop::Vtk::RobotVtkView(this);
    setCentralWidget(m_robotVtkView);
}

void MainWindow::CreateProjectTreeDock()
{
    m_projectTreeDock = new QDockWidget(QStringLiteral("项目树"), this);
    m_projectTreeDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

    m_projectTree = new QTreeWidget(m_projectTreeDock);
    m_projectTree->setHeaderLabel(QStringLiteral("项目树"));

    m_projectTreeDock->setWidget(m_projectTree);
    addDockWidget(Qt::LeftDockWidgetArea, m_projectTreeDock);

    // 【新增：开启右键菜单策略并绑定信号】
    m_projectTree->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(
        m_projectTree,
        &QTreeWidget::customContextMenuRequested,
        this,
        &MainWindow::HandleProjectTreeContextMenu);

}

void MainWindow::CreatePropertyDock()
{
    m_propertyDock = new QDockWidget(QStringLiteral("属性面板"), this);
    m_propertyDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

    m_propertyStack = new QStackedWidget(m_propertyDock);

    m_emptyProjectWidget = new RoboSDP::Desktop::Widgets::ProjectEmptyStateWidget(m_propertyStack);
    m_requirementWidget = new RoboSDP::Requirement::Ui::RequirementWidget(m_propertyStack);
    m_topologyWidget = new RoboSDP::Topology::Ui::TopologyWidget(m_propertyStack);
    m_kinematicsWidget = new RoboSDP::Kinematics::Ui::KinematicsWidget(m_propertyStack);
    m_dynamicsWidget = new RoboSDP::Dynamics::Ui::DynamicsWidget(m_propertyStack);
    m_selectionWidget = new RoboSDP::Selection::Ui::SelectionWidget(m_propertyStack);
    m_planningWidget = new RoboSDP::Planning::Ui::PlanningWidget(m_propertyStack);
    m_schemeWidget = new RoboSDP::Scheme::Ui::SchemeWidget(m_propertyStack);
    m_placeholderPropertyPanel = new QPlainTextEdit(m_propertyStack);
    m_placeholderPropertyPanel->setReadOnly(true);
    m_placeholderPropertyPanel->setPlainText(
        QStringLiteral(
            "当前已接入 Requirement / Topology / Kinematics / Dynamics / Selection / Planning / Scheme 七个模块的最小页面骨架。\n"
            "请在左侧树选择具体节点继续录入、生成、求解、保存与加载。\n"
            "其余模块仍保持占位状态。"));

    m_propertyStack->addWidget(m_emptyProjectWidget);
    m_propertyStack->addWidget(m_requirementWidget);
    m_propertyStack->addWidget(m_topologyWidget);
    m_propertyStack->addWidget(m_kinematicsWidget);
    m_propertyStack->addWidget(m_dynamicsWidget);
    m_propertyStack->addWidget(m_selectionWidget);
    m_propertyStack->addWidget(m_planningWidget);
    m_propertyStack->addWidget(m_schemeWidget);
    m_propertyStack->addWidget(m_placeholderPropertyPanel);
    m_propertyDock->setWidget(m_propertyStack);
    addDockWidget(Qt::RightDockWidgetArea, m_propertyDock);

    // 中文说明：全局保存固定按主链顺序执行；协调器只保存接口指针，不接管 Widget 生命周期。
    m_projectSaveCoordinator.RegisterParticipant(m_requirementWidget);
    m_projectSaveCoordinator.RegisterParticipant(m_topologyWidget);
    m_projectSaveCoordinator.RegisterParticipant(m_kinematicsWidget);
    m_projectSaveCoordinator.RegisterParticipant(m_dynamicsWidget);
    m_projectSaveCoordinator.RegisterParticipant(m_selectionWidget);
    m_projectSaveCoordinator.RegisterParticipant(m_planningWidget);
    m_projectSaveCoordinator.RegisterParticipant(m_schemeWidget);

    connect(
        m_requirementWidget,
        &RoboSDP::Requirement::Ui::RequirementWidget::LogMessageGenerated,
        this,
        [this](const QString& message) { AppendLogLine(message); });

    connect(
        m_topologyWidget,
        &RoboSDP::Topology::Ui::TopologyWidget::LogMessageGenerated,
        this,
        [this](const QString& message) { AppendLogLine(message); });

    // ── Topology 状态变更 → 刷新 Ribbon 构型按钮启用/禁用 ──
    connect(
        m_topologyWidget,
        &RoboSDP::Topology::Ui::TopologyWidget::StatusChanged,
        this,
        [this]() {
            if (m_topologyWidget == nullptr || m_ribbonBar == nullptr) return;
            m_ribbonBar->SetTopologyButtonsEnabled(
                m_topologyWidget->CanRefreshTemplates(),
                m_topologyWidget->CanGenerate(),
                m_topologyWidget->CanValidate(),
                m_topologyWidget->CanSaveDraft());
        });

    // --- 新增：接收 Topology 的实时预览并发送给 VTK 视图 ---
    connect(
        m_topologyWidget,
        &RoboSDP::Topology::Ui::TopologyWidget::TopologyPreviewGenerated,
        this,
        [this](const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& scene) {
            if (m_robotVtkView != nullptr)
            {
                // 先临时关闭网格模型显示(因为此时只有参数没有3D模型文件)
                m_robotVtkView->SetVisualMeshVisible(false);
                m_robotVtkView->SetCollisionMeshVisible(false);
                
                // 核心逻辑：只有当 m_shouldResetNextPreview 为 true 时才重置相机
                m_robotVtkView->ShowPreviewScene(scene, m_shouldResetNextPreview);
                // 执行完一次重置后，立即将标志位置为 false
                // 这样后续由滑块(DirtyTracking)触发的预览信号将不会重置相机
                m_shouldResetNextPreview = false;
                statusBar()->showMessage(QStringLiteral("中央区域：已实时同步构型尺寸骨架。"));
            }
        });

    connect(
        m_kinematicsWidget,
        &RoboSDP::Kinematics::Ui::KinematicsWidget::LogMessageGenerated,
        this,
        [this](const QString& message) { AppendLogLine(message); });

    // ── Kinematics 状态变更 → 刷新 Ribbon 运动学按钮启用/禁用 ──
    connect(
        m_kinematicsWidget,
        &RoboSDP::Kinematics::Ui::KinematicsWidget::StatusChanged,
        this,
        [this]() {
            if (m_kinematicsWidget == nullptr || m_ribbonBar == nullptr) return;
            m_ribbonBar->SetKinematicsButtonsEnabled(
                m_kinematicsWidget->CanImportUrdf(),
                m_kinematicsWidget->CanBuildFromTopology(),
                m_kinematicsWidget->CanPromoteToDhMaster(),
                m_kinematicsWidget->CanSwitchToUrdfMaster(),
                m_kinematicsWidget->CanRunFk(),
                m_kinematicsWidget->CanRunIk(),
                m_kinematicsWidget->CanSampleWorkspace(),
                m_kinematicsWidget->CanSaveDraft());
        });

    connect(
        m_kinematicsWidget,
        &RoboSDP::Kinematics::Ui::KinematicsWidget::TelemetryStatusGenerated,
        this,
        [this](const QString& message, bool warning) {
            ShowTelemetryStatus(QStringLiteral("Kinematics"), message, warning);
        });

    connect(
        m_kinematicsWidget,
        &RoboSDP::Kinematics::Ui::KinematicsWidget::PreviewSceneGenerated,
        this,
        [this](const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& scene) {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->ShowPreviewScene(scene, m_shouldResetNextPreview);
            }

            if (scene.IsEmpty())
            {
                // 中文说明：模型结构切换时 Kinematics 会发送空场景，中央视图借此清理旧骨架/Mesh 缓存。
                statusBar()->showMessage(QStringLiteral("中央区域：已清空骨架预览缓存，等待新的模型同步。"));
                AppendLogLine(QStringLiteral("[Desktop] 中央骨架预览缓存已清空。"));
                return;
            }

            const QString modelName = scene.model_name.isEmpty() ? QStringLiteral("未命名模型") : scene.model_name;
            statusBar()->showMessage(
                QStringLiteral("中央区域：已加载骨架预览 - %1（节点 %2，连杆段 %3）")
                    .arg(modelName)
                    .arg(scene.nodes.size())
                    .arg(scene.segments.size()));
            AppendLogLine(
                QStringLiteral("[Desktop] 骨架预览已同步到中央三维视图：%1").arg(modelName));
        });

    connect(
        m_kinematicsWidget,
        &RoboSDP::Kinematics::Ui::KinematicsWidget::PreviewPosesUpdated,
        this,
        [this](const RoboSDP::Kinematics::Ui::PreviewPoseMap& linkWorldPoses) {
            if (m_robotVtkView != nullptr)
            {
                // 中文说明：属性面板只产生轻量位姿 DTO，中央视图负责复用 Actor 缓存并更新 vtkTransform。
                m_robotVtkView->UpdatePreviewPoses(linkWorldPoses);
            }
        });
    connect(
        m_kinematicsWidget,
        &RoboSDP::Kinematics::Ui::KinematicsWidget::PreviewSceneGenerated,
        this,
        [this](const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& scene) {
            if (m_robotVtkView != nullptr)
            {
                // --- 开启硬核调试模式的视觉开关 ---
                m_robotVtkView->SetVisualMeshVisible(false);    // 隐藏蒙皮，看骨架
                m_robotVtkView->SetJointAxesVisible(true);     // 显示黄色旋转轴线
                m_robotVtkView->SetAxesVisible(true);          // 显示 RGB 三色坐标轴
                m_robotVtkView->SetJointLabelsVisible(true);   // 显示关节名称标签
                
                // 依然不重置相机，保持观察连续性
                m_robotVtkView->ShowPreviewScene(scene, false); 
            }
        });
    connect(
        m_dynamicsWidget,
        &RoboSDP::Dynamics::Ui::DynamicsWidget::LogMessageGenerated,
        this,
        [this](const QString& message) { AppendLogLine(message); });

    // ── Dynamics 状态变更 → 刷新 Ribbon 动力学按钮启用/禁用 ──
    connect(
        m_dynamicsWidget,
        &RoboSDP::Dynamics::Ui::DynamicsWidget::StatusChanged,
        this,
        [this]() {
            if (m_dynamicsWidget == nullptr || m_ribbonBar == nullptr) return;
            m_ribbonBar->SetDynamicsButtonsEnabled(
                m_dynamicsWidget->CanBuildFromKinematics(),
                m_dynamicsWidget->CanRunAnalysis(),
                m_dynamicsWidget->CanSaveDraft());
        });

    connect(
        m_dynamicsWidget,
        &RoboSDP::Dynamics::Ui::DynamicsWidget::TelemetryStatusGenerated,
        this,
        [this](const QString& message, bool warning) {
            ShowTelemetryStatus(QStringLiteral("Dynamics"), message, warning);
        });

    connect(
        m_selectionWidget,
        &RoboSDP::Selection::Ui::SelectionWidget::LogMessageGenerated,
        this,
        [this](const QString& message) { AppendLogLine(message); });

    connect(
        m_planningWidget,
        &RoboSDP::Planning::Ui::PlanningWidget::LogMessageGenerated,
        this,
        [this](const QString& message) { AppendLogLine(message); });

    connect(
        m_schemeWidget,
        &RoboSDP::Scheme::Ui::SchemeWidget::LogMessageGenerated,
        this,
        [this](const QString& message) { AppendLogLine(message); });
}

void MainWindow::CreateLogDock()
{
    m_logDock = new QDockWidget(QStringLiteral("输出信息"), this);
    m_logDock->setAllowedAreas(Qt::BottomDockWidgetArea);

    m_logPanel = new QPlainTextEdit(m_logDock);
    m_logPanel->setReadOnly(true);

    m_logDock->setWidget(m_logPanel);
    addDockWidget(Qt::BottomDockWidgetArea, m_logDock);

    AppendLogLine(QStringLiteral("RoboSDP Desktop 启动完成。"));
    AppendLogLine(QStringLiteral("当前未打开项目，请先新建或打开 RoboSDP 项目。"));
}

void MainWindow::ResetProjectTreeItemPointers()
{
    m_projectRootTreeItem = nullptr;
    m_requirementTreeItem = nullptr;
    m_topologyTreeItem = nullptr;
    m_kinematicsTreeItem = nullptr;
    m_dynamicsTreeItem = nullptr;
    m_selectionTreeItem = nullptr;
    m_planningTreeItem = nullptr;
    m_schemeTreeItem = nullptr;
}

void MainWindow::ShowEmptyProjectState()
{
    if (m_projectTree != nullptr)
    {
        m_projectTree->blockSignals(true);
        m_projectTree->clear();
        ResetProjectTreeItemPointers();
        m_projectRootTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("未打开项目")});
        m_projectTree->addTopLevelItem(m_projectRootTreeItem);
        m_projectTree->setCurrentItem(m_projectRootTreeItem);
        m_projectTree->blockSignals(false);
    }

    if (m_propertyStack != nullptr && m_emptyProjectWidget != nullptr)
    {
        m_propertyStack->setCurrentWidget(m_emptyProjectWidget);
    }

    statusBar()->setStyleSheet(QStringLiteral("QStatusBar{color:#667085;}"));
    statusBar()->showMessage(QStringLiteral("未打开项目：请先新建或打开 RoboSDP 项目。"));
}

void MainWindow::ShowActiveProjectState(const QString& projectRootPath)
{
    if (m_projectTree == nullptr)
    {
        return;
    }

    const QFileInfo projectRootInfo(projectRootPath);

    m_projectTree->blockSignals(true);
    m_projectTree->clear();
    ResetProjectTreeItemPointers();

    m_projectRootTreeItem = new QTreeWidgetItem(QStringList{
        QStringLiteral("%1 (%2)")
            .arg(projectRootInfo.fileName(), QDir::toNativeSeparators(projectRootInfo.absoluteFilePath()))});
    m_requirementTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("任务需求")});
    m_topologyTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("构型设计")});
    m_kinematicsTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("运动学分析")});
    m_dynamicsTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("动力学分析")});
    m_selectionTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("驱动选型")});
    m_planningTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("规划与分析")});
    m_schemeTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("结果与导出")});

    // 中文说明：项目树仅在有效项目上下文中暴露业务模块，避免无项目状态下误进入工作态。
    m_projectRootTreeItem->addChild(m_requirementTreeItem);
    m_projectRootTreeItem->addChild(m_topologyTreeItem);
    m_projectRootTreeItem->addChild(m_kinematicsTreeItem);
    m_projectRootTreeItem->addChild(m_dynamicsTreeItem);
    m_projectRootTreeItem->addChild(m_selectionTreeItem);
    m_projectRootTreeItem->addChild(m_planningTreeItem);
    m_projectRootTreeItem->addChild(m_schemeTreeItem);

    m_projectTree->addTopLevelItem(m_projectRootTreeItem);
    m_projectRootTreeItem->setExpanded(true);
    m_projectTree->setCurrentItem(m_requirementTreeItem);
    m_projectTree->blockSignals(false);

    HandleProjectTreeSelectionChanged(m_requirementTreeItem);
}

void MainWindow::HandleProjectTreeSelectionChanged(QTreeWidgetItem* currentItem)
{
    if (currentItem == nullptr || m_propertyStack == nullptr)
    {
        return;
    }

    const bool projectOpened =
        !RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath().trimmed().isEmpty();
    if (!projectOpened)
    {
        if (m_emptyProjectWidget != nullptr)
        {
            m_propertyStack->setCurrentWidget(m_emptyProjectWidget);
        }
        statusBar()->setStyleSheet(QStringLiteral("QStatusBar{color:#667085;}"));
        statusBar()->showMessage(QStringLiteral("未打开项目：请先新建或打开 RoboSDP 项目。"));
        return;
    }

    statusBar()->setStyleSheet(QString());

    if (currentItem == m_requirementTreeItem)
    {
        m_propertyStack->setCurrentWidget(m_requirementWidget);
        m_ribbonBar->SwitchToContextTab(QStringLiteral("Requirement"));
        statusBar()->showMessage(QStringLiteral("当前页面：Requirement"));
        return;
    }

    if (currentItem == m_topologyTreeItem)
    {
        m_propertyStack->setCurrentWidget(m_topologyWidget);
        m_ribbonBar->SwitchToContextTab(QStringLiteral("Topology"));
        statusBar()->showMessage(QStringLiteral("当前页面：Topology"));
        // 优化逻辑：判断是否是首次进入
        if (m_isFirstTopologyEntry)
        {
            // 1. 设置标志位，允许下一次收到的信号重置相机
            m_shouldResetNextPreview = true;
            
            // 2. 触发 TopologyWidget 生成初始预览信号
            m_topologyWidget->ForceEmitPreview();
            
            // 3. 标记已完成首次进入，后续再切回来将不再重置视角
            m_isFirstTopologyEntry = false;
        }
        // --- 新增：切到构型页面时，强制刷新一次三维视图 ---
        if (m_topologyWidget != nullptr) {
            m_topologyWidget->ForceEmitPreview();
        }
        return;
    }

    if (currentItem == m_kinematicsTreeItem)
    {
        m_propertyStack->setCurrentWidget(m_kinematicsWidget);
        m_ribbonBar->SwitchToContextTab(QStringLiteral("Kinematics"));
        statusBar()->showMessage(QStringLiteral("当前页面：Kinematics"));
        return;
    }

    if (currentItem == m_dynamicsTreeItem)
    {
        m_propertyStack->setCurrentWidget(m_dynamicsWidget);
        m_ribbonBar->SwitchToContextTab(QStringLiteral("Dynamics"));
        statusBar()->showMessage(QStringLiteral("当前页面：Dynamics"));
        return;
    }

    if (currentItem == m_selectionTreeItem)
    {
        m_propertyStack->setCurrentWidget(m_selectionWidget);
        m_ribbonBar->SwitchToContextTab(QStringLiteral("Selection"));
        statusBar()->showMessage(QStringLiteral("当前页面：Selection"));
        return;
    }

    if (currentItem == m_planningTreeItem)
    {
        m_propertyStack->setCurrentWidget(m_planningWidget);
        m_ribbonBar->SwitchToContextTab(QStringLiteral("Planning"));
        statusBar()->showMessage(QStringLiteral("当前页面：Planning"));
        return;
    }

    if (currentItem == m_schemeTreeItem)
    {
        m_propertyStack->setCurrentWidget(m_schemeWidget);
        m_ribbonBar->SwitchToContextTab(QStringLiteral("Scheme"));
        statusBar()->showMessage(QStringLiteral("当前页面：Scheme"));
        return;
    }

    m_propertyStack->setCurrentWidget(m_placeholderPropertyPanel);
    m_ribbonBar->SwitchToContextTab(QStringLiteral("File"));
    statusBar()->showMessage(QStringLiteral("当前节点尚未实现编辑页面"));
}


void MainWindow::HandleCreateNewProjectRequested()
{
    AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 用户请求新建项目"));

    const QString selectedDirectory = QFileDialog::getExistingDirectory(
        this,
        QStringLiteral("选择项目根目录"),
        QString(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (selectedDirectory.trimmed().isEmpty())
    {
        AppendLogLine(QStringLiteral("[INFO] [项目服务] 用户取消新建项目。"));
        return;
    }

    RoboSDP::Repository::ProjectService projectService;
    const RoboSDP::Repository::ProjectCreateResult createResult =
        projectService.CreateProject(selectedDirectory);

    if (!createResult.success)
    {
        AppendLogLine(QStringLiteral("[WARN] [项目服务] %1").arg(createResult.message));
        statusBar()->setStyleSheet(QStringLiteral("QStatusBar{color:#b54708;font-weight:600;}"));
        statusBar()->showMessage(createResult.message);
        return;
    }

    AppendLogLine(QStringLiteral("[INFO] [项目服务] %1").arg(createResult.message));
    AppendLogLine(QStringLiteral("[INFO] [项目服务] 已生成 project.json 和第一阶段最小目录骨架。"));
    RoboSDP::Infrastructure::ProjectManager::instance().setCurrentProjectPath(createResult.project_root_path);
}

void MainWindow::HandleOpenProjectRequested()
{
    AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 用户请求打开项目"));

    const QString selectedDirectory = QFileDialog::getExistingDirectory(
        this,
        QStringLiteral("打开 RoboSDP 项目"),
        RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (selectedDirectory.trimmed().isEmpty())
    {
        AppendLogLine(QStringLiteral("[INFO] [项目服务] 用户取消打开项目。"));
        return;
    }

    const QFileInfo projectFileInfo(QDir(selectedDirectory).filePath(QStringLiteral("project.json")));
    if (!projectFileInfo.exists() || !projectFileInfo.isFile())
    {
        const QString message = QStringLiteral("该目录不是有效的 RoboSDP 项目：缺少 project.json。");
        AppendLogLine(QStringLiteral("[ERROR] [项目服务] %1").arg(message));
        QMessageBox::warning(this, QStringLiteral("打开项目失败"), message);
        return;
    }

    // 中文说明：通过 ProjectManager 更新全局上下文，业务模块依靠信号自动刷新只读项目目录。
    RoboSDP::Infrastructure::ProjectManager::instance().setCurrentProjectPath(selectedDirectory);
    AppendLogLine(QStringLiteral("[INFO] [项目服务] 已打开项目：%1").arg(QDir::toNativeSeparators(selectedDirectory)));
}

void MainWindow::HandleGlobalSaveRequested()
{
    AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 用户请求全局保存"));

    if (RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath().trimmed().isEmpty())
    {
        const QString message = QStringLiteral("当前未打开项目，无法执行全局保存。");
        AppendLogLine(QStringLiteral("[ERROR] [全局保存] %1").arg(message));
        statusBar()->setStyleSheet(QStringLiteral("QStatusBar{color:#b54708;font-weight:600;}"));
        statusBar()->showMessage(message);
        return;
    }

    const RoboSDP::Infrastructure::ProjectSaveSummary summary =
        m_projectSaveCoordinator.SaveAll();
    if (summary.project_root_path.trimmed().isEmpty())
    {
        AppendLogLine(QStringLiteral("[ERROR] [全局保存] %1").arg(summary.message));
        statusBar()->setStyleSheet(QStringLiteral("QStatusBar{color:#b54708;font-weight:600;}"));
        statusBar()->showMessage(summary.message);
        return;
    }

    for (const RoboSDP::Infrastructure::ProjectSaveItemResult& itemResult : summary.item_results)
    {
        const QString logLevel = itemResult.dependency_refresh_required
            ? QStringLiteral("[WARN]")
            : (itemResult.success ? QStringLiteral("[INFO]") : QStringLiteral("[ERROR]"));
        AppendLogLine(
            QStringLiteral("%1 [全局保存] %2：%3")
                .arg(logLevel, itemResult.module_name, itemResult.message));
    }

    AppendLogLine(QStringLiteral("%1 [全局保存] %2")
        .arg(summary.success ? QStringLiteral("[INFO]") : QStringLiteral("[ERROR]"), summary.message));
    statusBar()->setStyleSheet(
        summary.success
            ? QStringLiteral("QStatusBar{color:#1b7f3b;}")
            : QStringLiteral("QStatusBar{color:#b54708;font-weight:600;}"));
    statusBar()->showMessage(summary.message);

    // 中文说明：保存完成后只展示汇总结果，不在弹窗中再次触发任何写盘动作。
    RoboSDP::Desktop::Dialogs::GlobalSaveResultDialog resultDialog(summary, this);
    connect(
        &resultDialog,
        &RoboSDP::Desktop::Dialogs::GlobalSaveResultDialog::ModuleNavigationRequested,
        this,
        [this](const QString& moduleName) { HandleGlobalSaveResultNavigateRequested(moduleName); });
    resultDialog.exec();
}

void MainWindow::HandleGlobalSaveResultNavigateRequested(const QString& moduleName)
{
    QTreeWidgetItem* targetItem = nullptr;
    QString targetName;
    if (moduleName == QStringLiteral("Requirement"))
    {
        targetItem = m_requirementTreeItem;
        targetName = QStringLiteral("需求定义");
    }
    else if (moduleName == QStringLiteral("Topology"))
    {
        targetItem = m_topologyTreeItem;
        targetName = QStringLiteral("构型设计");
    }
    else if (moduleName == QStringLiteral("Kinematics"))
    {
        targetItem = m_kinematicsTreeItem;
        targetName = QStringLiteral("运动学建模");
    }
    else if (moduleName == QStringLiteral("Dynamics"))
    {
        targetItem = m_dynamicsTreeItem;
        targetName = QStringLiteral("动力学分析");
    }
    else if (moduleName == QStringLiteral("Selection"))
    {
        targetItem = m_selectionTreeItem;
        targetName = QStringLiteral("驱动选型");
    }
    else if (moduleName == QStringLiteral("Planning"))
    {
        targetItem = m_planningTreeItem;
        targetName = QStringLiteral("规划与分析");
    }
    else if (moduleName == QStringLiteral("Scheme"))
    {
        targetItem = m_schemeTreeItem;
        targetName = QStringLiteral("结果与导出");
    }

    if (m_projectTree == nullptr || targetItem == nullptr)
    {
        AppendLogLine(QStringLiteral("[WARN] [全局保存] 无法定位模块：%1").arg(moduleName));
        return;
    }

    // 中文说明：定位只复用项目树当前页切换，不改变保存结果或模块 dirty 状态。
    AppendLogLine(QStringLiteral("[INFO] [全局保存] 定位模块：%1").arg(targetName));
    m_projectTree->setCurrentItem(targetItem);
}

void MainWindow::HandleProjectContextPathChanged(const QString& projectRootPath)
{
    if (projectRootPath.trimmed().isEmpty())
    {
        ShowEmptyProjectState();
        AppendLogLine(QStringLiteral("[INFO] [项目上下文] 当前未打开项目。"));
        return;
    }

    ShowActiveProjectState(projectRootPath);

    statusBar()->setStyleSheet(QStringLiteral("QStatusBar{color:#1b7f3b;}"));
    statusBar()->showMessage(QStringLiteral("当前项目：%1").arg(QDir::toNativeSeparators(projectRootPath)));
    AppendLogLine(QStringLiteral("[INFO] [项目上下文] 当前项目已切换：%1").arg(QDir::toNativeSeparators(projectRootPath)));
}

void MainWindow::ShowTelemetryStatus(const QString& moduleName, const QString& message, bool warning)
{
    statusBar()->setStyleSheet(warning
        ? QStringLiteral("QStatusBar{color:#b54708;font-weight:600;}")
        : QStringLiteral("QStatusBar{color:#1b7f3b;}"));
    statusBar()->showMessage(QStringLiteral("[%1] %2").arg(moduleName, message));
}

void MainWindow::AppendLogLine(const QString& message)
{
    if (m_logPanel == nullptr)
    {
        return;
    }

    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss"));
    m_logPanel->appendPlainText(QStringLiteral("[%1] %2").arg(timestamp, message));
}
// 【新增实现代码】
void MainWindow::HandleProjectTreeContextMenu(const QPoint& pos)
{
    // 获取当前鼠标右键点击的树节点
    QTreeWidgetItem* clickedItem = m_projectTree->itemAt(pos);

    // 检查是否有打开的项目
    const bool isProjectOpen = !RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath().isEmpty();

    // 只有当点击的是“根节点（主项目）”，并且当前确实有项目打开时，才弹出菜单
    if (clickedItem == m_projectRootTreeItem && isProjectOpen)
    {
        QMenu contextMenu(this);
        
        QAction* closeAction = contextMenu.addAction(QStringLiteral("关闭项目"));
        connect(closeAction, &QAction::triggered, this, &MainWindow::HandleCloseProjectRequested);
        
        // 在鼠标当前屏幕位置显示菜单
        contextMenu.exec(m_projectTree->viewport()->mapToGlobal(pos));
    }
}

void MainWindow::HandleCloseProjectRequested()
{
    // 调用拦截器：如果返回 false，说明用户取消了操作，直接 return 终止
    if (!PromptForUnsavedChanges())
    {
        return; 
    }

    AppendLogLine(QStringLiteral("[INFO] [项目服务] 用户关闭了当前项目。"));
    RoboSDP::Infrastructure::ProjectManager::instance().setCurrentProjectPath(QString());
}
// 🔽🔽🔽 【新增：未保存进度拦截核心逻辑】 🔽🔽🔽
bool MainWindow::PromptForUnsavedChanges()
{
    // 1. 遍历检查所有注册的业务模块，看看是否有谁被修改过了 (Dirty)
    bool hasUnsaved = false;
    if (m_requirementWidget && m_requirementWidget->HasUnsavedChanges()) hasUnsaved = true;
    if (m_topologyWidget && m_topologyWidget->HasUnsavedChanges()) hasUnsaved = true;
    if (m_kinematicsWidget && m_kinematicsWidget->HasUnsavedChanges()) hasUnsaved = true;
    if (m_dynamicsWidget && m_dynamicsWidget->HasUnsavedChanges()) hasUnsaved = true;
    if (m_selectionWidget && m_selectionWidget->HasUnsavedChanges()) hasUnsaved = true;
    if (m_planningWidget && m_planningWidget->HasUnsavedChanges()) hasUnsaved = true;
    if (m_schemeWidget && m_schemeWidget->HasUnsavedChanges()) hasUnsaved = true;

    // 如果大家都干干净净，直接放行
    if (!hasUnsaved)
    {
        return true; 
    }

    // 2. 如果有未保存的内容，弹出工业标准的拦截对话框
    QMessageBox msgBox(this);
    msgBox.setWindowTitle(QStringLiteral("未保存的更改"));
    msgBox.setText(QStringLiteral("当前项目有未保存的进度，关闭前是否要保存？"));
    msgBox.setIcon(QMessageBox::Warning);
    
    // 设置三个标准按钮
    msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Save);
    
    // 汉化按钮文本以提升用户体验
    msgBox.setButtonText(QMessageBox::Save, QStringLiteral("保存并关闭"));
    msgBox.setButtonText(QMessageBox::Discard, QStringLiteral("放弃更改"));
    msgBox.setButtonText(QMessageBox::Cancel, QStringLiteral("取消"));

    const int ret = msgBox.exec();

    // 3. 根据用户的选择执行相应的动作
    if (ret == QMessageBox::Save)
    {
        // 用户选了保存，调用我们现成的全局保存协调器
        const RoboSDP::Infrastructure::ProjectSaveSummary summary = m_projectSaveCoordinator.SaveAll();
        if (!summary.success)
        {
            // 如果因为磁盘满了等原因保存失败，必须拦截关闭动作，保护现场
            QMessageBox::critical(this, QStringLiteral("保存失败"), QStringLiteral("保存项目时发生错误，请检查底部日志信息。"));
            return false; 
        }
        return true; // 保存成功，放行关闭
    }
    else if (ret == QMessageBox::Discard)
    {
        return true; // 用户明确表示不要了，放行关闭
    }

    // 用户点了取消，或者按了右上角的 X 关掉对话框，拦截关闭动作
    return false; 
}
// 🔼🔼🔼 【新增结束】 🔼🔼🔼
// 🔽🔽🔽 【新增：保护整个软件被意外退出的事件】 🔽🔽🔽
void MainWindow::closeEvent(QCloseEvent *event)
{
    const bool isProjectOpen = !RoboSDP::Infrastructure::ProjectManager::instance().getCurrentProjectPath().isEmpty();
    
    // 只有在打开了项目的情况下，才需要检查未保存进度
    if (isProjectOpen)
    {
        if (!PromptForUnsavedChanges())
        {
            event->ignore(); // 拦截操作，阻止窗口关闭
            return;
        }
    }
    
    // 放行操作，正常退出软件
    event->accept(); 
}
} // namespace RoboSDP::Desktop
