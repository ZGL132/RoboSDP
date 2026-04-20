#include "apps/desktop-qt/MainWindow.h"

#include "apps/desktop-qt/widgets/ribbon/RibbonBarWidget.h"
#include "apps/desktop-qt/widgets/vtk/RobotVtkView.h"
#include "modules/dynamics/ui/DynamicsWidget.h"
#include "modules/kinematics/ui/KinematicsWidget.h"
#include "modules/planning/ui/PlanningWidget.h"
#include "modules/requirement/ui/RequirementWidget.h"
#include "modules/scheme/ui/SchemeWidget.h"
#include "modules/selection/ui/SelectionWidget.h"
#include "modules/topology/ui/TopologyWidget.h"

#include <QDateTime>
#include <QDockWidget>
#include <QPlainTextEdit>
#include <QStackedWidget>
#include <QStatusBar>
#include <QToolBar>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

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
        m_projectTree,
        &QTreeWidget::currentItemChanged,
        this,
        [this](QTreeWidgetItem* current, QTreeWidgetItem*) { HandleProjectTreeSelectionChanged(current); });

    if (m_requirementTreeItem != nullptr)
    {
        m_projectTree->setCurrentItem(m_requirementTreeItem);
    }

    statusBar()->showMessage(QStringLiteral("中央区域：最小 VTK 三维测试场景已接入。"));
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
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::NavigationRequested,
        this,
        [this](RoboSDP::Desktop::Ribbon::RibbonNavigationTarget target) {
            HandleRibbonNavigationRequested(target);
        });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::RibbonActionRequested,
        this,
        [this](const QString& actionName) {
            AppendLogLine(QStringLiteral("[Ribbon] 触发顶部功能区入口：%1").arg(actionName));
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
    auto* dock = new QDockWidget(QStringLiteral("项目树"), this);
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

    m_projectTree = new QTreeWidget(dock);
    m_projectTree->setHeaderLabel(QStringLiteral("项目骨架"));

    auto* projectRoot = new QTreeWidgetItem(QStringList{QStringLiteral("当前项目")});
    m_requirementTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("Requirement")});
    m_topologyTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("Topology")});
    m_kinematicsTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("Kinematics")});
    m_dynamicsTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("Dynamics")});
    m_selectionTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("Selection")});
    m_planningTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("Planning")});
    m_schemeTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("Scheme")});
    projectRoot->addChild(m_requirementTreeItem);
    projectRoot->addChild(m_topologyTreeItem);
    projectRoot->addChild(m_kinematicsTreeItem);
    projectRoot->addChild(m_dynamicsTreeItem);
    projectRoot->addChild(m_selectionTreeItem);
    projectRoot->addChild(m_planningTreeItem);
    projectRoot->addChild(m_schemeTreeItem);

    m_projectTree->addTopLevelItem(projectRoot);
    projectRoot->setExpanded(true);

    dock->setWidget(m_projectTree);
    addDockWidget(Qt::LeftDockWidgetArea, dock);
}

void MainWindow::CreatePropertyDock()
{
    auto* dock = new QDockWidget(QStringLiteral("属性面板"), this);
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

    m_propertyStack = new QStackedWidget(dock);

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

    m_propertyStack->addWidget(m_requirementWidget);
    m_propertyStack->addWidget(m_topologyWidget);
    m_propertyStack->addWidget(m_kinematicsWidget);
    m_propertyStack->addWidget(m_dynamicsWidget);
    m_propertyStack->addWidget(m_selectionWidget);
    m_propertyStack->addWidget(m_planningWidget);
    m_propertyStack->addWidget(m_schemeWidget);
    m_propertyStack->addWidget(m_placeholderPropertyPanel);
    dock->setWidget(m_propertyStack);
    addDockWidget(Qt::RightDockWidgetArea, dock);

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

    connect(
        m_kinematicsWidget,
        &RoboSDP::Kinematics::Ui::KinematicsWidget::LogMessageGenerated,
        this,
        [this](const QString& message) { AppendLogLine(message); });

    connect(
        m_kinematicsWidget,
        &RoboSDP::Kinematics::Ui::KinematicsWidget::TelemetryStatusGenerated,
        this,
        [this](const QString& message, bool warning) {
            ShowTelemetryStatus(QStringLiteral("Kinematics"), message, warning);
        });

    connect(
        m_kinematicsWidget,
        &RoboSDP::Kinematics::Ui::KinematicsWidget::UrdfPreviewSceneGenerated,
        this,
        [this](const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& scene) {
            if (m_robotVtkView != nullptr)
            {
                m_robotVtkView->ShowUrdfPreviewScene(scene);
            }

            if (scene.IsEmpty())
            {
                // 中文说明：模型结构切换时 Kinematics 会发送空场景，中央视图借此清理旧 URDF/Mesh 缓存。
                statusBar()->showMessage(QStringLiteral("中央区域：已清空 URDF 预览缓存，等待新的模型导入。"));
                AppendLogLine(QStringLiteral("[Desktop] URDF 预览缓存已清空。"));
                return;
            }

            const QString modelName = scene.model_name.isEmpty() ? QStringLiteral("未命名模型") : scene.model_name;
            statusBar()->showMessage(
                QStringLiteral("中央区域：已加载 URDF 骨架预览 - %1（节点 %2，连杆段 %3）")
                    .arg(modelName)
                    .arg(scene.nodes.size())
                    .arg(scene.segments.size()));
            AppendLogLine(
                QStringLiteral("[Desktop] URDF 骨架预览已同步到中央三维视图：%1").arg(modelName));
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

    if (m_ribbonBar != nullptr)
    {
        connect(
            m_ribbonBar,
            &RoboSDP::Desktop::Ribbon::RibbonBarWidget::ImportUrdfRequested,
            this,
            [this]() {
                if (m_kinematicsWidget == nullptr)
                {
                    AppendLogLine(QStringLiteral("[Ribbon][Warning] Kinematics 页面尚未就绪，无法导入 URDF。"));
                    return;
                }

                // 中文说明：Ribbon 只负责把操作意图转交给 Kinematics 页面，文件选择和 Service 编排仍保持原边界。
                m_kinematicsWidget->TriggerImportUrdf();
            });
    }

    connect(
        m_dynamicsWidget,
        &RoboSDP::Dynamics::Ui::DynamicsWidget::LogMessageGenerated,
        this,
        [this](const QString& message) { AppendLogLine(message); });

    if (m_ribbonBar != nullptr)
    {
        connect(
            m_ribbonBar,
            &RoboSDP::Desktop::Ribbon::RibbonBarWidget::RunDynamicsAnalysisRequested,
            this,
            [this]() {
                if (m_dynamicsWidget == nullptr)
                {
                    AppendLogLine(QStringLiteral("[Ribbon][Warning] Dynamics 页面尚未就绪，无法执行动力学分析。"));
                    return;
                }

                // 中文说明：Ribbon 只负责转交动作意图，Dynamics 页面继续负责参数校验、Service 编排与结果展示。
                m_dynamicsWidget->TriggerRunAnalysis();
            });
    }

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
    auto* dock = new QDockWidget(QStringLiteral("输出信息"), this);
    dock->setAllowedAreas(Qt::BottomDockWidgetArea);

    m_logPanel = new QPlainTextEdit(dock);
    m_logPanel->setReadOnly(true);

    dock->setWidget(m_logPanel);
    addDockWidget(Qt::BottomDockWidgetArea, dock);

    AppendLogLine(QStringLiteral("RoboSDP Desktop 启动完成。"));
    AppendLogLine(QStringLiteral("当前已接入 Requirement / Topology / Kinematics / Dynamics / Selection / Planning / Scheme 模块的最小页面。"));
}

void MainWindow::HandleProjectTreeSelectionChanged(QTreeWidgetItem* currentItem)
{
    if (currentItem == nullptr || m_propertyStack == nullptr)
    {
        return;
    }

    statusBar()->setStyleSheet(QString());

    if (currentItem == m_requirementTreeItem)
    {
        m_propertyStack->setCurrentWidget(m_requirementWidget);
        statusBar()->showMessage(QStringLiteral("当前页面：Requirement"));
        return;
    }

    if (currentItem == m_topologyTreeItem)
    {
        m_propertyStack->setCurrentWidget(m_topologyWidget);
        statusBar()->showMessage(QStringLiteral("当前页面：Topology"));
        return;
    }

    if (currentItem == m_kinematicsTreeItem)
    {
        m_propertyStack->setCurrentWidget(m_kinematicsWidget);
        statusBar()->showMessage(QStringLiteral("当前页面：Kinematics"));
        return;
    }

    if (currentItem == m_dynamicsTreeItem)
    {
        m_propertyStack->setCurrentWidget(m_dynamicsWidget);
        statusBar()->showMessage(QStringLiteral("当前页面：Dynamics"));
        return;
    }

    if (currentItem == m_selectionTreeItem)
    {
        m_propertyStack->setCurrentWidget(m_selectionWidget);
        statusBar()->showMessage(QStringLiteral("当前页面：Selection"));
        return;
    }

    if (currentItem == m_planningTreeItem)
    {
        m_propertyStack->setCurrentWidget(m_planningWidget);
        statusBar()->showMessage(QStringLiteral("当前页面：Planning"));
        return;
    }

    if (currentItem == m_schemeTreeItem)
    {
        m_propertyStack->setCurrentWidget(m_schemeWidget);
        statusBar()->showMessage(QStringLiteral("当前页面：Scheme"));
        return;
    }

    m_propertyStack->setCurrentWidget(m_placeholderPropertyPanel);
    statusBar()->showMessage(QStringLiteral("当前节点尚未实现编辑页面"));
}

void MainWindow::HandleRibbonNavigationRequested(
    RoboSDP::Desktop::Ribbon::RibbonNavigationTarget target)
{
    if (m_projectTree == nullptr)
    {
        return;
    }

    QTreeWidgetItem* targetItem = nullptr;
    switch (target)
    {
    case RoboSDP::Desktop::Ribbon::RibbonNavigationTarget::Requirement:
        targetItem = m_requirementTreeItem;
        break;
    case RoboSDP::Desktop::Ribbon::RibbonNavigationTarget::Topology:
        targetItem = m_topologyTreeItem;
        break;
    case RoboSDP::Desktop::Ribbon::RibbonNavigationTarget::Kinematics:
        targetItem = m_kinematicsTreeItem;
        break;
    case RoboSDP::Desktop::Ribbon::RibbonNavigationTarget::Dynamics:
        targetItem = m_dynamicsTreeItem;
        break;
    case RoboSDP::Desktop::Ribbon::RibbonNavigationTarget::Selection:
        targetItem = m_selectionTreeItem;
        break;
    case RoboSDP::Desktop::Ribbon::RibbonNavigationTarget::Planning:
        targetItem = m_planningTreeItem;
        break;
    case RoboSDP::Desktop::Ribbon::RibbonNavigationTarget::Scheme:
        targetItem = m_schemeTreeItem;
        break;
    }

    if (targetItem == nullptr)
    {
        AppendLogLine(QStringLiteral("[Ribbon][Warning] 顶部功能区导航目标尚未接入项目树。"));
        return;
    }

    // 中文说明：Ribbon 只发起导航请求，真正的属性页切换仍复用项目树现有逻辑，避免双份状态。
    m_projectTree->setCurrentItem(targetItem);
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

} // namespace RoboSDP::Desktop
