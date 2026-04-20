#include "apps/desktop-qt/MainWindow.h"

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
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalNavigateToRequirement,
        this,
        [this]() { HandleRibbonNavigateTo(m_requirementTreeItem, QStringLiteral("需求定义")); });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalNavigateToTopology,
        this,
        [this]() { HandleRibbonNavigateTo(m_topologyTreeItem, QStringLiteral("构型设计")); });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalNavigateToKinematics,
        this,
        [this]() { HandleRibbonNavigateTo(m_kinematicsTreeItem, QStringLiteral("运动学建模")); });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalNavigateToDynamics,
        this,
        [this]() { HandleRibbonNavigateTo(m_dynamicsTreeItem, QStringLiteral("动力学分析")); });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalNavigateToSelection,
        this,
        [this]() { HandleRibbonNavigateTo(m_selectionTreeItem, QStringLiteral("驱动选型")); });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalNavigateToPlanning,
        this,
        [this]() { HandleRibbonNavigateTo(m_planningTreeItem, QStringLiteral("规划与分析")); });
    connect(
        m_ribbonBar,
        &RoboSDP::Desktop::Ribbon::RibbonBarWidget::signalNavigateToScheme,
        this,
        [this]() { HandleRibbonNavigateTo(m_schemeTreeItem, QStringLiteral("结果与导出")); });
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

    m_projectRootTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("当前项目")});
    m_requirementTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("Requirement")});
    m_topologyTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("Topology")});
    m_kinematicsTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("Kinematics")});
    m_dynamicsTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("Dynamics")});
    m_selectionTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("Selection")});
    m_planningTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("Planning")});
    m_schemeTreeItem = new QTreeWidgetItem(QStringList{QStringLiteral("Scheme")});
    m_projectRootTreeItem->addChild(m_requirementTreeItem);
    m_projectRootTreeItem->addChild(m_topologyTreeItem);
    m_projectRootTreeItem->addChild(m_kinematicsTreeItem);
    m_projectRootTreeItem->addChild(m_dynamicsTreeItem);
    m_projectRootTreeItem->addChild(m_selectionTreeItem);
    m_projectRootTreeItem->addChild(m_planningTreeItem);
    m_projectRootTreeItem->addChild(m_schemeTreeItem);

    m_projectTree->addTopLevelItem(m_projectRootTreeItem);
    m_projectRootTreeItem->setExpanded(true);

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

    connect(
        m_dynamicsWidget,
        &RoboSDP::Dynamics::Ui::DynamicsWidget::LogMessageGenerated,
        this,
        [this](const QString& message) { AppendLogLine(message); });

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

void MainWindow::HandleRibbonNavigateTo(QTreeWidgetItem* targetItem, const QString& targetName)
{
    if (m_projectTree == nullptr)
    {
        return;
    }

    if (targetItem == nullptr)
    {
        AppendLogLine(QStringLiteral("[WARN] [顶部功能区] 导航目标尚未接入项目树：%1").arg(targetName));
        return;
    }

    // 中文说明：Ribbon 只发起导航请求，真正的属性页切换仍复用项目树现有逻辑，避免双份状态。
    AppendLogLine(QStringLiteral("[INFO] [顶部功能区] 用户请求切换至%1").arg(targetName));
    m_projectTree->setCurrentItem(targetItem);
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
        AppendLogLine(
            QStringLiteral("%1 [全局保存] %2：%3")
                .arg(itemResult.success ? QStringLiteral("[INFO]") : QStringLiteral("[ERROR]"),
                     itemResult.module_name,
                     itemResult.message));
    }

    AppendLogLine(QStringLiteral("%1 [全局保存] %2")
        .arg(summary.success ? QStringLiteral("[INFO]") : QStringLiteral("[ERROR]"), summary.message));
    statusBar()->setStyleSheet(
        summary.success
            ? QStringLiteral("QStatusBar{color:#1b7f3b;}")
            : QStringLiteral("QStatusBar{color:#b54708;font-weight:600;}"));
    statusBar()->showMessage(summary.message);
}

void MainWindow::HandleProjectContextPathChanged(const QString& projectRootPath)
{
    if (projectRootPath.trimmed().isEmpty())
    {
        return;
    }

    const QFileInfo projectRootInfo(projectRootPath);
    if (m_projectRootTreeItem != nullptr)
    {
        // 中文说明：项目树只展示 ProjectManager 中的当前项目上下文摘要，不保存独立项目状态。
        m_projectRootTreeItem->setText(
            0,
            QStringLiteral("%1 (%2)")
                .arg(projectRootInfo.fileName(), QDir::toNativeSeparators(projectRootInfo.absoluteFilePath())));
        m_projectRootTreeItem->setExpanded(true);
    }

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

} // namespace RoboSDP::Desktop
