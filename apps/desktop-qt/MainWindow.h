#pragma once

#include "core/infrastructure/ProjectSaveCoordinator.h"

#include <QMainWindow>
#include <QString>

class QPlainTextEdit;
class QStackedWidget;
class QTreeWidget;
class QTreeWidgetItem;
class QWidget;

namespace RoboSDP::Desktop
{
namespace Vtk
{
class RobotVtkView;
}
namespace Ribbon
{
class RibbonBarWidget;
}
}

namespace RoboSDP::Requirement::Ui
{
class RequirementWidget;
}

namespace RoboSDP::Topology::Ui
{
class TopologyWidget;
}

namespace RoboSDP::Kinematics::Ui
{
class KinematicsWidget;
}

namespace RoboSDP::Dynamics::Ui
{
class DynamicsWidget;
}

namespace RoboSDP::Selection::Ui
{
class SelectionWidget;
}

namespace RoboSDP::Planning::Ui
{
class PlanningWidget;
}

namespace RoboSDP::Scheme::Ui
{
class SchemeWidget;
}

namespace RoboSDP::Desktop
{

/**
 * @brief 主窗口基础骨架。
 *
 * 当前窗口负责承载左侧项目树、右侧属性区、中央占位视图与底部日志区，
 * 并在当前阶段接入 Requirement / Topology / Kinematics / Dynamics / Selection / Planning / Scheme 七个最小业务页面。
 */
class MainWindow : public QMainWindow
{
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;

private:
    /// 构建主窗口基础界面，不承载复杂业务逻辑。
    void BuildUi();

    /// 创建顶部 Ribbon / Toolbar 功能区，只承载导航和高频入口，不直接执行复杂业务。
    void CreateRibbonBar();

    /// 创建中央最小三维视图，为后续业务对象接入保留承载容器。
    void CreateCentralView();

    /// 创建左侧项目树导航区域。
    void CreateProjectTreeDock();

    /// 创建右侧属性面板，并挂接 Requirement / Topology / Kinematics / Dynamics / Selection / Planning / Scheme 页面。
    void CreatePropertyDock();

    /// 创建底部日志面板。
    void CreateLogDock();

    /// 根据项目树选中节点切换属性页。
    void HandleProjectTreeSelectionChanged(QTreeWidgetItem* currentItem);

    /// 响应顶部功能区导航请求，统一复用左侧项目树和右侧属性页切换逻辑。
    void HandleRibbonNavigateTo(QTreeWidgetItem* targetItem, const QString& targetName);

    /// 响应顶部功能区“新建项目”命令，创建最小项目目录骨架。
    void HandleCreateNewProjectRequested();

    /// 响应顶部功能区“打开项目”命令，校验 project.json 后更新全局项目上下文。
    void HandleOpenProjectRequested();

    /// 响应顶部功能区“保存”命令，执行全局保存编排。
    void HandleGlobalSaveRequested();

    /// 响应全局项目上下文变化，刷新主窗口项目树、状态栏和日志。
    void HandleProjectContextPathChanged(const QString& projectRootPath);

    /// 将底层引擎状态遥测同步到状态栏，warning 为 true 时使用告警样式。
    void ShowTelemetryStatus(const QString& moduleName, const QString& message, bool warning);

    /// 向底部日志面板追加一条消息。
    void AppendLogLine(const QString& message);

private:
    RoboSDP::Desktop::Ribbon::RibbonBarWidget* m_ribbonBar = nullptr;
    RoboSDP::Desktop::Vtk::RobotVtkView* m_robotVtkView = nullptr;
    QTreeWidget* m_projectTree = nullptr;
    QTreeWidgetItem* m_projectRootTreeItem = nullptr;
    QTreeWidgetItem* m_requirementTreeItem = nullptr;
    QTreeWidgetItem* m_topologyTreeItem = nullptr;
    QTreeWidgetItem* m_kinematicsTreeItem = nullptr;
    QTreeWidgetItem* m_dynamicsTreeItem = nullptr;
    QTreeWidgetItem* m_selectionTreeItem = nullptr;
    QTreeWidgetItem* m_planningTreeItem = nullptr;
    QTreeWidgetItem* m_schemeTreeItem = nullptr;
    QStackedWidget* m_propertyStack = nullptr;
    QPlainTextEdit* m_placeholderPropertyPanel = nullptr;
    RoboSDP::Requirement::Ui::RequirementWidget* m_requirementWidget = nullptr;
    RoboSDP::Topology::Ui::TopologyWidget* m_topologyWidget = nullptr;
    RoboSDP::Kinematics::Ui::KinematicsWidget* m_kinematicsWidget = nullptr;
    RoboSDP::Dynamics::Ui::DynamicsWidget* m_dynamicsWidget = nullptr;
    RoboSDP::Selection::Ui::SelectionWidget* m_selectionWidget = nullptr;
    RoboSDP::Planning::Ui::PlanningWidget* m_planningWidget = nullptr;
    RoboSDP::Scheme::Ui::SchemeWidget* m_schemeWidget = nullptr;
    QPlainTextEdit* m_logPanel = nullptr;
    RoboSDP::Infrastructure::ProjectSaveCoordinator m_projectSaveCoordinator;
};

} // namespace RoboSDP::Desktop
