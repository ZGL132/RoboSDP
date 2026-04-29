#include "apps/desktop-qt/widgets/ribbon/RibbonBarWidget.h"

#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QTabWidget>
#include <QToolButton>
#include <QVBoxLayout>

namespace RoboSDP::Desktop::Ribbon
{

RibbonBarWidget::RibbonBarWidget(QWidget* parent)
    : QWidget(parent)
{
    BuildUi();
}

void RibbonBarWidget::BuildUi()
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(0, 0, 0, 0);
    rootLayout->setSpacing(0);

    m_tabs = new QTabWidget(this);
    m_tabs->setDocumentMode(true);
    m_tabs->setTabPosition(QTabWidget::North);
    m_tabs->addTab(CreateFileTab(), QStringLiteral("文件"));
    m_tabs->addTab(CreateModelingTab(), QStringLiteral("建模"));
    m_tabs->addTab(CreateDynamicsTab(), QStringLiteral("动力学"));
    m_tabs->addTab(CreateDriveSelectionTab(), QStringLiteral("驱动选型"));
    m_tabs->addTab(CreatePlanningAnalysisTab(), QStringLiteral("规划与分析"));
    m_tabs->addTab(CreateResultExportTab(), QStringLiteral("结果与导出"));
    m_tabs->addTab(CreateViewTab(), QStringLiteral("视图"));

    rootLayout->addWidget(m_tabs);
    setMaximumHeight(128);
}

QWidget* RibbonBarWidget::CreateFileTab()
{
    auto* tab = new QWidget(this);
    auto* layout = new QHBoxLayout(tab);
    layout->setContentsMargins(8, 4, 8, 6);
    layout->setSpacing(8);

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("项目文件"),
        {
            CreateNewProjectButton(),
            CreateOpenProjectButton(),
            CreateGlobalSaveButton(),
            CreateDisabledButton(QStringLiteral("导出 JSON"), QStringLiteral("等待统一导出服务接入，请暂用 Scheme 页面内部导出能力。")),
        }));
    layout->addStretch();
    return tab;
}

QWidget* RibbonBarWidget::CreateModelingTab()
{
    auto* tab = new QWidget(this);
    auto* layout = new QHBoxLayout(tab);
    layout->setContentsMargins(8, 4, 8, 6);
    layout->setSpacing(8);

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("需求与构型"),
        {
            CreateNavigationButton(
                QStringLiteral("需求定义"),
                QStringLiteral("切换到 Requirement 页面。"),
                &RibbonBarWidget::signalNavigateToRequirement),
            CreateNavigationButton(
                QStringLiteral("构型设计"),
                QStringLiteral("切换到 Topology 页面。"),
                &RibbonBarWidget::signalNavigateToTopology),
        }));
    layout->addWidget(CreateButtonGroup(
        QStringLiteral("运动学建模"),
        {
            CreateNavigationButton(
                QStringLiteral("DH/MDH 建模"),
                QStringLiteral("切换到 Kinematics 页面编辑 DH/MDH 参数。"),
                &RibbonBarWidget::signalNavigateToKinematics),
            CreateNavigationButton(
                QStringLiteral("导入 URDF"),
                QStringLiteral("切换到 Kinematics 页面，下一阶段接入项目级 URDF 导入服务。"),
                &RibbonBarWidget::signalNavigateToKinematics),
        }));
    layout->addStretch();
    return tab;
}

QWidget* RibbonBarWidget::CreateDynamicsTab()
{
    auto* tab = new QWidget(this);
    auto* layout = new QHBoxLayout(tab);
    layout->setContentsMargins(8, 4, 8, 6);
    layout->setSpacing(8);

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("动力学分析"),
        {
            CreateNavigationButton(
                QStringLiteral("动力学分析"),
                QStringLiteral("切换到 Dynamics 页面。"),
                &RibbonBarWidget::signalNavigateToDynamics),
        }));
    layout->addStretch();
    return tab;
}

QWidget* RibbonBarWidget::CreateDriveSelectionTab()
{
    auto* tab = new QWidget(this);
    auto* layout = new QHBoxLayout(tab);
    layout->setContentsMargins(8, 4, 8, 6);
    layout->setSpacing(8);

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("驱动链"),
        {
            CreateDisabledButton(QStringLiteral("电机选型"), QStringLiteral("二期建设中，等待电机库与项目级选型服务接入。")),
            CreateDisabledButton(QStringLiteral("减速器选型"), QStringLiteral("二期建设中，等待减速器库与项目级选型服务接入。")),
            CreateDisabledButton(QStringLiteral("驱动链匹配"), QStringLiteral("等待驱动链匹配服务纳入顶部全局编排。")),
        }));
    layout->addStretch();
    return tab;
}

QWidget* RibbonBarWidget::CreatePlanningAnalysisTab()
{
    auto* tab = new QWidget(this);
    auto* layout = new QHBoxLayout(tab);
    layout->setContentsMargins(8, 4, 8, 6);
    layout->setSpacing(8);

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("规划验证"),
        {
            CreateNavigationButton(
                QStringLiteral("规划场景"),
                QStringLiteral("切换到 Planning 页面编辑规划场景。"),
                &RibbonBarWidget::signalNavigateToPlanning),
            CreateNavigationButton(
                QStringLiteral("路径规划"),
                QStringLiteral("切换到 Planning 页面执行最小规划验证。"),
                &RibbonBarWidget::signalNavigateToPlanning),
            CreateNavigationButton(
                QStringLiteral("碰撞检测"),
                QStringLiteral("切换到 Planning 页面查看碰撞检测结果表。"),
                &RibbonBarWidget::signalNavigateToPlanning),
        }));
    layout->addStretch();
    return tab;
}

QWidget* RibbonBarWidget::CreateResultExportTab()
{
    auto* tab = new QWidget(this);
    auto* layout = new QHBoxLayout(tab);
    layout->setContentsMargins(8, 4, 8, 6);
    layout->setSpacing(8);

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("方案结果"),
        {
            CreateNavigationButton(
                QStringLiteral("方案快照"),
                QStringLiteral("切换到 Scheme 页面生成方案快照。"),
                &RibbonBarWidget::signalNavigateToScheme),
            CreateNavigationButton(
                QStringLiteral("导出结果"),
                QStringLiteral("切换到 Scheme 页面执行结果导出。"),
                &RibbonBarWidget::signalNavigateToScheme),
        }));
    layout->addStretch();
    return tab;
}

QWidget* RibbonBarWidget::CreateViewTab()
{
    auto* tab = new QWidget(this);
    auto* layout = new QHBoxLayout(tab);
    layout->setContentsMargins(8, 4, 8, 6);
    layout->setSpacing(8);

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("三维显示"),
        {
            CreateToggleButton(
                QStringLiteral("显示骨架"),
                QStringLiteral("显示 link 节点、joint 高亮和父子连杆线。"),
                true,
                &RibbonBarWidget::signalSetSkeletonVisible),
            CreateToggleButton(
                QStringLiteral("Visual"),
                QStringLiteral("显示 URDF visual mesh，用于检查外观几何与骨架是否重合。"),
                true,
                &RibbonBarWidget::signalSetVisualMeshVisible),
            CreateToggleButton(
                QStringLiteral("Collision"),
                QStringLiteral("以半透明线框显示 URDF collision mesh；碰撞计算仍在后续阶段接入。"),
                false,
                &RibbonBarWidget::signalSetCollisionMeshVisible),
            CreateToggleButton(
                QStringLiteral("关节轴"),
                QStringLiteral("显示 URDF joint axis 箭头，方向随当前 FK 姿态更新。"),
                true,
                &RibbonBarWidget::signalSetJointAxesVisible),
        }));

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("标注与相机"),
        {
            CreateToggleButton(
                QStringLiteral("坐标系"),
                QStringLiteral("显示中央三维视图中的世界坐标系。"),
                true,
                &RibbonBarWidget::signalSetAxesVisible),
            CreateToggleButton(
                QStringLiteral("地面网格"),
                QStringLiteral("显示或隐藏中央三维视图中的 Z=0 地面网格参考平面。"),
                true,
                &RibbonBarWidget::signalSetGroundGridVisible),
            CreateToggleButton(
                QStringLiteral("角落坐标轴"),
                QStringLiteral("显示或隐藏屏幕角落的小型方向坐标轴，用于替代大世界坐标轴判断方向。"),
                true,
                &RibbonBarWidget::signalSetCornerAxesVisible),
            CreateToggleButton(
                QStringLiteral("Link 标签"),
                QStringLiteral("显示 URDF Link 名称标签。"),
                true,
                &RibbonBarWidget::signalSetLinkLabelsVisible),
            CreateToggleButton(
                QStringLiteral("Joint 标签"),
                QStringLiteral("显示 URDF Joint 名称标签。"),
                true,
                &RibbonBarWidget::signalSetJointLabelsVisible),
            CreateResetCameraButton(),
        }));

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("相机视角"),
        {
            CreateCameraPresetButton(
                QStringLiteral("正视图"),
                QStringLiteral("沿 -Y 方向观察模型，Z 轴向上。"),
                &RibbonBarWidget::signalCameraFrontViewRequested),
            CreateCameraPresetButton(
                QStringLiteral("侧视图"),
                QStringLiteral("沿 +X 方向观察模型，Z 轴向上。"),
                &RibbonBarWidget::signalCameraSideViewRequested),
            CreateCameraPresetButton(
                QStringLiteral("俯视图"),
                QStringLiteral("沿 +Z 方向俯视模型，Y 轴向上。"),
                &RibbonBarWidget::signalCameraTopViewRequested),
            CreateCameraPresetButton(
                QStringLiteral("等轴测"),
                QStringLiteral("从 (1, -1, 1) 方向观察模型，适合检查整体结构。"),
                &RibbonBarWidget::signalCameraIsometricViewRequested),
        }));

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("页面布置"),
        {
            CreateToggleButton(
                QStringLiteral("项目树"),
                QStringLiteral("显示或隐藏左侧项目树导航面板。"),
                true,
                &RibbonBarWidget::signalSetProjectTreeVisible),
            CreateToggleButton(
                QStringLiteral("属性面板"),
                QStringLiteral("显示或隐藏右侧业务属性面板。"),
                true,
                &RibbonBarWidget::signalSetPropertyPanelVisible),
            CreateToggleButton(
                QStringLiteral("输出信息"),
                QStringLiteral("显示或隐藏底部输出信息面板。"),
                true,
                &RibbonBarWidget::signalSetOutputPanelVisible),
            CreateDisabledButton(QStringLiteral("布局恢复"), QStringLiteral("等待统一窗口布局持久化服务接入。")),
        }));

    layout->addStretch();
    return tab;
}

QToolButton* RibbonBarWidget::CreateActionButton(const QString& text, const QString& tooltip)
{
    auto* button = new QToolButton(this);
    button->setText(text);
    button->setToolTip(tooltip);
    button->setMinimumWidth(92);
    button->setMinimumHeight(44);
    button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    button->setEnabled(true);
    return button;
}

QToolButton* RibbonBarWidget::CreateToggleButton(
    const QString& text,
    const QString& tooltip,
    bool checked,
    void (RibbonBarWidget::*toggleSignal)(bool))
{
    auto* button = CreateActionButton(text, tooltip);
    button->setCheckable(true);
    button->setChecked(checked);
    connect(button, &QToolButton::toggled, this, [this, toggleSignal](bool visible) {
        // 中文说明：视图页签只发布显示状态变化，不直接接触 VTK Actor 或 DockWidget 细节。
        (this->*toggleSignal)(visible);
    });
    return button;
}

QToolButton* RibbonBarWidget::CreateNewProjectButton()
{
    auto* button = CreateActionButton(
        QStringLiteral("新建项目"),
        QStringLiteral("选择项目根目录并创建最小项目骨架。"));
    connect(button, &QToolButton::clicked, this, [this]() {
        // 中文说明：Ribbon 只发出项目级命令，目录选择、服务调用和状态同步交给 MainWindow。
        emit signalCreateNewProject();
    });
    return button;
}

QToolButton* RibbonBarWidget::CreateOpenProjectButton()
{
    auto* button = CreateActionButton(
        QStringLiteral("打开项目"),
        QStringLiteral("选择包含 project.json 的 RoboSDP 项目目录。"));
    connect(button, &QToolButton::clicked, this, [this]() {
        // 中文说明：Ribbon 只发出打开项目命令，项目合法性校验由 MainWindow 与 ProjectManager 协作完成。
        emit signalOpenProject();
    });
    return button;
}

QToolButton* RibbonBarWidget::CreateGlobalSaveButton()
{
    auto* button = CreateActionButton(
        QStringLiteral("保存"),
        QStringLiteral("保存当前项目中所有已接入模块的草稿。"));
    connect(button, &QToolButton::clicked, this, [this]() {
        // 中文说明：Ribbon 只发出全局保存命令，具体保存编排由 MainWindow / ProjectSaveCoordinator 处理。
        emit signalGlobalSaveRequested();
    });
    return button;
}

QToolButton* RibbonBarWidget::CreateResetCameraButton()
{
    auto* button = CreateActionButton(
        QStringLiteral("重置相机"),
        QStringLiteral("将相机重新对准当前预览场景。"));
    connect(button, &QToolButton::clicked, this, [this]() {
        // 中文说明：相机操作属于全局视图命令，由 MainWindow 转交中央三维视图执行。
        emit signalResetCameraRequested();
    });
    return button;
}

QToolButton* RibbonBarWidget::CreateCameraPresetButton(
    const QString& text,
    const QString& tooltip,
    void (RibbonBarWidget::*cameraSignal)())
{
    auto* button = CreateActionButton(text, tooltip);
    connect(button, &QToolButton::clicked, this, [this, cameraSignal]() {
        // 中文说明：相机预设只表达视角意图，具体 VTK camera 设置由中央视图封装。
        (this->*cameraSignal)();
    });
    return button;
}

QToolButton* RibbonBarWidget::CreateNavigationButton(
    const QString& text,
    const QString& tooltip,
    void (RibbonBarWidget::*navigationSignal)())
{
    auto* button = CreateActionButton(text, tooltip);
    connect(button, &QToolButton::clicked, this, [this, navigationSignal]() {
        // 中文说明：顶部功能区只表达导航意图，具体页面状态切换由 MainWindow 统一处理。
        (this->*navigationSignal)();
    });
    return button;
}

QToolButton* RibbonBarWidget::CreateDisabledButton(const QString& text, const QString& reason)
{
    auto* button = CreateActionButton(text, reason);
    button->setEnabled(false);
    return button;
}

QWidget* RibbonBarWidget::CreateButtonGroup(const QString& title, const QList<QToolButton*>& buttons)
{
    auto* group = new QFrame(this);
    group->setFrameShape(QFrame::StyledPanel);
    group->setObjectName(QStringLiteral("ribbon_button_group"));

    auto* layout = new QVBoxLayout(group);
    layout->setContentsMargins(6, 4, 6, 4);
    layout->setSpacing(4);

    auto* buttonLayout = new QHBoxLayout();
    buttonLayout->setSpacing(4);
    for (QToolButton* button : buttons)
    {
        buttonLayout->addWidget(button);
    }

    auto* titleLabel = new QLabel(title, group);
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setStyleSheet(QStringLiteral("color:#475467;font-size:11px;"));

    layout->addLayout(buttonLayout);
    layout->addWidget(titleLabel);
    return group;
}

} // namespace RoboSDP::Desktop::Ribbon
