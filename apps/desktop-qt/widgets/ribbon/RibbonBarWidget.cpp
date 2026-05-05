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

    // 页签顺序：文件 → 需求校验 → 构型工具 → 运动学工具 → 动力学工具 → 视图 → 选型工具
    int index = 0;
    m_tabs->addTab(CreateFileTab(), QStringLiteral("文件"));
    m_moduleTabIndexMap.insert(QStringLiteral("File"), index++);

    m_tabs->insertTab(index, CreateRequirementTab(), QStringLiteral("需求校验"));
    m_moduleTabIndexMap.insert(QStringLiteral("Requirement"), index++);

    m_tabs->addTab(CreateTopologyTab(), QStringLiteral("构型工具"));
    m_moduleTabIndexMap.insert(QStringLiteral("Topology"), index++);

    m_tabs->addTab(CreateKinematicsTab(), QStringLiteral("运动学工具"));
    m_moduleTabIndexMap.insert(QStringLiteral("Kinematics"), index++);

    m_tabs->addTab(CreateDynamicsTab(), QStringLiteral("动力学工具"));
    m_moduleTabIndexMap.insert(QStringLiteral("Dynamics"), index++);

    m_tabs->addTab(CreateViewTab(), QStringLiteral("视图"));
    m_moduleTabIndexMap.insert(QStringLiteral("View"), index++);

    m_tabs->insertTab(index, CreateSelectionTab(), QStringLiteral("选型工具"));
    m_moduleTabIndexMap.insert(QStringLiteral("Selection"), index++);

    m_tabs->addTab(CreatePlanningTab(), QStringLiteral("规划工具"));
    m_moduleTabIndexMap.insert(QStringLiteral("Planning"), index++);

    m_tabs->addTab(CreateSchemeTab(), QStringLiteral("方案导出"));
    m_moduleTabIndexMap.insert(QStringLiteral("Scheme"), index++);

    rootLayout->addWidget(m_tabs);
    setMaximumHeight(128);
}

void RibbonBarWidget::SwitchToContextTab(const QString& moduleName)
{
    // 根据模块名快速定位并切换到对应的 Ribbon 页签
    auto it = m_moduleTabIndexMap.constFind(moduleName);
    if (it != m_moduleTabIndexMap.constEnd())
    {
        m_tabs->setCurrentIndex(it.value());
    }
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

QWidget* RibbonBarWidget::CreateTopologyTab()
{
    auto* tab = new QWidget(this);
    auto* layout = new QHBoxLayout(tab);
    layout->setContentsMargins(8, 4, 8, 6);
    layout->setSpacing(8);

    m_topologyRefreshBtn = CreateActionButton(
        QStringLiteral("刷新模板"),
        QStringLiteral("从模板库重新加载可用构型模板。"));
    connect(m_topologyRefreshBtn, &QToolButton::clicked, this, [this]() {
        emit signalTopologyRefreshTemplates();
    });

    m_topologyGenerateBtn = CreateActionButton(
        QStringLiteral("生成构型"),
        QStringLiteral("基于当前选中的模板和轴参数生成机器人构型。"));
    connect(m_topologyGenerateBtn, &QToolButton::clicked, this, [this]() {
        emit signalTopologyGenerate();
    });

    m_topologyValidateBtn = CreateActionButton(
        QStringLiteral("校验"),
        QStringLiteral("校验当前构型参数是否完整有效。"));
    connect(m_topologyValidateBtn, &QToolButton::clicked, this, [this]() {
        emit signalTopologyValidate();
    });

    m_topologySaveBtn = CreateActionButton(
        QStringLiteral("保存草稿"),
        QStringLiteral("将当前构型草稿保存到项目。"));
    connect(m_topologySaveBtn, &QToolButton::clicked, this, [this]() {
        emit signalTopologySaveDraft();
    });

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("构型操作"),
        { m_topologyRefreshBtn, m_topologyGenerateBtn, m_topologyValidateBtn, m_topologySaveBtn }));
    layout->addStretch();
    return tab;
}

QWidget* RibbonBarWidget::CreateKinematicsTab()
{
    auto* tab = new QWidget(this);
    auto* layout = new QHBoxLayout(tab);
    layout->setContentsMargins(8, 4, 8, 6);
    layout->setSpacing(8);

    m_kinematicsImportUrdfBtn = CreateActionButton(
        QStringLiteral("导入 URDF"),
        QStringLiteral("从文件系统导入 URDF 文件构建运动学模型。"));
    connect(m_kinematicsImportUrdfBtn, &QToolButton::clicked, this, [this]() {
        emit signalKinematicsImportUrdf();
    });

    m_kinematicsBuildFromTopologyBtn = CreateActionButton(
        QStringLiteral("从构型构建"),
        QStringLiteral("基于拓扑构型结果自动生成运动学模型。"));
    connect(m_kinematicsBuildFromTopologyBtn, &QToolButton::clicked, this, [this]() {
        emit signalKinematicsBuildFromTopology();
    });

    m_kinematicsPromoteDhBtn = CreateActionButton(
        QStringLiteral("DH 主模型"),
        QStringLiteral("提升当前 DH 草稿为主模型。"));
    connect(m_kinematicsPromoteDhBtn, &QToolButton::clicked, this, [this]() {
        emit signalKinematicsPromoteToDhMaster();
    });

    m_kinematicsSwitchUrdfBtn = CreateActionButton(
        QStringLiteral("URDF 主模型"),
        QStringLiteral("切换回 URDF 主模型。"));
    connect(m_kinematicsSwitchUrdfBtn, &QToolButton::clicked, this, [this]() {
        emit signalKinematicsSwitchToUrdfMaster();
    });

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("运动学建模"),
        { m_kinematicsImportUrdfBtn, m_kinematicsBuildFromTopologyBtn,
          m_kinematicsPromoteDhBtn, m_kinematicsSwitchUrdfBtn }));

    m_kinematicsRunFkBtn = CreateActionButton(
        QStringLiteral("正运动学"),
        QStringLiteral("基于当前关节角度执行正运动学求解。"));
    connect(m_kinematicsRunFkBtn, &QToolButton::clicked, this, [this]() {
        emit signalKinematicsRunFk();
    });

    m_kinematicsRunIkBtn = CreateActionButton(
        QStringLiteral("逆运动学"),
        QStringLiteral("基于末端位姿执行逆运动学求解。"));
    connect(m_kinematicsRunIkBtn, &QToolButton::clicked, this, [this]() {
        emit signalKinematicsRunIk();
    });

    m_kinematicsSampleWorkspaceBtn = CreateActionButton(
        QStringLiteral("采样工作空间"),
        QStringLiteral("采样并显示工作空间点云。"));
    connect(m_kinematicsSampleWorkspaceBtn, &QToolButton::clicked, this, [this]() {
        emit signalKinematicsSampleWorkspace();
    });

    m_kinematicsSaveBtn = CreateActionButton(
        QStringLiteral("保存草稿"),
        QStringLiteral("将当前运动学草稿保存到项目。"));
    connect(m_kinematicsSaveBtn, &QToolButton::clicked, this, [this]() {
        emit signalKinematicsSaveDraft();
    });

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("求解与验证"),
        { m_kinematicsRunFkBtn, m_kinematicsRunIkBtn,
          m_kinematicsSampleWorkspaceBtn, m_kinematicsSaveBtn }));
    layout->addStretch();
    return tab;
}

QWidget* RibbonBarWidget::CreateDynamicsTab()
{
    auto* tab = new QWidget(this);
    auto* layout = new QHBoxLayout(tab);
    layout->setContentsMargins(8, 4, 8, 6);
    layout->setSpacing(8);

    m_dynamicsBuildFromKinematicsBtn = CreateActionButton(
        QStringLiteral("从运动学构建"),
        QStringLiteral("基于运动学模型自动构建动力学参数。"));
    connect(m_dynamicsBuildFromKinematicsBtn, &QToolButton::clicked, this, [this]() {
        emit signalDynamicsBuildFromKinematics();
    });

    m_dynamicsRunAnalysisBtn = CreateActionButton(
        QStringLiteral("运行分析"),
        QStringLiteral("执行动力学分析计算。"));
    connect(m_dynamicsRunAnalysisBtn, &QToolButton::clicked, this, [this]() {
        emit signalDynamicsRunAnalysis();
    });

    m_dynamicsSaveBtn = CreateActionButton(
        QStringLiteral("保存草稿"),
        QStringLiteral("将当前动力学草稿保存到项目。"));
    connect(m_dynamicsSaveBtn, &QToolButton::clicked, this, [this]() {
        emit signalDynamicsSaveDraft();
    });

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("动力学分析"),
        { m_dynamicsBuildFromKinematicsBtn, m_dynamicsRunAnalysisBtn, m_dynamicsSaveBtn }));
    layout->addStretch();
    return tab;
}

QWidget* RibbonBarWidget::CreateRequirementTab()
{
    auto* tab = new QWidget(this);
    auto* layout = new QHBoxLayout(tab);
    layout->setContentsMargins(8, 4, 8, 6);
    layout->setSpacing(8);

    auto* validateBtn = CreateActionButton(
        QStringLiteral("执行校验"),
        QStringLiteral("校验当前需求参数是否完整有效。"));
    connect(validateBtn, &QToolButton::clicked, this, [this]() {
        emit signalRequirementValidate();
    });

    auto* saveBtn = CreateActionButton(
        QStringLiteral("保存本页"),
        QStringLiteral("将当前需求草稿保存到项目。"));
    connect(saveBtn, &QToolButton::clicked, this, [this]() {
        emit signalRequirementSaveDraft();
    });

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("需求校验"),
        { validateBtn, saveBtn }));
    layout->addStretch();
    return tab;
}

QWidget* RibbonBarWidget::CreateSelectionTab()
{
    auto* tab = new QWidget(this);
    auto* layout = new QHBoxLayout(tab);
    layout->setContentsMargins(8, 4, 8, 6);
    layout->setSpacing(8);

    auto* runBtn = CreateActionButton(
        QStringLiteral("执行选型"),
        QStringLiteral("基于当前参数执行驱动选型计算。"));
    connect(runBtn, &QToolButton::clicked, this, [this]() {
        emit signalSelectionRun();
    });

    auto* saveBtn = CreateActionButton(
        QStringLiteral("保存本页"),
        QStringLiteral("将当前选型草稿保存到项目。"));
    connect(saveBtn, &QToolButton::clicked, this, [this]() {
        emit signalSelectionSaveDraft();
    });

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("选型操作"),
        { runBtn, saveBtn }));
    layout->addStretch();
    return tab;
}

QWidget* RibbonBarWidget::CreatePlanningTab()
{
    auto* tab = new QWidget(this);
    auto* layout = new QHBoxLayout(tab);
    layout->setContentsMargins(8, 4, 8, 6);
    layout->setSpacing(8);

    auto* buildSceneBtn = CreateActionButton(
        QStringLiteral("构建场景"),
        QStringLiteral("基于当前运动学和动力学结果构建规划场景。"));
    connect(buildSceneBtn, &QToolButton::clicked, this, [this]() {
        emit signalPlanningBuildScene();
    });

    auto* runVerificationBtn = CreateActionButton(
        QStringLiteral("执行验证"),
        QStringLiteral("执行规划路径验证计算。"));
    connect(runVerificationBtn, &QToolButton::clicked, this, [this]() {
        emit signalPlanningRunVerification();
    });

    auto* saveBtn = CreateActionButton(
        QStringLiteral("保存本页"),
        QStringLiteral("将当前规划草稿保存到项目。"));
    connect(saveBtn, &QToolButton::clicked, this, [this]() {
        emit signalPlanningSaveDraft();
    });

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("规划验证"),
        { buildSceneBtn, runVerificationBtn, saveBtn }));
    layout->addStretch();
    return tab;
}

QWidget* RibbonBarWidget::CreateSchemeTab()
{
    auto* tab = new QWidget(this);
    auto* layout = new QHBoxLayout(tab);
    layout->setContentsMargins(8, 4, 8, 6);
    layout->setSpacing(8);

    auto* generateSnapshotBtn = CreateActionButton(
        QStringLiteral("生成快照"),
        QStringLiteral("生成当前方案快照。"));
    connect(generateSnapshotBtn, &QToolButton::clicked, this, [this]() {
        emit signalSchemeGenerateSnapshot();
    });

    auto* regenerateSaveBtn = CreateActionButton(
        QStringLiteral("重新生成并保存"),
        QStringLiteral("重新生成方案快照并保存到项目。"));
    connect(regenerateSaveBtn, &QToolButton::clicked, this, [this]() {
        emit signalSchemeRegenerateAndSave();
    });

    auto* loadSnapshotBtn = CreateActionButton(
        QStringLiteral("加载快照"),
        QStringLiteral("从项目加载已保存的方案快照。"));
    connect(loadSnapshotBtn, &QToolButton::clicked, this, [this]() {
        emit signalSchemeLoadSnapshot();
    });

    auto* exportJsonBtn = CreateActionButton(
        QStringLiteral("导出 JSON"),
        QStringLiteral("将当前方案导出为 JSON 文件。"));
    connect(exportJsonBtn, &QToolButton::clicked, this, [this]() {
        emit signalSchemeExportJson();
    });

    layout->addWidget(CreateButtonGroup(
        QStringLiteral("方案操作"),
        { generateSnapshotBtn, regenerateSaveBtn, loadSnapshotBtn, exportJsonBtn }));
    layout->addStretch();
    return tab;
}

// ── 按钮状态更新 ────────────────────────────────────────────────────

void RibbonBarWidget::SetTopologyButtonsEnabled(bool refreshEnabled, bool generateEnabled, bool validateEnabled, bool saveEnabled)
{
    if (m_topologyRefreshBtn) m_topologyRefreshBtn->setEnabled(refreshEnabled);
    if (m_topologyGenerateBtn) m_topologyGenerateBtn->setEnabled(generateEnabled);
    if (m_topologyValidateBtn) m_topologyValidateBtn->setEnabled(validateEnabled);
    if (m_topologySaveBtn) m_topologySaveBtn->setEnabled(saveEnabled);
}

void RibbonBarWidget::SetKinematicsButtonsEnabled(
    bool importUrdfEnabled,
    bool buildFromTopologyEnabled,
    bool promoteDhEnabled,
    bool switchUrdfEnabled,
    bool runFkEnabled,
    bool runIkEnabled,
    bool sampleWorkspaceEnabled,
    bool saveEnabled)
{
    if (m_kinematicsImportUrdfBtn) m_kinematicsImportUrdfBtn->setEnabled(importUrdfEnabled);
    if (m_kinematicsBuildFromTopologyBtn) m_kinematicsBuildFromTopologyBtn->setEnabled(buildFromTopologyEnabled);
    if (m_kinematicsPromoteDhBtn) m_kinematicsPromoteDhBtn->setEnabled(promoteDhEnabled);
    if (m_kinematicsSwitchUrdfBtn) m_kinematicsSwitchUrdfBtn->setEnabled(switchUrdfEnabled);
    if (m_kinematicsRunFkBtn) m_kinematicsRunFkBtn->setEnabled(runFkEnabled);
    if (m_kinematicsRunIkBtn) m_kinematicsRunIkBtn->setEnabled(runIkEnabled);
    if (m_kinematicsSampleWorkspaceBtn) m_kinematicsSampleWorkspaceBtn->setEnabled(sampleWorkspaceEnabled);
    if (m_kinematicsSaveBtn) m_kinematicsSaveBtn->setEnabled(saveEnabled);
}

void RibbonBarWidget::SetDynamicsButtonsEnabled(bool buildFromKinematicsEnabled, bool runAnalysisEnabled, bool saveEnabled)
{
    if (m_dynamicsBuildFromKinematicsBtn) m_dynamicsBuildFromKinematicsBtn->setEnabled(buildFromKinematicsEnabled);
    if (m_dynamicsRunAnalysisBtn) m_dynamicsRunAnalysisBtn->setEnabled(runAnalysisEnabled);
    if (m_dynamicsSaveBtn) m_dynamicsSaveBtn->setEnabled(saveEnabled);
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
