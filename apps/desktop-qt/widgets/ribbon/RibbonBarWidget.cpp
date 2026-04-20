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
            CreateDisabledButton(QStringLiteral("新建项目"), QStringLiteral("统一项目级新建流程尚未接入，本轮先冻结入口。")),
            CreateDisabledButton(QStringLiteral("打开项目"), QStringLiteral("统一项目级打开流程尚未接入，请暂用各模块项目目录输入框。")),
            CreateDisabledButton(QStringLiteral("保存"), QStringLiteral("统一项目级保存尚未接入，请暂用右侧模块保存草稿。")),
        }));
    layout->addWidget(CreateButtonGroup(
        QStringLiteral("导入 / 导出"),
        {
            CreateImportUrdfButton(
                QStringLiteral("导入 URDF"),
                QStringLiteral("切换到 Kinematics 页面并打开 URDF 文件选择。")),
            CreateDisabledButton(QStringLiteral("导出 JSON"), QStringLiteral("统一导出入口尚未接入，后续由 Scheme / Export 模块承载。")),
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
            CreateNavigationButton(QStringLiteral("需求定义"), QStringLiteral("切换到 Requirement 页面。"), RibbonNavigationTarget::Requirement),
            CreateNavigationButton(QStringLiteral("构型设计"), QStringLiteral("切换到 Topology 页面。"), RibbonNavigationTarget::Topology),
        }));
    layout->addWidget(CreateButtonGroup(
        QStringLiteral("运动学建模"),
        {
            CreateNavigationButton(QStringLiteral("DH/MDH 建模"), QStringLiteral("切换到 Kinematics 页面编辑 DH/MDH 参数。"), RibbonNavigationTarget::Kinematics),
            CreateImportUrdfButton(QStringLiteral("URDF 导入"), QStringLiteral("切换到 Kinematics 页面并打开 URDF 文件选择。")),
            CreateDisabledButton(QStringLiteral("坐标系定义"), QStringLiteral("坐标系定义当前位于 Kinematics 右侧属性面板，本轮不新增弹窗。")),
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
            CreateNavigationButton(QStringLiteral("质量/惯量定义"), QStringLiteral("切换到 Dynamics 页面。"), RibbonNavigationTarget::Dynamics),
            CreateNavigationButton(QStringLiteral("工况轨迹"), QStringLiteral("切换到 Dynamics 页面编辑轨迹输入。"), RibbonNavigationTarget::Dynamics),
            CreateRunDynamicsAnalysisButton(QStringLiteral("执行动力学"), QStringLiteral("切换到 Dynamics 页面并执行逆动力学分析。")),
            CreateDisabledButton(QStringLiteral("负载包络"), QStringLiteral("负载包络详情入口后续接入结果详情窗口。")),
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
            CreateNavigationButton(QStringLiteral("电机选型"), QStringLiteral("切换到 Selection 页面。"), RibbonNavigationTarget::Selection),
            CreateNavigationButton(QStringLiteral("减速器选型"), QStringLiteral("切换到 Selection 页面。"), RibbonNavigationTarget::Selection),
            CreateNavigationButton(QStringLiteral("驱动链匹配"), QStringLiteral("切换到 Selection 页面执行联合匹配。"), RibbonNavigationTarget::Selection),
            CreateDisabledButton(QStringLiteral("抱闸校核"), QStringLiteral("抱闸校核属于后续细化能力，本轮暂不接入。")),
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
            CreateNavigationButton(QStringLiteral("规划场景"), QStringLiteral("切换到 Planning 页面。"), RibbonNavigationTarget::Planning),
            CreateNavigationButton(QStringLiteral("路径规划"), QStringLiteral("切换到 Planning 页面执行最小规划验证。"), RibbonNavigationTarget::Planning),
            CreateDisabledButton(QStringLiteral("碰撞检测"), QStringLiteral("HPP-FCL 碰撞检测内核尚未接入，本轮先冻结入口。")),
            CreateDisabledButton(QStringLiteral("自碰撞检测"), QStringLiteral("自碰撞检测将在碰撞内核阶段接入。")),
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
            CreateNavigationButton(QStringLiteral("方案快照"), QStringLiteral("切换到 Scheme 页面生成快照。"), RibbonNavigationTarget::Scheme),
            CreateNavigationButton(QStringLiteral("结果摘要"), QStringLiteral("切换到 Scheme 页面查看当前摘要。"), RibbonNavigationTarget::Scheme),
            CreateDisabledButton(QStringLiteral("结果对比"), QStringLiteral("结果对比窗口尚未接入。")),
            CreateDisabledButton(QStringLiteral("导出结果"), QStringLiteral("统一结果导出入口后续接入。")),
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
    connect(button, &QToolButton::clicked, this, [this, text]() {
        emit RibbonActionRequested(text);
    });
    return button;
}

QToolButton* RibbonBarWidget::CreateNavigationButton(
    const QString& text,
    const QString& tooltip,
    RoboSDP::Desktop::Ribbon::RibbonNavigationTarget target)
{
    auto* button = CreateActionButton(text, tooltip);
    disconnect(button, nullptr, this, nullptr);
    connect(button, &QToolButton::clicked, this, [this, target, text]() {
        emit RibbonActionRequested(text);
        emit NavigationRequested(target);
    });
    return button;
}

QToolButton* RibbonBarWidget::CreateImportUrdfButton(const QString& text, const QString& tooltip)
{
    auto* button = CreateActionButton(text, tooltip);
    disconnect(button, nullptr, this, nullptr);
    connect(button, &QToolButton::clicked, this, [this, text]() {
        // 中文说明：Ribbon 只发出导入意图，文件选择和 URDF 解析仍由 KinematicsWidget / Service 承担。
        emit RibbonActionRequested(text);
        emit NavigationRequested(RibbonNavigationTarget::Kinematics);
        emit ImportUrdfRequested();
    });
    return button;
}

QToolButton* RibbonBarWidget::CreateRunDynamicsAnalysisButton(const QString& text, const QString& tooltip)
{
    auto* button = CreateActionButton(text, tooltip);
    disconnect(button, nullptr, this, nullptr);
    connect(button, &QToolButton::clicked, this, [this, text]() {
        // 中文说明：Ribbon 只发起动力学分析意图，表格校验、Service 调用和图表刷新仍由 DynamicsWidget 承担。
        emit RibbonActionRequested(text);
        emit NavigationRequested(RibbonNavigationTarget::Dynamics);
        emit RunDynamicsAnalysisRequested();
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
