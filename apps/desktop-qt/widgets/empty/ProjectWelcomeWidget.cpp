#include "apps/desktop-qt/widgets/empty/ProjectWelcomeWidget.h"

#include <QApplication>
#include <QBoxLayout>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QFrame>
#include <QGraphicsDropShadowEffect>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace
{

QLabel* CreateTextLabel(const QString& text, const QString& styleSheet, QWidget* parent)
{
    auto* label = new QLabel(text, parent);
    label->setWordWrap(true);
    label->setStyleSheet(styleSheet);
    return label;
}

int ScaledInt(int baseValue, double scale)
{
    return std::max(1, static_cast<int>(baseValue * scale + 0.5));
}

void SetLabelFontSize(QLabel* label, int basePixelSize, double scale)
{
    if (label == nullptr)
    {
        return;
    }

    auto font = label->font();
    font.setPixelSize(ScaledInt(basePixelSize, scale));
    label->setFont(font);
}

void SetButtonFontSize(QPushButton* button, int basePixelSize, double scale)
{
    if (button == nullptr)
    {
        return;
    }

    auto font = button->font();
    font.setPixelSize(ScaledInt(basePixelSize, scale));
    button->setFont(font);
}

QString CreateHeroCardStyleSheet(double scale)
{
    return QStringLiteral(
        "QFrame#welcomeHeroCard{"
        "background:#f8fafc;"
        "border:%1px solid #475569;"
        "border-top:%2px solid #334155;"
        "border-radius:0px;"
        "}").arg(ScaledInt(4, scale)).arg(ScaledInt(16, scale));
}

QString CreateWorkflowCardStyleSheet(double scale)
{
    return QStringLiteral(
        "QFrame#workflowCard{"
        "background:#ecfeff;"
        "border:%1px solid #0f766e;"
        "border-left:%2px solid #f97316;"
        "border-radius:0px;"
        "}").arg(ScaledInt(4, scale)).arg(ScaledInt(12, scale));
}

QString CreateRecentCardStyleSheet(double scale)
{
    return QStringLiteral(
        "QFrame#recentProjectCard{"
        "background:#f8fafc;"
        "border:%1px solid #64748b;"
        "border-left:%2px solid #0f766e;"
        "border-radius:0px;"
        "}").arg(ScaledInt(4, scale)).arg(ScaledInt(12, scale));
}

QString CreateRecentEmptyStyleSheet(double scale)
{
    return QStringLiteral(
        "border-radius:0px;"
        "background:#e2e8f0;"
        "border:%1px solid #94a3b8;"
        "padding:%2px;"
        "color:#475569;"
        "font-weight:700;").arg(ScaledInt(2, scale)).arg(ScaledInt(48, scale));
}

QString CreateRecentProjectButtonStyleSheet(double scale)
{
    return QStringLiteral(
        "QPushButton#recentProjectButton{"
        "background:#ffffff;"
        "border:%1px solid #cbd5e1;"
        "border-left:%2px solid #0f766e;"
        "border-radius:0px;"
        "color:#0f172a;"
        "font-weight:700;"
        "padding:%3px %4px;"
        "text-align:left;"
        "}"
        "QPushButton#recentProjectButton:hover{"
        "background:#f1f5f9;"
        "border-color:#0f766e;"
        "}").arg(ScaledInt(2, scale)).arg(ScaledInt(8, scale)).arg(ScaledInt(18, scale)).arg(ScaledInt(22, scale));
}

QString CreatePrimaryButtonStyleSheet(double scale)
{
    return QStringLiteral(
        "QPushButton#welcomePrimaryButton{"
        "background:#0f766e;color:white;border:%1px solid #0b4f4a;border-radius:0px;"
        "padding:%2px %3px;font-weight:800;"
        "}"
        "QPushButton#welcomePrimaryButton:hover{background:#115e59;border-color:#0f766e;}").arg(
        ScaledInt(4, scale)).arg(ScaledInt(30, scale)).arg(ScaledInt(60, scale));
}

QString CreateSecondaryButtonStyleSheet(double scale)
{
    return QStringLiteral(
        "QPushButton#welcomeSecondaryButton{"
        "background:#f8fafc;color:#0f172a;"
        "border:%1px solid #64748b;border-radius:0px;"
        "padding:%2px %3px;font-weight:800;"
        "}"
        "QPushButton#welcomeSecondaryButton:hover{"
        "background:white;border-color:#0f766e;"
        "}").arg(ScaledInt(4, scale)).arg(ScaledInt(30, scale)).arg(ScaledInt(60, scale));
}

QFrame* CreateRecentProjectPlaceholder(
    QWidget* parent,
    QBoxLayout** recentLayout,
    QLabel** titleLabel,
    QLabel** bodyLabel,
    QLabel** emptyLabel)
{
    auto* card = new QFrame(parent);
    card->setObjectName(QStringLiteral("recentProjectCard"));

    auto* layout = new QVBoxLayout(card);

    auto* title = CreateTextLabel(
        QStringLiteral("最近打开的项目"),
        QStringLiteral("font-weight:800;color:#172033;"),
        card);
    auto* body = CreateTextLabel(
        QStringLiteral("点击列表中的项目可直接恢复工作上下文。"),
        QStringLiteral("line-height:1.45;color:#475467;"),
        card);
    auto* empty = CreateTextLabel(
        QStringLiteral("暂无最近项目"),
        QString(),
        card);
    empty->setAlignment(Qt::AlignCenter);

    layout->addWidget(title);
    layout->addWidget(body);
    layout->addWidget(empty);

    if (recentLayout != nullptr)
    {
        *recentLayout = layout;
    }
    if (titleLabel != nullptr)
    {
        *titleLabel = title;
    }
    if (bodyLabel != nullptr)
    {
        *bodyLabel = body;
    }
    if (emptyLabel != nullptr)
    {
        *emptyLabel = empty;
    }

    return card;
}

QString CreateRecentProjectButtonText(const QString& projectRootPath)
{
    const QFileInfo projectRootInfo(projectRootPath);
    QString displayName = projectRootInfo.fileName();
    if (displayName.trimmed().isEmpty())
    {
        displayName = QDir::toNativeSeparators(projectRootInfo.absoluteFilePath());
    }

    return QStringLiteral("%1\n%2")
        .arg(displayName, QDir::toNativeSeparators(projectRootInfo.absoluteFilePath()));
}

} // namespace

namespace RoboSDP::Desktop::Widgets
{

ProjectWelcomeWidget::ProjectWelcomeWidget(QWidget* parent)
    : QWidget(parent)
{
    setObjectName(QStringLiteral("projectWelcomeWidget"));
    setAutoFillBackground(true);
    setStyleSheet(QStringLiteral(
        "QWidget#projectWelcomeWidget{"
        "background:qlineargradient(x1:0,y1:0,x2:1,y2:1,"
        "stop:0 #e5e7eb,stop:0.55 #f8fafc,stop:1 #d7dee8);"
        "}"));

    m_root_layout = new QVBoxLayout(this);
    m_root_layout->setSpacing(0);

    m_center_row = new QHBoxLayout();
    m_center_row->addStretch(1);

    m_hero_card = new QFrame(this);
    m_hero_card->setObjectName(QStringLiteral("welcomeHeroCard"));
    m_hero_card->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

    auto* shadow = new QGraphicsDropShadowEffect(m_hero_card);
    shadow->setBlurRadius(32);
    shadow->setOffset(16, 20);
    shadow->setColor(QColor(15, 23, 42, 42));
    m_hero_card->setGraphicsEffect(shadow);

    m_hero_layout = new QVBoxLayout(m_hero_card);

    m_logo_label = new QLabel(m_hero_card);
    m_logo_label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

    const QString version = QCoreApplication::applicationVersion().trimmed().isEmpty()
        ? QStringLiteral("0.1.0")
        : QCoreApplication::applicationVersion().trimmed();

    m_title_label = CreateTextLabel(
        QStringLiteral("RoboSDP Desktop"),
        QStringLiteral("font-weight:900;letter-spacing:0px;color:#0f172a;"),
        m_hero_card);
    m_version_label = CreateTextLabel(
        QStringLiteral("Version %1").arg(version),
        QStringLiteral("font-weight:800;color:#0f766e;"),
        m_hero_card);
    m_subtitle_label = CreateTextLabel(
        QStringLiteral("从需求定义到构型、运动学、动力学、选型、规划与方案导出的一体化机器人设计工作台。"),
        QStringLiteral("line-height:1.55;color:#334155;"),
        m_hero_card);

    m_actions_layout = new QHBoxLayout();
    m_new_project_button = new QPushButton(QStringLiteral("新建项目"), m_hero_card);
    m_new_project_button->setObjectName(QStringLiteral("welcomePrimaryButton"));
    m_new_project_button->setCursor(Qt::PointingHandCursor);

    m_open_project_button = new QPushButton(QStringLiteral("打开项目"), m_hero_card);
    m_open_project_button->setObjectName(QStringLiteral("welcomeSecondaryButton"));
    m_open_project_button->setCursor(Qt::PointingHandCursor);

    m_actions_layout->addWidget(m_new_project_button);
    m_actions_layout->addWidget(m_open_project_button);
    m_actions_layout->addStretch(1);

    m_hint_label = CreateTextLabel(
        QStringLiteral("也可以继续使用顶部功能区的“文件”页签完成同样操作。"),
        QStringLiteral("color:#475569;"),
        m_hero_card);

    m_hero_layout->addWidget(m_logo_label);
    m_hero_layout->addWidget(m_title_label);
    m_hero_layout->addWidget(m_version_label);
    m_hero_layout->addSpacing(8);
    m_hero_layout->addWidget(m_subtitle_label);
    m_hero_layout->addSpacing(24);
    m_hero_layout->addLayout(m_actions_layout);
    m_hero_layout->addWidget(m_hint_label);

    m_side_panel = new QWidget(this);
    m_side_panel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    m_side_layout = new QVBoxLayout(m_side_panel);
    m_side_layout->setContentsMargins(0, 0, 0, 0);

    m_workflow_card = new QFrame(m_side_panel);
    m_workflow_card->setObjectName(QStringLiteral("workflowCard"));
    m_workflow_layout = new QVBoxLayout(m_workflow_card);
    m_workflow_title_label = CreateTextLabel(
        QStringLiteral("建议工作流"),
        QStringLiteral("font-weight:800;color:#134e4a;"),
        m_workflow_card);
    m_workflow_body_label = CreateTextLabel(
        QStringLiteral("1. 新建或打开项目\n2. 填写任务需求\n3. 生成构型并进入运动学分析\n4. 保存阶段成果并导出方案"),
        QStringLiteral("line-height:1.75;color:#164e63;"),
        m_workflow_card);
    m_workflow_layout->addWidget(m_workflow_title_label);
    m_workflow_layout->addWidget(m_workflow_body_label);

    m_recent_card = CreateRecentProjectPlaceholder(
        m_side_panel,
        &m_recent_layout,
        &m_recent_title_label,
        &m_recent_body_label,
        &m_recent_empty_label);

    m_side_layout->addWidget(m_workflow_card);
    m_side_layout->addWidget(m_recent_card);

    m_center_row->addWidget(m_hero_card, 0, Qt::AlignVCenter);
    m_center_row->addWidget(m_side_panel, 0, Qt::AlignVCenter);
    m_center_row->addStretch(1);

    m_root_layout->addStretch(1);
    m_root_layout->addLayout(m_center_row);
    m_root_layout->addStretch(1);

    UpdateResponsiveScale();

    connect(m_new_project_button, &QPushButton::clicked, this, &ProjectWelcomeWidget::CreateNewProjectRequested);
    connect(m_open_project_button, &QPushButton::clicked, this, &ProjectWelcomeWidget::OpenProjectRequested);
}

void ProjectWelcomeWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    UpdateResponsiveScale();
}

void ProjectWelcomeWidget::UpdateResponsiveScale()
{
    const double baseWidth = 112.0 * 2.0 + 1280.0 + 64.0 + 780.0;
    const double baseHeight = 88.0 * 2.0 + 1080.0;
    const double availableWidth = std::max(1, width());
    const double availableHeight = std::max(1, height());
    const double scale = std::clamp(
        std::min(availableWidth / baseWidth, availableHeight / baseHeight),
        0.28,
        1.0);
    if (std::abs(scale - m_current_scale) < 0.005)
    {
        return;
    }
    m_current_scale = scale;

    m_root_layout->setContentsMargins(
        ScaledInt(112, scale),
        ScaledInt(88, scale),
        ScaledInt(112, scale),
        ScaledInt(88, scale));
    m_center_row->setSpacing(ScaledInt(64, scale));
    m_hero_layout->setContentsMargins(
        ScaledInt(128, scale),
        ScaledInt(112, scale),
        ScaledInt(128, scale),
        ScaledInt(112, scale));
    m_hero_layout->setSpacing(ScaledInt(40, scale));
    m_actions_layout->setSpacing(ScaledInt(32, scale));
    m_side_layout->setSpacing(ScaledInt(44, scale));
    m_workflow_layout->setContentsMargins(
        ScaledInt(56, scale),
        ScaledInt(48, scale),
        ScaledInt(56, scale),
        ScaledInt(48, scale));
    m_workflow_layout->setSpacing(ScaledInt(28, scale));
    m_recent_layout->setContentsMargins(
        ScaledInt(56, scale),
        ScaledInt(48, scale),
        ScaledInt(56, scale),
        ScaledInt(48, scale));
    m_recent_layout->setSpacing(ScaledInt(24, scale));

    m_hero_card->setMinimumSize(0, 0);
    m_hero_card->resize(ScaledInt(1280, scale), ScaledInt(1080, scale));
    m_hero_card->setMaximumWidth(ScaledInt(1760, scale));
    m_side_panel->setMinimumWidth(0);
    m_side_panel->resize(ScaledInt(780, scale), m_side_panel->height());
    m_side_panel->setMaximumWidth(ScaledInt(880, scale));
    m_workflow_card->setMinimumHeight(0);
    m_workflow_card->resize(m_workflow_card->width(), ScaledInt(440, scale));
    m_recent_card->setMinimumHeight(0);
    m_recent_card->resize(m_recent_card->width(), ScaledInt(472, scale));
    m_new_project_button->setMinimumHeight(ScaledInt(112, scale));
    m_open_project_button->setMinimumHeight(ScaledInt(112, scale));

    m_hero_card->setStyleSheet(CreateHeroCardStyleSheet(scale));
    m_workflow_card->setStyleSheet(CreateWorkflowCardStyleSheet(scale));
    m_recent_card->setStyleSheet(CreateRecentCardStyleSheet(scale));
    m_recent_empty_label->setStyleSheet(CreateRecentEmptyStyleSheet(scale));

    const int logoSize = ScaledInt(240, scale);
    if (logoSize != m_current_logo_size)
    {
        m_current_logo_size = logoSize;
        m_logo_label->setPixmap(QPixmap(QStringLiteral(":/app/robosdp_256.png")).scaled(
            logoSize,
            logoSize,
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation));
    }

    SetLabelFontSize(m_title_label, 92, scale);
    SetLabelFontSize(m_version_label, 30, scale);
    SetLabelFontSize(m_subtitle_label, 38, scale);
    SetLabelFontSize(m_hint_label, 28, scale);
    SetLabelFontSize(m_workflow_title_label, 38, scale);
    SetLabelFontSize(m_workflow_body_label, 30, scale);
    SetLabelFontSize(m_recent_title_label, 38, scale);
    SetLabelFontSize(m_recent_body_label, 30, scale);
    SetLabelFontSize(m_recent_empty_label, 30, scale);
    SetButtonFontSize(m_new_project_button, 34, scale);
    SetButtonFontSize(m_open_project_button, 34, scale);

    for (QPushButton* button : m_recent_project_buttons)
    {
        if (button == nullptr)
        {
            continue;
        }
        SetButtonFontSize(button, 22, scale);
        button->setMinimumHeight(ScaledInt(92, scale));
        button->setStyleSheet(CreateRecentProjectButtonStyleSheet(scale));
    }

    m_new_project_button->setStyleSheet(CreatePrimaryButtonStyleSheet(scale));
    m_open_project_button->setStyleSheet(CreateSecondaryButtonStyleSheet(scale));
}

void ProjectWelcomeWidget::SetRecentProjects(const QStringList& projectRootPaths)
{
    ClearRecentProjectButtons();

    const bool hasRecentProjects = !projectRootPaths.isEmpty();
    if (m_recent_empty_label != nullptr)
    {
        m_recent_empty_label->setVisible(!hasRecentProjects);
    }

    for (const QString& projectRootPath : projectRootPaths)
    {
        const QString normalizedPath = QDir::cleanPath(projectRootPath);
        if (normalizedPath.trimmed().isEmpty())
        {
            continue;
        }

        auto* button = new QPushButton(CreateRecentProjectButtonText(normalizedPath), m_recent_card);
        button->setObjectName(QStringLiteral("recentProjectButton"));
        button->setCursor(Qt::PointingHandCursor);
        button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        button->setToolTip(QDir::toNativeSeparators(normalizedPath));
        connect(button, &QPushButton::clicked, this, [this, normalizedPath]() {
            emit RecentProjectOpenRequested(normalizedPath);
        });

        const int insertIndex = m_recent_empty_label == nullptr
            ? m_recent_layout->count()
            : std::max(0, m_recent_layout->indexOf(m_recent_empty_label));
        m_recent_layout->insertWidget(insertIndex, button);
        m_recent_project_buttons.push_back(button);
    }

    m_current_scale = -1.0;
    UpdateResponsiveScale();
}

void ProjectWelcomeWidget::ClearRecentProjectButtons()
{
    for (QPushButton* button : m_recent_project_buttons)
    {
        if (button == nullptr)
        {
            continue;
        }
        m_recent_layout->removeWidget(button);
        button->deleteLater();
    }
    m_recent_project_buttons.clear();
}

} // namespace RoboSDP::Desktop::Widgets
