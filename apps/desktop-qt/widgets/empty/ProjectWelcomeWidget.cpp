#include "apps/desktop-qt/widgets/empty/ProjectWelcomeWidget.h"

#include <QApplication>
#include <QCoreApplication>
#include <QFrame>
#include <QGraphicsDropShadowEffect>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSizePolicy>
#include <QVBoxLayout>

namespace
{

QLabel* CreateTextLabel(const QString& text, const QString& styleSheet, QWidget* parent)
{
    auto* label = new QLabel(text, parent);
    label->setWordWrap(true);
    label->setStyleSheet(styleSheet);
    return label;
}

QFrame* CreateRecentProjectPlaceholder(QWidget* parent)
{
    auto* card = new QFrame(parent);
    card->setObjectName(QStringLiteral("recentProjectCard"));
    card->setStyleSheet(QStringLiteral(
        "QFrame#recentProjectCard{"
        "background:rgba(255,255,255,0.82);"
        "border:1px solid rgba(148,163,184,0.26);"
        "border-radius:18px;"
        "}"));

    auto* layout = new QVBoxLayout(card);
    layout->setContentsMargins(22, 20, 22, 20);
    layout->setSpacing(8);

    auto* title = CreateTextLabel(
        QStringLiteral("最近打开的项目"),
        QStringLiteral("font-size:17px;font-weight:700;color:#172033;"),
        card);
    auto* body = CreateTextLabel(
        QStringLiteral("这里已预留 Recent Projects 区域。后续接入项目历史记录后，可点击项目卡片直接打开。"),
        QStringLiteral("font-size:13px;line-height:1.45;color:#667085;"),
        card);
    auto* empty = CreateTextLabel(
        QStringLiteral("暂无最近项目"),
        QStringLiteral(
            "margin-top:8px;padding:18px;border-radius:12px;"
            "background:rgba(226,232,240,0.55);"
            "color:#64748b;font-size:13px;"),
        card);
    empty->setAlignment(Qt::AlignCenter);

    layout->addWidget(title);
    layout->addWidget(body);
    layout->addWidget(empty);
    return card;
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
        "stop:0 #eef7ff,stop:0.42 #f8fbff,stop:1 #fff6e7);"
        "}"
        "QPushButton#welcomePrimaryButton{"
        "background:#0f766e;color:white;border:none;border-radius:14px;"
        "padding:13px 24px;font-size:16px;font-weight:700;"
        "}"
        "QPushButton#welcomePrimaryButton:hover{background:#115e59;}"
        "QPushButton#welcomeSecondaryButton{"
        "background:rgba(255,255,255,0.9);color:#0f172a;"
        "border:1px solid rgba(15,23,42,0.14);border-radius:14px;"
        "padding:13px 24px;font-size:16px;font-weight:700;"
        "}"
        "QPushButton#welcomeSecondaryButton:hover{"
        "background:white;border-color:rgba(15,118,110,0.45);"
        "}"));

    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(64, 44, 64, 44);
    rootLayout->setSpacing(0);

    auto* centerRow = new QHBoxLayout();
    centerRow->setSpacing(36);
    centerRow->addStretch(1);

    auto* heroCard = new QFrame(this);
    heroCard->setObjectName(QStringLiteral("welcomeHeroCard"));
    heroCard->setMaximumWidth(760);
    heroCard->setMinimumWidth(560);
    heroCard->setStyleSheet(QStringLiteral(
        "QFrame#welcomeHeroCard{"
        "background:rgba(255,255,255,0.78);"
        "border:1px solid rgba(148,163,184,0.30);"
        "border-radius:28px;"
        "}"));

    auto* shadow = new QGraphicsDropShadowEffect(heroCard);
    shadow->setBlurRadius(34);
    shadow->setOffset(0, 18);
    shadow->setColor(QColor(15, 23, 42, 34));
    heroCard->setGraphicsEffect(shadow);

    auto* heroLayout = new QVBoxLayout(heroCard);
    heroLayout->setContentsMargins(52, 46, 52, 46);
    heroLayout->setSpacing(18);

    auto* logoLabel = new QLabel(heroCard);
    logoLabel->setPixmap(QPixmap(QStringLiteral(":/app/robosdp_256.png")).scaled(
        104,
        104,
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation));
    logoLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

    const QString version = QCoreApplication::applicationVersion().trimmed().isEmpty()
        ? QStringLiteral("0.1.0")
        : QCoreApplication::applicationVersion().trimmed();

    auto* titleLabel = CreateTextLabel(
        QStringLiteral("RoboSDP Desktop"),
        QStringLiteral("font-size:42px;font-weight:800;letter-spacing:-1px;color:#0f172a;"),
        heroCard);
    auto* versionLabel = CreateTextLabel(
        QStringLiteral("Version %1").arg(version),
        QStringLiteral("font-size:14px;font-weight:700;color:#0f766e;"),
        heroCard);
    auto* subtitleLabel = CreateTextLabel(
        QStringLiteral("从需求定义到构型、运动学、动力学、选型、规划与方案导出的一体化机器人设计工作台。"),
        QStringLiteral("font-size:17px;line-height:1.55;color:#475467;"),
        heroCard);

    auto* actionsLayout = new QHBoxLayout();
    actionsLayout->setSpacing(14);
    auto* newProjectButton = new QPushButton(QStringLiteral("新建项目"), heroCard);
    newProjectButton->setObjectName(QStringLiteral("welcomePrimaryButton"));
    newProjectButton->setCursor(Qt::PointingHandCursor);
    newProjectButton->setMinimumHeight(48);

    auto* openProjectButton = new QPushButton(QStringLiteral("打开项目"), heroCard);
    openProjectButton->setObjectName(QStringLiteral("welcomeSecondaryButton"));
    openProjectButton->setCursor(Qt::PointingHandCursor);
    openProjectButton->setMinimumHeight(48);

    actionsLayout->addWidget(newProjectButton);
    actionsLayout->addWidget(openProjectButton);
    actionsLayout->addStretch(1);

    auto* hintLabel = CreateTextLabel(
        QStringLiteral("也可以继续使用顶部功能区的“文件”页签完成同样操作。"),
        QStringLiteral("font-size:13px;color:#667085;"),
        heroCard);

    heroLayout->addWidget(logoLabel);
    heroLayout->addWidget(titleLabel);
    heroLayout->addWidget(versionLabel);
    heroLayout->addSpacing(4);
    heroLayout->addWidget(subtitleLabel);
    heroLayout->addSpacing(12);
    heroLayout->addLayout(actionsLayout);
    heroLayout->addWidget(hintLabel);

    auto* sidePanel = new QWidget(this);
    sidePanel->setMaximumWidth(360);
    sidePanel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    auto* sideLayout = new QVBoxLayout(sidePanel);
    sideLayout->setContentsMargins(0, 0, 0, 0);
    sideLayout->setSpacing(16);

    auto* workflowCard = new QFrame(sidePanel);
    workflowCard->setObjectName(QStringLiteral("workflowCard"));
    workflowCard->setStyleSheet(QStringLiteral(
        "QFrame#workflowCard{"
        "background:rgba(15,118,110,0.10);"
        "border:1px solid rgba(15,118,110,0.18);"
        "border-radius:18px;"
        "}"));
    auto* workflowLayout = new QVBoxLayout(workflowCard);
    workflowLayout->setContentsMargins(22, 20, 22, 20);
    workflowLayout->setSpacing(10);
    workflowLayout->addWidget(CreateTextLabel(
        QStringLiteral("建议工作流"),
        QStringLiteral("font-size:17px;font-weight:700;color:#134e4a;"),
        workflowCard));
    workflowLayout->addWidget(CreateTextLabel(
        QStringLiteral("1. 新建或打开项目\n2. 填写任务需求\n3. 生成构型并进入运动学分析\n4. 保存阶段成果并导出方案"),
        QStringLiteral("font-size:13px;line-height:1.65;color:#335c67;"),
        workflowCard));

    sideLayout->addWidget(workflowCard);
    sideLayout->addWidget(CreateRecentProjectPlaceholder(sidePanel));

    centerRow->addWidget(heroCard, 0, Qt::AlignVCenter);
    centerRow->addWidget(sidePanel, 0, Qt::AlignVCenter);
    centerRow->addStretch(1);

    rootLayout->addStretch(1);
    rootLayout->addLayout(centerRow);
    rootLayout->addStretch(1);

    connect(newProjectButton, &QPushButton::clicked, this, &ProjectWelcomeWidget::CreateNewProjectRequested);
    connect(openProjectButton, &QPushButton::clicked, this, &ProjectWelcomeWidget::OpenProjectRequested);
}

} // namespace RoboSDP::Desktop::Widgets
