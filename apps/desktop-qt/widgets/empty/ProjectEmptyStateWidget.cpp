#include "apps/desktop-qt/widgets/empty/ProjectEmptyStateWidget.h"

#include <QFrame>
#include <QLabel>
#include <QVBoxLayout>

namespace RoboSDP::Desktop::Widgets
{

ProjectEmptyStateWidget::ProjectEmptyStateWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(18, 18, 18, 18);
    rootLayout->setSpacing(12);

    auto* content = new QFrame(this);
    content->setFrameShape(QFrame::StyledPanel);
    auto* contentLayout = new QVBoxLayout(content);
    contentLayout->setContentsMargins(18, 18, 18, 18);
    contentLayout->setSpacing(10);

    auto* titleLabel = new QLabel(QStringLiteral("尚未打开项目"), content);
    titleLabel->setStyleSheet(QStringLiteral("font-size:18px;font-weight:600;color:#1f2937;"));

    auto* bodyLabel = new QLabel(
        QStringLiteral(
            "请先新建项目或打开已有 RoboSDP 项目。\n\n"
            "项目用于统一保存任务需求、构型设计、运动学模型、动力学验证、驱动选型、规划分析与方案导出结果。"),
        content);
    bodyLabel->setWordWrap(true);
    bodyLabel->setStyleSheet(QStringLiteral("font-size:13px;line-height:1.45;color:#475467;"));

    auto* hintLabel = new QLabel(
        QStringLiteral("可从顶部功能区“文件”页签选择“新建项目”或“打开项目”。"),
        content);
    hintLabel->setWordWrap(true);
    hintLabel->setStyleSheet(QStringLiteral("font-size:12px;color:#667085;"));

    contentLayout->addWidget(titleLabel);
    contentLayout->addWidget(bodyLabel);
    contentLayout->addWidget(hintLabel);
    contentLayout->addStretch();

    rootLayout->addWidget(content);
    rootLayout->addStretch();
}

} // namespace RoboSDP::Desktop::Widgets
