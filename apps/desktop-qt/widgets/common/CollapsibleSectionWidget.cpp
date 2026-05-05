#include "CollapsibleSectionWidget.h"

#include <QLabel>
#include <QToolButton>
#include <QVBoxLayout>

// 折叠面板标题栏的样式：左侧箭头 + 标题文字，鼠标悬停有高亮反馈
static const QString kToggleButtonStyle = QStringLiteral(
    "QToolButton {"
    "  border: none;"
    "  background: #f3f4f6;"
    "  padding: 6px 8px;"
    "  text-align: left;"
    "  font-weight: bold;"
    "  font-size: 32px;"
    "  border-radius: 4px;"
    "}"
    "QToolButton:hover {"
    "  background: #e5e7eb;"
    "}"
    "QToolButton::menu-indicator {"
    "  image: none;"
    "}"
);

CollapsibleSectionWidget::CollapsibleSectionWidget(const QString& title, QWidget* parent)
    : QWidget(parent)
{
    m_layout = new QVBoxLayout(this);
    m_layout->setContentsMargins(0, 0, 0, 0);
    m_layout->setSpacing(0);

    // 标题栏：使用 QToolButton 以便显示箭头指示器
    m_toggle_button = new QToolButton(this);
    m_toggle_button->setText(title);
    m_toggle_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    m_toggle_button->setArrowType(Qt::DownArrow);
    m_toggle_button->setCheckable(false);
    m_toggle_button->setStyleSheet(kToggleButtonStyle);
    m_toggle_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    connect(m_toggle_button, &QToolButton::clicked, this, &CollapsibleSectionWidget::OnToggleClicked);

    m_layout->addWidget(m_toggle_button);

    // 内容区域默认空，等待 SetContent 注入
    m_content_widget = nullptr;
}

void CollapsibleSectionWidget::SetContent(QWidget* contentWidget)
{
    if (m_content_widget != nullptr)
    {
        // 移除旧内容
        m_layout->removeWidget(m_content_widget);
        m_content_widget->deleteLater();
    }

    m_content_widget = contentWidget;
    if (m_content_widget != nullptr)
    {
        m_content_widget->setVisible(!m_collapsed);
        m_layout->addWidget(m_content_widget);
    }
}

void CollapsibleSectionWidget::SetCollapsed(bool collapsed)
{
    if (m_collapsed == collapsed)
        return;

    m_collapsed = collapsed;

    // 更新箭头方向和内容可见性
    m_toggle_button->setArrowType(m_collapsed ? Qt::RightArrow : Qt::DownArrow);

    if (m_content_widget != nullptr)
    {
        m_content_widget->setVisible(!m_collapsed);
    }

    emit CollapsedChanged(m_collapsed);
}

void CollapsibleSectionWidget::Toggle()
{
    SetCollapsed(!m_collapsed);
}

void CollapsibleSectionWidget::OnToggleClicked()
{
    Toggle();
}
