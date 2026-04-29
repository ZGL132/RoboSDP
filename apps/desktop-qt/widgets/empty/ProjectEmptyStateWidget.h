#pragma once

#include <QWidget>

namespace RoboSDP::Desktop::Widgets
{

/**
 * @brief 未打开项目时的属性面板空状态。
 *
 * 该控件只负责展示启动引导，不承载任何业务模块编辑状态，避免无项目上下文时误进入工作态。
 */
class ProjectEmptyStateWidget : public QWidget
{
public:
    explicit ProjectEmptyStateWidget(QWidget* parent = nullptr);
};

} // namespace RoboSDP::Desktop::Widgets
