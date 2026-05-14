#pragma once

#include <QWidget>

namespace RoboSDP::Desktop::Widgets
{

/**
 * @brief 未打开项目时的中央欢迎页。
 *
 * 该控件只负责启动引导和发出用户意图，不直接创建或打开项目；
 * 项目合法性、目录选择和全局上下文切换仍由 MainWindow / ProjectManager 统一处理。
 */
class ProjectWelcomeWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ProjectWelcomeWidget(QWidget* parent = nullptr);

signals:
    void CreateNewProjectRequested();
    void OpenProjectRequested();
};

} // namespace RoboSDP::Desktop::Widgets
