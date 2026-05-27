#pragma once

#include <QLabel>
#include <QPushButton>
#include <QFrame>
#include <QStringList>
#include <QWidget>
#include <vector>

class QBoxLayout;
class QResizeEvent;

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
    void SetRecentProjects(const QStringList& projectRootPaths);

protected:
    void resizeEvent(QResizeEvent* event) override;

signals:
    void CreateNewProjectRequested();
    void OpenProjectRequested();
    void RecentProjectOpenRequested(const QString& projectRootPath);

private:
    void UpdateResponsiveScale();
    void ClearRecentProjectButtons();

    QBoxLayout* m_root_layout = nullptr;
    QBoxLayout* m_center_row = nullptr;
    QBoxLayout* m_hero_layout = nullptr;
    QBoxLayout* m_actions_layout = nullptr;
    QBoxLayout* m_side_layout = nullptr;
    QBoxLayout* m_workflow_layout = nullptr;
    QBoxLayout* m_recent_layout = nullptr;

    QFrame* m_hero_card = nullptr;
    QFrame* m_workflow_card = nullptr;
    QFrame* m_recent_card = nullptr;
    QLabel* m_logo_label = nullptr;
    QLabel* m_title_label = nullptr;
    QLabel* m_version_label = nullptr;
    QLabel* m_subtitle_label = nullptr;
    QLabel* m_hint_label = nullptr;
    QLabel* m_workflow_title_label = nullptr;
    QLabel* m_workflow_body_label = nullptr;
    QLabel* m_recent_title_label = nullptr;
    QLabel* m_recent_body_label = nullptr;
    QLabel* m_recent_empty_label = nullptr;
    QPushButton* m_new_project_button = nullptr;
    QPushButton* m_open_project_button = nullptr;
    std::vector<QPushButton*> m_recent_project_buttons;
    QWidget* m_side_panel = nullptr;
    double m_current_scale = -1.0;
    int m_current_logo_size = 0;
};

} // namespace RoboSDP::Desktop::Widgets
