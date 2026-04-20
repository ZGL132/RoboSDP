#pragma once

#include <QObject>
#include <QString>

namespace RoboSDP::Infrastructure
{

/**
 * @brief 全局项目上下文管理器。
 *
 * ProjectManager 是当前进程内项目根目录的单一事实来源。
 * UI 页面只监听路径变化并读取当前路径，不再各自维护可写项目目录状态。
 */
class ProjectManager final : public QObject
{
    Q_OBJECT

public:
    /// 返回进程内唯一项目上下文实例。
    static ProjectManager& instance();

    /// 设置当前项目根目录；只有路径实际变化时才广播 projectPathChanged。
    void setCurrentProjectPath(const QString& path);

    /// 获取当前项目根目录，未打开项目时返回空字符串。
    QString getCurrentProjectPath() const;

signals:
    /// 当前项目根目录发生变化时发射。
    void projectPathChanged(const QString& newPath);

private:
    explicit ProjectManager(QObject* parent = nullptr);

    QString m_currentProjectPath;
};

} // namespace RoboSDP::Infrastructure
