#include "core/infrastructure/ProjectManager.h"

#include <QDir>

namespace RoboSDP::Infrastructure
{

ProjectManager::ProjectManager(QObject* parent)
    : QObject(parent)
{
}

ProjectManager& ProjectManager::instance()
{
    static ProjectManager manager;
    return manager;
}

void ProjectManager::setCurrentProjectPath(const QString& path)
{
    const QString trimmedPath = path.trimmed();
    const QString normalizedPath = trimmedPath.isEmpty()
        ? QString()
        : QDir::toNativeSeparators(QDir(trimmedPath).absolutePath());
    if (normalizedPath == m_currentProjectPath)
    {
        return;
    }

    // 中文说明：项目路径是全局上下文状态，只在真实变化时通知 UI 和业务入口。
    m_currentProjectPath = normalizedPath;
    emit projectPathChanged(m_currentProjectPath);
}

QString ProjectManager::getCurrentProjectPath() const
{
    return m_currentProjectPath;
}

} // namespace RoboSDP::Infrastructure
