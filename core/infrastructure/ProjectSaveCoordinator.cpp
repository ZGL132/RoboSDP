#include "core/infrastructure/ProjectSaveCoordinator.h"

#include "core/infrastructure/ProjectDirtyDependencyGraph.h"
#include "core/infrastructure/ProjectManager.h"

namespace RoboSDP::Infrastructure
{

void ProjectSaveCoordinator::RegisterParticipant(IProjectSaveParticipant* participant)
{
    if (participant == nullptr || m_participants.contains(participant))
    {
        return;
    }

    m_participants.push_back(participant);
}

ProjectSaveSummary ProjectSaveCoordinator::SaveAll() const
{
    ProjectSaveSummary summary;
    summary.project_root_path = ProjectManager::instance().getCurrentProjectPath();
    if (summary.project_root_path.trimmed().isEmpty())
    {
        summary.success = false;
        summary.message = QStringLiteral("当前没有打开或新建项目，无法执行全局保存。");
        return summary;
    }

    QVector<QString> moduleNames;
    QVector<bool> dirtyFlags;
    moduleNames.reserve(m_participants.size());
    dirtyFlags.reserve(m_participants.size());
    for (IProjectSaveParticipant* participant : m_participants)
    {
        moduleNames.push_back(participant != nullptr ? participant->ModuleName() : QString());
        dirtyFlags.push_back(participant != nullptr && participant->HasUnsavedChanges());
    }

    const ProjectDirtyDependencyGraph dirtyDependencyGraph;
    const QVector<ProjectDirtyModuleState> dependencyStates =
        dirtyDependencyGraph.Evaluate(moduleNames, dirtyFlags);

    for (int participantIndex = 0; participantIndex < m_participants.size(); ++participantIndex)
    {
        IProjectSaveParticipant* participant = m_participants.at(participantIndex);
        if (participant == nullptr)
        {
            continue;
        }

        const ProjectDirtyModuleState dependencyState =
            participantIndex < dependencyStates.size()
                ? dependencyStates.at(participantIndex)
                : ProjectDirtyModuleState{};

        if (!dirtyFlags.at(participantIndex))
        {
            if (dependencyState.has_dirty_upstream)
            {
                // 中文说明：上游有变更时，下游即使自身未编辑，也需要提醒用户刷新或重算。
                ++summary.skipped_count;
                ++summary.refresh_required_count;
                summary.item_results.push_back({
                    participant->ModuleName(),
                    true,
                    QStringLiteral("上游模块 %1 已变更，当前模块需要刷新或重算后再保存。")
                        .arg(dependencyState.dirty_upstream_modules.join(QStringLiteral(", "))),
                    true,
                    true,
                    dependencyState.dirty_upstream_modules});
                continue;
            }

            // 中文说明：无脏数据时只进入汇总，不触发模块写盘，避免重复保存带来的无意义 IO。
            ++summary.skipped_count;
            summary.item_results.push_back({
                participant->ModuleName(),
                true,
                QStringLiteral("当前模块无未保存变更，已跳过写盘。"),
                true,
                false,
                {}});
            continue;
        }

        ProjectSaveItemResult itemResult = participant->SaveCurrentDraft();
        if (itemResult.module_name.trimmed().isEmpty())
        {
            itemResult.module_name = participant->ModuleName();
        }

        if (dependencyState.has_dirty_upstream)
        {
            ++summary.refresh_required_count;
            itemResult.dependency_refresh_required = true;
            itemResult.dirty_upstream_modules = dependencyState.dirty_upstream_modules;
            itemResult.message = QStringLiteral("%1；上游模块 %2 已变更，建议刷新或重算当前模块。")
                .arg(itemResult.message, dependencyState.dirty_upstream_modules.join(QStringLiteral(", ")));
        }

        if (itemResult.success)
        {
            ++summary.success_count;
        }
        else
        {
            ++summary.failure_count;
        }

        // 中文说明：单模块失败只进入汇总，不中断后续模块保存。
        summary.item_results.push_back(itemResult);
    }

    summary.success = summary.failure_count == 0 && !summary.item_results.isEmpty();
    summary.message = QStringLiteral("全局保存完成：成功 %1，失败 %2，跳过 %3，需刷新 %4。")
        .arg(summary.success_count)
        .arg(summary.failure_count)
        .arg(summary.skipped_count)
        .arg(summary.refresh_required_count);
    return summary;
}

} // namespace RoboSDP::Infrastructure
