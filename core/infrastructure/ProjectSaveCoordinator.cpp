#include "core/infrastructure/ProjectSaveCoordinator.h"

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

    for (IProjectSaveParticipant* participant : m_participants)
    {
        if (participant == nullptr)
        {
            continue;
        }

        ProjectSaveItemResult itemResult = participant->SaveCurrentDraft();
        if (itemResult.module_name.trimmed().isEmpty())
        {
            itemResult.module_name = participant->ModuleName();
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
    summary.message = QStringLiteral("全局保存完成：成功 %1，失败 %2。")
        .arg(summary.success_count)
        .arg(summary.failure_count);
    return summary;
}

} // namespace RoboSDP::Infrastructure
