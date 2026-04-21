#include "core/infrastructure/ProjectDirtyDependencyGraph.h"

namespace RoboSDP::Infrastructure
{

QVector<ProjectDirtyModuleState> ProjectDirtyDependencyGraph::Evaluate(
    const QVector<QString>& moduleNames,
    const QVector<bool>& dirtyFlags) const
{
    QVector<ProjectDirtyModuleState> states;
    states.reserve(moduleNames.size());

    for (int index = 0; index < moduleNames.size(); ++index)
    {
        ProjectDirtyModuleState state;
        state.module_name = moduleNames.at(index);
        state.has_unsaved_changes = index < dirtyFlags.size() && dirtyFlags.at(index);

        const int currentMainChainIndex = MainChainIndex(state.module_name);
        if (currentMainChainIndex >= 0)
        {
            for (int upstreamIndex = 0; upstreamIndex < moduleNames.size(); ++upstreamIndex)
            {
                if (upstreamIndex >= dirtyFlags.size() || !dirtyFlags.at(upstreamIndex))
                {
                    continue;
                }

                const int upstreamMainChainIndex = MainChainIndex(moduleNames.at(upstreamIndex));
                if (upstreamMainChainIndex >= 0 && upstreamMainChainIndex < currentMainChainIndex)
                {
                    // 中文说明：只记录主链上更靠前且本轮已有未保存变更的上游模块。
                    state.has_dirty_upstream = true;
                    state.dirty_upstream_modules.push_back(moduleNames.at(upstreamIndex));
                }
            }
        }

        states.push_back(state);
    }

    return states;
}

int ProjectDirtyDependencyGraph::MainChainIndex(const QString& moduleName) const
{
    static const QStringList kMainChain = {
        QStringLiteral("Requirement"),
        QStringLiteral("Topology"),
        QStringLiteral("Kinematics"),
        QStringLiteral("Dynamics"),
        QStringLiteral("Selection"),
        QStringLiteral("Planning"),
        QStringLiteral("Scheme")};

    return kMainChain.indexOf(moduleName);
}

} // namespace RoboSDP::Infrastructure
