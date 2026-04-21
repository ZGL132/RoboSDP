#pragma once

#include <QString>
#include <QStringList>
#include <QVector>

namespace RoboSDP::Infrastructure
{

/**
 * @brief 单模块依赖脏状态。
 *
 * 该 DTO 只描述项目主链上的“上游是否变更”关系，不读取或修改任何业务模块数据。
 */
struct ProjectDirtyModuleState
{
    /// 当前模块名称，需与 IProjectSaveParticipant::ModuleName() 保持一致。
    QString module_name;

    /// 当前模块自身是否存在未保存变更。
    bool has_unsaved_changes = false;

    /// 当前模块上游是否存在未保存变更，因此需要刷新或重算。
    bool has_dirty_upstream = false;

    /// 触发当前模块刷新提示的上游模块列表。
    QStringList dirty_upstream_modules;
};

/**
 * @brief 第一阶段项目主链脏依赖图。
 *
 * 当前最小版只表达线性主链：
 * Requirement -> Topology -> Kinematics -> Dynamics -> Selection -> Planning -> Scheme。
 * 它不执行自动重算，只给全局保存结果提供“需刷新”提示依据。
 */
class ProjectDirtyDependencyGraph
{
public:
    /// 按参与者注册顺序评估依赖脏状态，返回顺序与输入 moduleNames 一致。
    QVector<ProjectDirtyModuleState> Evaluate(
        const QVector<QString>& moduleNames,
        const QVector<bool>& dirtyFlags) const;

private:
    int MainChainIndex(const QString& moduleName) const;
};

} // namespace RoboSDP::Infrastructure
