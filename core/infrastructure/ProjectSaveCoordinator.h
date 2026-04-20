#pragma once

#include <QString>
#include <QVector>

namespace RoboSDP::Infrastructure
{

/**
 * @brief 单模块保存结果 DTO。
 *
 * 该对象只承载项目级保存编排所需的轻量结果，不保存模块内部 DTO 或 UI 状态。
 */
struct ProjectSaveItemResult
{
    /// 模块名称，用于日志和汇总展示。
    QString module_name;

    /// 当前模块是否保存成功。
    bool success = false;

    /// 当前模块保存结果的中文说明。
    QString message;
};

/**
 * @brief 全局保存汇总结果 DTO。
 *
 * 汇总对象由 ProjectSaveCoordinator 生成，MainWindow 只负责展示，
 * 不参与具体模块保存细节。
 */
struct ProjectSaveSummary
{
    /// 是否存在可用项目上下文且所有参与模块均保存成功。
    bool success = false;

    /// 本次保存使用的项目根目录。
    QString project_root_path;

    /// 成功保存的模块数量。
    int success_count = 0;

    /// 保存失败的模块数量。
    int failure_count = 0;

    /// 面向用户的汇总说明。
    QString message;

    /// 各模块保存结果，按注册顺序排列。
    QVector<ProjectSaveItemResult> item_results;
};

/**
 * @brief 项目级保存参与者接口。
 *
 * 业务模块 Widget 通过实现该接口接入全局保存；
 * 协调器只认接口，不依赖具体 UI 类。
 */
class IProjectSaveParticipant
{
public:
    virtual ~IProjectSaveParticipant() = default;

    /// 返回模块名称，用于日志汇总。
    virtual QString ModuleName() const = 0;

    /// 保存当前模块草稿并返回结果。
    virtual ProjectSaveItemResult SaveCurrentDraft() = 0;
};

/**
 * @brief 项目级保存协调器。
 *
 * 该类负责读取 ProjectManager 中的当前项目目录，按固定顺序调用已注册模块保存，
 * 并汇总每个模块的成功/失败结果。
 */
class ProjectSaveCoordinator
{
public:
    /// 注册一个模块保存参与者；不接管对象生命周期。
    void RegisterParticipant(IProjectSaveParticipant* participant);

    /// 执行全局保存；单个模块失败不影响后续模块继续保存。
    ProjectSaveSummary SaveAll() const;

private:
    QVector<IProjectSaveParticipant*> m_participants;
};

} // namespace RoboSDP::Infrastructure
