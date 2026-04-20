#pragma once

#include "core/errors/ErrorCode.h"

#include <QString>

namespace RoboSDP::Repository
{

/**
 * @brief 项目创建结果 DTO。
 *
 * 该对象只承载“新建项目”入口的执行结果，不保存 UI 临时控件状态，
 * 便于 MainWindow 将结果写入状态栏和底部输出信息面板。
 */
struct ProjectCreateResult
{
    /// 是否成功创建或初始化项目骨架。
    bool success = false;

    /// 统一错误码，成功时为 ErrorCode::Ok。
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::UnknownError;

    /// 面向用户的中文结果说明。
    QString message;

    /// 规范化后的项目根目录绝对路径。
    QString project_root_path;

    /// 从目录名推导出的项目名称，后续可由项目属性页修改。
    QString project_name;
};

/**
 * @brief 项目级最小服务。
 *
 * 当前阶段只负责初始化项目目录骨架与 project.json 元信息，
 * 不直接创建各业务模块草稿，避免和 Requirement / Topology 等模块边界耦合。
 */
class ProjectService
{
public:
    /// 创建最小项目骨架；当 project.json 已存在时拒绝覆盖。
    ProjectCreateResult CreateProject(const QString& projectRootPath) const;
};

} // namespace RoboSDP::Repository
