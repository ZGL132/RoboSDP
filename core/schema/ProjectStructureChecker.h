#pragma once

#include "core/schema/ValidationResult.h"

namespace RoboSDP::Schema
{

/**
 * @brief 项目目录结构检查器。
 *
 * 本轮只检查第一阶段主链约定的最小目录与主文件是否存在，
 * 不引入迁移系统，也不自动修复目录。
 */
class ProjectStructureChecker
{
public:
    ValidationResult CheckProject(const QString& projectRootPath) const;
};

} // namespace RoboSDP::Schema
