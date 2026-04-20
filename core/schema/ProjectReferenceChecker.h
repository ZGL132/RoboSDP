#pragma once

#include "core/repository/IJsonRepository.h"
#include "core/schema/ValidationResult.h"

namespace RoboSDP::Schema
{

/**
 * @brief 项目引用关系检查器。
 *
 * 当前仓库中引用既包含对象 ID，也包含相对文件路径，
 * 因此本检查器按“当前稳定可接受值集合”做最小兼容校验，
 * 不在本轮强行重构引用口径。
 */
class ProjectReferenceChecker
{
public:
    explicit ProjectReferenceChecker(RoboSDP::Repository::IJsonRepository& repository);

    ValidationResult CheckProject(const QString& projectRootPath) const;

private:
    RoboSDP::Repository::IJsonRepository& m_repository;
};

} // namespace RoboSDP::Schema
