#pragma once

#include "core/repository/IJsonRepository.h"
#include "core/schema/ValidationResult.h"

namespace RoboSDP::Schema
{

/**
 * @brief 第一阶段主链统一 validator 入口。
 *
 * 本轮不重构各模块业务 service，只做一层轻量注册表：
 * 1. 已有 Requirement / Topology validator 直接复用；
 * 2. 其它模块补最小结构校验；
 * 3. 所有输出统一映射到 field / code / message_zh / severity。
 */
class ModuleValidationRegistry
{
public:
    explicit ModuleValidationRegistry(RoboSDP::Repository::IJsonRepository& repository);

    ValidationResult ValidateProject(const QString& projectRootPath) const;

private:
    RoboSDP::Repository::IJsonRepository& m_repository;
};

} // namespace RoboSDP::Schema
