#pragma once

#include "core/repository/IJsonRepository.h"
#include "core/schema/ValidationResult.h"

namespace RoboSDP::Schema
{

/**
 * @brief 最小 schema / DTO 一致性检查器。
 *
 * 由于当前仓库尚未提供完整 JSON Schema 文件全集，
 * 本轮采用“关键顶层键 + 现有持久化文件结构”作为最小一致性口径：
 * 1. 优先检查每个模块已落盘 JSON 是否可读取；
 * 2. 再检查顶层关键键是否存在；
 * 3. 不引入重型反射或完整 schema 引擎。
 */
class SchemaDtoConsistencyChecker
{
public:
    explicit SchemaDtoConsistencyChecker(RoboSDP::Repository::IJsonRepository& repository);

    ValidationResult CheckProject(const QString& projectRootPath) const;

private:
    RoboSDP::Repository::IJsonRepository& m_repository;
};

} // namespace RoboSDP::Schema
