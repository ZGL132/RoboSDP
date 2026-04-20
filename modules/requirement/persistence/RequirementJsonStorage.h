#pragma once

#include "core/errors/ErrorCode.h"
#include "core/repository/IJsonRepository.h"
#include "modules/requirement/dto/RequirementModelDto.h"

namespace RoboSDP::Requirement::Persistence
{

/**
 * @brief Requirement JSON 持久化组件。
 *
 * 该类负责 Requirement DTO 与 `requirements/requirement-model.json`
 * 之间的序列化与反序列化，不承担校验编排逻辑。
 */
class RequirementJsonStorage
{
public:
    explicit RequirementJsonStorage(RoboSDP::Repository::IJsonRepository& repository);

    RoboSDP::Errors::ErrorCode Save(
        const QString& projectRootPath,
        const RoboSDP::Requirement::Dto::RequirementModelDto& model) const;

    RoboSDP::Errors::ErrorCode Load(
        const QString& projectRootPath,
        RoboSDP::Requirement::Dto::RequirementModelDto& model) const;

    /// 返回 Requirement JSON 的相对路径。
    QString RelativeFilePath() const;

    /// 根据项目根目录推导 Requirement JSON 的绝对路径。
    QString BuildAbsoluteFilePath(const QString& projectRootPath) const;

private:
    QJsonObject ToJsonObject(const RoboSDP::Requirement::Dto::RequirementModelDto& model) const;
    RoboSDP::Requirement::Dto::RequirementModelDto FromJsonObject(const QJsonObject& jsonObject) const;

private:
    RoboSDP::Repository::IJsonRepository& m_repository;
};

} // namespace RoboSDP::Requirement::Persistence
