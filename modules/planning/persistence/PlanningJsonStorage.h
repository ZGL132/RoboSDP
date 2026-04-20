#pragma once

#include "core/errors/ErrorCode.h"
#include "core/repository/IJsonRepository.h"
#include "modules/planning/dto/PlanningVerificationResultDto.h"

#include <QJsonObject>

namespace RoboSDP::Planning::Persistence
{

/**
 * @brief Planning JSON 持久化组件。
 *
 * 本轮按文档的最小目录约定拆成三个文件：
 * 1. `planning/planning-scene.json`
 * 2. `planning/planning-requests.json`
 * 3. `planning/planning-results.json`
 */
class PlanningJsonStorage
{
public:
    explicit PlanningJsonStorage(RoboSDP::Repository::IJsonRepository& repository);

    RoboSDP::Errors::ErrorCode Save(
        const QString& projectRootPath,
        const RoboSDP::Planning::Dto::PlanningWorkspaceStateDto& state) const;

    RoboSDP::Errors::ErrorCode Load(
        const QString& projectRootPath,
        RoboSDP::Planning::Dto::PlanningWorkspaceStateDto& state) const;

    QString RelativeSceneFilePath() const;
    QString RelativeRequestFilePath() const;
    QString RelativeResultFilePath() const;
    QString BuildAbsoluteSceneFilePath(const QString& projectRootPath) const;
    QString BuildAbsoluteRequestFilePath(const QString& projectRootPath) const;
    QString BuildAbsoluteResultFilePath(const QString& projectRootPath) const;

private:
    QJsonObject ToSceneJsonObject(const RoboSDP::Planning::Dto::PlanningSceneDto& scene) const;
    RoboSDP::Planning::Dto::PlanningSceneDto FromSceneJsonObject(const QJsonObject& jsonObject) const;

    QJsonObject ToRequestsJsonObject(const std::vector<RoboSDP::Planning::Dto::PlanningRequestDto>& requests) const;
    std::vector<RoboSDP::Planning::Dto::PlanningRequestDto> FromRequestsJsonObject(const QJsonObject& jsonObject) const;

    QJsonObject ToResultsJsonObject(
        const std::vector<RoboSDP::Planning::Dto::PlanningVerificationResultDto>& results) const;
    std::vector<RoboSDP::Planning::Dto::PlanningVerificationResultDto> FromResultsJsonObject(
        const QJsonObject& jsonObject) const;

private:
    RoboSDP::Repository::IJsonRepository& m_repository;
};

} // namespace RoboSDP::Planning::Persistence
