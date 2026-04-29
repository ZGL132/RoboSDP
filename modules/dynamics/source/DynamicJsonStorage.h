#pragma once

#include "core/errors/ErrorCode.h"
#include "core/repository/IJsonRepository.h"
#include "modules/dynamics/dto/DynamicsResultDto.h"

#include <QJsonObject>

namespace RoboSDP::Dynamics::Persistence
{

/**
 * @brief Dynamics JSON 持久化组件。
 *
 * 当前实现按文档建议拆分为三类文件：
 * 1. `dynamics/dynamic-model.json`
 * 2. `dynamics/mass-properties.json`
 * 3. `dynamics/load-envelopes.json`
 */
class DynamicJsonStorage
{
public:
    explicit DynamicJsonStorage(RoboSDP::Repository::IJsonRepository& repository);

    RoboSDP::Errors::ErrorCode Save(
        const QString& projectRootPath,
        const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& state) const;

    RoboSDP::Errors::ErrorCode Load(
        const QString& projectRootPath,
        RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& state) const;

    QString RelativeModelFilePath() const;
    QString RelativeMassFilePath() const;
    QString RelativeEnvelopeFilePath() const;
    QString BuildAbsoluteModelFilePath(const QString& projectRootPath) const;
    QString BuildAbsoluteMassFilePath(const QString& projectRootPath) const;
    QString BuildAbsoluteEnvelopeFilePath(const QString& projectRootPath) const;

private:
    QJsonObject ToModelJsonObject(const RoboSDP::Dynamics::Dto::DynamicModelDto& model) const;
    void FillModelFromJsonObject(
        const QJsonObject& jsonObject,
        RoboSDP::Dynamics::Dto::DynamicModelDto& model) const;

    QJsonObject ToMassJsonObject(const RoboSDP::Dynamics::Dto::DynamicModelDto& model) const;
    void FillMassFromJsonObject(
        const QJsonObject& jsonObject,
        RoboSDP::Dynamics::Dto::DynamicModelDto& model) const;

    QJsonObject ToEnvelopeJsonObject(const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& state) const;
    void FillEnvelopeFromJsonObject(
        const QJsonObject& jsonObject,
        RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& state) const;

private:
    RoboSDP::Repository::IJsonRepository& m_repository;
};

} // namespace RoboSDP::Dynamics::Persistence
