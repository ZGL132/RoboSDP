#pragma once

#include "core/errors/ErrorCode.h"
#include "core/repository/IJsonRepository.h"
#include "modules/kinematics/dto/KinematicSolverResultDto.h"

#include <QJsonObject>

namespace RoboSDP::Kinematics::Persistence
{

/**
 * @brief Kinematics JSON 持久化组件。
 *
 * 当前实现把模型与最近一次 FK/IK 摘要写入 `kinematics/kinematic-model.json`，
 * 把基础工作空间采样结果写入 `kinematics/workspace-cache.json`。
 */
class KinematicJsonStorage
{
public:
    explicit KinematicJsonStorage(RoboSDP::Repository::IJsonRepository& repository);

    RoboSDP::Errors::ErrorCode SaveModel(
        const QString& projectRootPath,
        const RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& state) const;

    RoboSDP::Errors::ErrorCode LoadModel(
        const QString& projectRootPath,
        RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& state) const;

    RoboSDP::Errors::ErrorCode SaveWorkspaceCache(
        const QString& projectRootPath,
        const RoboSDP::Kinematics::Dto::WorkspaceResultDto& result) const;

    RoboSDP::Errors::ErrorCode LoadWorkspaceCache(
        const QString& projectRootPath,
        RoboSDP::Kinematics::Dto::WorkspaceResultDto& result) const;

    QString RelativeModelFilePath() const;
    QString RelativeWorkspaceFilePath() const;
    QString BuildAbsoluteModelFilePath(const QString& projectRootPath) const;
    QString BuildAbsoluteWorkspaceFilePath(const QString& projectRootPath) const;

private:
    QJsonObject ToModelJsonObject(const RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& state) const;
    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto FromModelJsonObject(const QJsonObject& jsonObject) const;

    QJsonObject ToWorkspaceJsonObject(const RoboSDP::Kinematics::Dto::WorkspaceResultDto& result) const;
    RoboSDP::Kinematics::Dto::WorkspaceResultDto FromWorkspaceJsonObject(const QJsonObject& jsonObject) const;

private:
    RoboSDP::Repository::IJsonRepository& m_repository;
};

} // namespace RoboSDP::Kinematics::Persistence
