#pragma once

#include "core/errors/ErrorCode.h"
#include "core/repository/IJsonRepository.h"
#include "modules/selection/dto/DriveTrainSelectionDto.h"

#include <QJsonObject>

namespace RoboSDP::Selection::Persistence
{

/**
 * @brief 选型结果 JSON 持久化组件。
 *
 * 本轮按最小闭环拆成三个文件：
 * 1. `selection/motor-selection.json`
 * 2. `selection/reducer-selection.json`
 * 3. `selection/drivetrain-selection.json`
 */
class SelectionJsonStorage
{
public:
    explicit SelectionJsonStorage(RoboSDP::Repository::IJsonRepository& repository);

    RoboSDP::Errors::ErrorCode Save(
        const QString& projectRootPath,
        const RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const;

    RoboSDP::Errors::ErrorCode Load(
        const QString& projectRootPath,
        RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const;

    QString RelativeMotorFilePath() const;
    QString RelativeReducerFilePath() const;
    QString RelativeDriveTrainFilePath() const;
    QString BuildAbsoluteMotorFilePath(const QString& projectRootPath) const;
    QString BuildAbsoluteReducerFilePath(const QString& projectRootPath) const;
    QString BuildAbsoluteDriveTrainFilePath(const QString& projectRootPath) const;

private:
    QJsonObject ToMotorJsonObject(const RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const;
    QJsonObject ToReducerJsonObject(const RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const;
    QJsonObject ToDriveTrainJsonObject(const RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const;

    void FillMotorResultsFromJsonObject(
        const QJsonObject& jsonObject,
        RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const;
    void FillReducerResultsFromJsonObject(
        const QJsonObject& jsonObject,
        RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const;
    void FillDriveTrainFromJsonObject(
        const QJsonObject& jsonObject,
        RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const;

private:
    RoboSDP::Repository::IJsonRepository& m_repository;
};

} // namespace RoboSDP::Selection::Persistence
