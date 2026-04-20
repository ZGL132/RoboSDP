#pragma once

#include "core/errors/ErrorCode.h"
#include "core/repository/IJsonRepository.h"
#include "modules/scheme/dto/SchemeExportDto.h"
#include "modules/scheme/dto/SchemeSnapshotDto.h"

#include <QJsonObject>

namespace RoboSDP::Scheme::Persistence
{

/**
 * @brief Scheme JSON 持久化组件。
 *
 * 该组件区分两类 JSON：
 * 1. `snapshots/scheme-snapshot.json`：系统内部快照；
 * 2. `exports/scheme-export.json`：导出交付摘要。
 * 两类 JSON 各自使用独立序列化结构，不共享同一 DTO 视图。
 */
class SchemeJsonStorage
{
public:
    explicit SchemeJsonStorage(RoboSDP::Repository::IJsonRepository& repository);

    RoboSDP::Errors::ErrorCode SaveSnapshot(
        const QString& projectRootPath,
        const RoboSDP::Scheme::Dto::SchemeSnapshotDto& snapshot) const;

    RoboSDP::Errors::ErrorCode LoadSnapshot(
        const QString& projectRootPath,
        RoboSDP::Scheme::Dto::SchemeSnapshotDto& snapshot) const;

    RoboSDP::Errors::ErrorCode SaveExportJson(
        const QString& projectRootPath,
        const RoboSDP::Scheme::Dto::SchemeExportDto& exportDto) const;

    QString RelativeSnapshotFilePath() const;
    QString RelativeExportFilePath() const;
    QString BuildAbsoluteSnapshotFilePath(const QString& projectRootPath) const;
    QString BuildAbsoluteExportFilePath(const QString& projectRootPath) const;

private:
    QJsonObject ToSnapshotJsonObject(const RoboSDP::Scheme::Dto::SchemeSnapshotDto& snapshot) const;
    RoboSDP::Scheme::Dto::SchemeSnapshotDto FromSnapshotJsonObject(const QJsonObject& jsonObject) const;
    QJsonObject ToExportJsonObject(const RoboSDP::Scheme::Dto::SchemeExportDto& exportDto) const;

private:
    RoboSDP::Repository::IJsonRepository& m_repository;
};

} // namespace RoboSDP::Scheme::Persistence
