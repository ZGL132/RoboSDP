#pragma once

#include "core/errors/ErrorCode.h"
#include "core/logging/ILogger.h"
#include "modules/scheme/dto/SchemeExportDto.h"
#include "modules/scheme/dto/SchemeSnapshotDto.h"
#include "modules/scheme/persistence/SchemeJsonStorage.h"

namespace RoboSDP::Scheme::Service
{

/// 方案导出结果。
struct SchemeExportResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    RoboSDP::Scheme::Dto::SchemeExportDto export_dto;
    QString export_file_path;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/**
 * @brief 方案导出服务。
 *
 * 本轮只实现 JSON 导出骨架，并预留 Markdown 导出接口。
 * 真实 Markdown 模板与 PDF 导出能力留到后续轮次。
 */
class SchemeExportService
{
public:
    explicit SchemeExportService(
        RoboSDP::Scheme::Persistence::SchemeJsonStorage& storage,
        RoboSDP::Logging::ILogger* logger = nullptr);

    SchemeExportResult ExportAsJson(
        const QString& projectRootPath,
        const RoboSDP::Scheme::Dto::SchemeSnapshotDto& snapshot) const;

    SchemeExportResult ExportAsMarkdown(
        const QString& projectRootPath,
        const RoboSDP::Scheme::Dto::SchemeSnapshotDto& snapshot) const;

private:
    RoboSDP::Scheme::Dto::SchemeExportDto BuildJsonExportDto(
        const QString& projectRootPath,
        const RoboSDP::Scheme::Dto::SchemeSnapshotDto& snapshot) const;

private:
    RoboSDP::Scheme::Persistence::SchemeJsonStorage& m_storage;
    RoboSDP::Logging::ILogger* m_logger = nullptr;
};

} // namespace RoboSDP::Scheme::Service
