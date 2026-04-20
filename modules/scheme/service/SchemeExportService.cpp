#include "modules/scheme/service/SchemeExportService.h"

#include <QDateTime>

namespace RoboSDP::Scheme::Service
{

namespace
{

QString BuildTimestamp()
{
    return QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
}

void AppendModuleRecord(
    std::vector<RoboSDP::Scheme::Dto::SchemeExportModuleRecordDto>& records,
    const QString& moduleName,
    const QString& objectRef,
    bool available,
    const QString& summaryText)
{
    RoboSDP::Scheme::Dto::SchemeExportModuleRecordDto record;
    record.module_name = moduleName;
    record.object_ref = objectRef;
    record.available = available;
    record.summary_text = summaryText;
    records.push_back(record);
}

} // namespace

SchemeExportService::SchemeExportService(
    RoboSDP::Scheme::Persistence::SchemeJsonStorage& storage,
    RoboSDP::Logging::ILogger* logger)
    : m_storage(storage)
    , m_logger(logger)
{
}

SchemeExportResult SchemeExportService::ExportAsJson(
    const QString& projectRootPath,
    const RoboSDP::Scheme::Dto::SchemeSnapshotDto& snapshot) const
{
    SchemeExportResult result;
    result.export_dto = BuildJsonExportDto(projectRootPath, snapshot);
    result.export_dto.success = true;
    result.export_dto.message = QStringLiteral("JSON 导出已生成。");
    result.export_file_path = result.export_dto.output_file_path;
    result.error_code = m_storage.SaveExportJson(projectRootPath, result.export_dto);

    if (result.error_code == RoboSDP::Errors::ErrorCode::Ok)
    {
        result.message = QStringLiteral("方案导出 JSON 已生成。");
    }
    else
    {
        result.export_dto.success = false;
        result.export_dto.message = RoboSDP::Errors::ToChineseMessage(result.error_code);
        result.message = result.export_dto.message;
    }

    return result;
}

SchemeExportResult SchemeExportService::ExportAsMarkdown(
    const QString& projectRootPath,
    const RoboSDP::Scheme::Dto::SchemeSnapshotDto& snapshot) const
{
    Q_UNUSED(projectRootPath);
    Q_UNUSED(snapshot);

    SchemeExportResult result;
    result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
    result.export_dto.export_format = QStringLiteral("markdown");
    result.message = QStringLiteral("Markdown 导出接口已预留，本轮未实现。");
    result.export_dto.message = result.message;
    return result;
}

RoboSDP::Scheme::Dto::SchemeExportDto SchemeExportService::BuildJsonExportDto(
    const QString& projectRootPath,
    const RoboSDP::Scheme::Dto::SchemeSnapshotDto& snapshot) const
{
    RoboSDP::Scheme::Dto::SchemeExportDto exportDto =
        RoboSDP::Scheme::Dto::SchemeExportDto::CreateDefault();
    exportDto.export_id = QStringLiteral("scheme_export_%1").arg(QDateTime::currentSecsSinceEpoch());
    exportDto.scheme_id = snapshot.meta.scheme_id;
    exportDto.export_format = QStringLiteral("json");
    exportDto.generated_at = BuildTimestamp();
    exportDto.output_file_path = m_storage.BuildAbsoluteExportFilePath(projectRootPath);
    exportDto.available_module_count = snapshot.aggregate.available_module_count;
    exportDto.executive_summary = snapshot.aggregate.completeness_summary;

    AppendModuleRecord(
        exportDto.module_records,
        QStringLiteral("Requirement"),
        snapshot.aggregate.refs.requirement_ref,
        snapshot.aggregate.requirement_summary.available,
        snapshot.aggregate.requirement_summary.summary_text);
    AppendModuleRecord(
        exportDto.module_records,
        QStringLiteral("Topology"),
        snapshot.aggregate.refs.topology_ref,
        snapshot.aggregate.topology_summary.available,
        snapshot.aggregate.topology_summary.summary_text);
    AppendModuleRecord(
        exportDto.module_records,
        QStringLiteral("Kinematics"),
        snapshot.aggregate.refs.kinematic_ref,
        snapshot.aggregate.kinematics_summary.available,
        snapshot.aggregate.kinematics_summary.summary_text);
    AppendModuleRecord(
        exportDto.module_records,
        QStringLiteral("Dynamics"),
        snapshot.aggregate.refs.dynamic_ref,
        snapshot.aggregate.dynamics_summary.available,
        snapshot.aggregate.dynamics_summary.summary_text);
    AppendModuleRecord(
        exportDto.module_records,
        QStringLiteral("Selection"),
        snapshot.aggregate.refs.selection_ref,
        snapshot.aggregate.selection_summary.available,
        snapshot.aggregate.selection_summary.summary_text);
    AppendModuleRecord(
        exportDto.module_records,
        QStringLiteral("Planning"),
        snapshot.aggregate.refs.planning_result_ref.isEmpty()
            ? snapshot.aggregate.refs.planning_scene_ref
            : snapshot.aggregate.refs.planning_result_ref,
        snapshot.aggregate.planning_summary.available,
        snapshot.aggregate.planning_summary.summary_text);

    return exportDto;
}

} // namespace RoboSDP::Scheme::Service
