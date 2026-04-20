#pragma once

#include <QString>

#include <vector>

namespace RoboSDP::Scheme::Dto
{

/// 导出文件中单个模块的摘要记录。
struct SchemeExportModuleRecordDto
{
    QString module_name;
    QString object_ref;
    bool available = false;
    QString summary_text;
};

/**
 * @brief 方案导出 DTO。
 *
 * 本对象面向导出与交付摘要，只保留外部可消费的信息，
 * 不携带内部保存所需的所有快照元数据。
 */
struct SchemeExportDto
{
    QString export_id = QStringLiteral("scheme_export_001");
    QString scheme_id;
    QString export_format = QStringLiteral("json");
    QString generated_at;
    QString output_file_path;
    bool success = false;
    QString message;
    int available_module_count = 0;
    QString executive_summary;
    std::vector<SchemeExportModuleRecordDto> module_records;

    static SchemeExportDto CreateDefault()
    {
        return {};
    }
};

} // namespace RoboSDP::Scheme::Dto
