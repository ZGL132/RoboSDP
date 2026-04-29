#pragma once

#include "core/errors/ErrorCode.h"
#include "modules/topology/dto/RobotTopologyModelDto.h"

#include <QJsonObject>
#include <QString>

#include <vector>

namespace RoboSDP::Topology::Service
{

/// 模板概览信息 DTO。
struct TopologyTemplateSummary
{
    QString template_id;
    QString display_name;
    QString description;
};

/// 模板完整记录 DTO。
struct TopologyTemplateRecord
{
    TopologyTemplateSummary summary;
    RoboSDP::Topology::Dto::RobotTopologyModelDto model;
    QString file_path;
};

/**
 * @brief 构型模板读取器。
 *
 * 当前只负责读取本地 JSON 模板文件，提供“列出模板”和“按 ID 读取模板”两种能力，
 * 不引入规则库或数据库层，保持模板化 6R 串联方案的最小实现。
 */
class TopologyTemplateLoader
{
public:
    explicit TopologyTemplateLoader(const QString& templateDirectory = {});

    RoboSDP::Errors::ErrorCode LoadTemplateSummaries(
        std::vector<TopologyTemplateSummary>& summaries) const;

    RoboSDP::Errors::ErrorCode LoadTemplates(
        std::vector<TopologyTemplateRecord>& templates) const;

    RoboSDP::Errors::ErrorCode LoadTemplate(
        const QString& templateId,
        TopologyTemplateRecord& templateRecord) const;

    QString TemplateDirectory() const;

private:
    QString ResolveTemplateDirectory() const;
    TopologyTemplateRecord ParseTemplateDocument(
        const QJsonObject& jsonObject,
        const QString& filePath) const;
    RoboSDP::Topology::Dto::RobotTopologyModelDto ParseModelObject(const QJsonObject& jsonObject) const;

private:
    QString m_template_directory;
};

} // namespace RoboSDP::Topology::Service
