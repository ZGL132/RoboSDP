#pragma once

#include "modules/scheme/dto/SchemeAggregateDto.h"

#include <QString>

namespace RoboSDP::Scheme::Dto
{

/// SchemeSnapshot 元信息。
struct SchemeSnapshotMetaDto
{
    QString scheme_id = QStringLiteral("scheme_snapshot_001");
    QString name = QStringLiteral("阶段 1 方案快照");
    int version = 1;
    QString status = QStringLiteral("draft");
    QString source_project_root;
    QString created_at;
    QString updated_at;
};

/**
 * @brief 方案快照 DTO。
 *
 * 本对象面向系统内部保存/加载，记录一次聚合完成后的内部快照。
 * 与导出 JSON 职责区分开，避免同一结构同时承担内部态和交付态。
 */
struct SchemeSnapshotDto
{
    SchemeSnapshotMetaDto meta;
    SchemeAggregateDto aggregate;

    static SchemeSnapshotDto CreateDefault()
    {
        return {};
    }
};

} // namespace RoboSDP::Scheme::Dto
