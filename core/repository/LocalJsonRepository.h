#pragma once

#include "core/repository/IJsonRepository.h"

namespace RoboSDP::Repository
{

/**
 * @brief 本地 JSON 仓储实现。
 *
 * 该类负责在项目目录下读取和写入结构化 JSON 文件，
 * 为模块持久化提供统一入口。
 */
class LocalJsonRepository final : public IJsonRepository
{
public:
    RoboSDP::Errors::ErrorCode OpenProject(const QString& projectRootPath) override;

    RoboSDP::Errors::ErrorCode ReadDocument(
        const QString& relativePath,
        QJsonObject& document) const override;

    RoboSDP::Errors::ErrorCode WriteDocument(
        const QString& relativePath,
        const QJsonObject& document) override;

    QString ProjectRootPath() const override;

private:
    QString m_project_root_path;
};

} // namespace RoboSDP::Repository
