#pragma once

#include "core/errors/ErrorCode.h"

#include <QString>

class QJsonObject;

namespace RoboSDP::Repository
{

/**
 * @brief JSON 项目仓储接口。
 *
 * 第一阶段项目采用“项目目录 + 结构化 JSON 文件”存储。
 * 第 0 阶段先统一接口边界，不引入具体业务模块读写逻辑。
 */
class IJsonRepository
{
public:
    virtual ~IJsonRepository() = default;

    /// 打开项目根目录，为后续 JSON 文档读写建立上下文。
    virtual RoboSDP::Errors::ErrorCode OpenProject(const QString& projectRootPath) = 0;

    /// 读取指定相对路径的 JSON 文档。
    virtual RoboSDP::Errors::ErrorCode ReadDocument(
        const QString& relativePath,
        QJsonObject& document) const = 0;

    /// 写入指定相对路径的 JSON 文档。
    virtual RoboSDP::Errors::ErrorCode WriteDocument(
        const QString& relativePath,
        const QJsonObject& document) = 0;

    /// 返回当前仓储绑定的项目根目录。
    virtual QString ProjectRootPath() const = 0;
};

} // namespace RoboSDP::Repository
