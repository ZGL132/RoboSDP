#pragma once

#include "core/errors/ErrorCode.h"
#include "modules/selection/dto/MotorSelectionDto.h"
#include "modules/selection/dto/ReducerSelectionDto.h"

#include <QString>

#include <vector>

namespace RoboSDP::Selection::Catalog
{

/// 外部样例目录加载结果。
struct CatalogLoadResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString directory_path;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/**
 * @brief 外部 JSON 元件目录加载器。
 *
 * 当前只负责读取阶段 1 的少量样例电机与减速器数据，
 * 让目录扩充可以通过修改 JSON 完成，而不需要改动 C++ 代码。
 */
class JsonComponentCatalog
{
public:
    CatalogLoadResult LoadFromDirectory(const QString& catalogRootPath);

    const std::vector<RoboSDP::Selection::Dto::MotorCatalogItemDto>& Motors() const;
    const std::vector<RoboSDP::Selection::Dto::ReducerCatalogItemDto>& Reducers() const;
    QString LoadedDirectoryPath() const;

private:
    std::vector<RoboSDP::Selection::Dto::MotorCatalogItemDto> m_motors;
    std::vector<RoboSDP::Selection::Dto::ReducerCatalogItemDto> m_reducers;
    QString m_loaded_directory_path;
};

} // namespace RoboSDP::Selection::Catalog
