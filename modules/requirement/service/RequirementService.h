#pragma once

#include "core/errors/ErrorCode.h"
#include "core/logging/ILogger.h"
#include "modules/requirement/dto/RequirementModelDto.h"
#include "modules/requirement/persistence/RequirementJsonStorage.h"
#include "modules/requirement/validator/RequirementValidator.h"

namespace RoboSDP::Requirement::Service
{

/// Requirement 保存结果。
struct RequirementSaveResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString file_path;
    RoboSDP::Requirement::Validation::RequirementValidationResult validation_result;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// Requirement 加载结果。
struct RequirementLoadResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString file_path;
    RoboSDP::Requirement::Dto::RequirementModelDto model;
    RoboSDP::Requirement::Validation::RequirementValidationResult validation_result;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/**
 * @brief Requirement 服务层。
 *
 * 本轮服务层仅负责默认数据创建、校验编排以及保存/加载流程，
 * 不承载 Topology 或其他模块的业务逻辑。
 */
class RequirementService
{
public:
    RequirementService(
        RoboSDP::Requirement::Persistence::RequirementJsonStorage& storage,
        RoboSDP::Requirement::Validation::RequirementValidator& validator,
        RoboSDP::Logging::ILogger* logger = nullptr);

    RoboSDP::Requirement::Dto::RequirementModelDto CreateDefaultModel() const;

    RoboSDP::Requirement::Validation::RequirementValidationResult Validate(
        const RoboSDP::Requirement::Dto::RequirementModelDto& model) const;

    RequirementSaveResult SaveDraft(
        const QString& projectRootPath,
        const RoboSDP::Requirement::Dto::RequirementModelDto& model) const;

    RequirementLoadResult LoadDraft(const QString& projectRootPath) const;

private:
    RoboSDP::Requirement::Persistence::RequirementJsonStorage& m_storage;
    RoboSDP::Requirement::Validation::RequirementValidator& m_validator;
    RoboSDP::Logging::ILogger* m_logger = nullptr;
};

} // namespace RoboSDP::Requirement::Service
