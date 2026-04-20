#include "modules/requirement/service/RequirementService.h"

namespace RoboSDP::Requirement::Service
{

RequirementService::RequirementService(
    RoboSDP::Requirement::Persistence::RequirementJsonStorage& storage,
    RoboSDP::Requirement::Validation::RequirementValidator& validator,
    RoboSDP::Logging::ILogger* logger)
    : m_storage(storage)
    , m_validator(validator)
    , m_logger(logger)
{
}

RoboSDP::Requirement::Dto::RequirementModelDto RequirementService::CreateDefaultModel() const
{
    return RoboSDP::Requirement::Dto::RequirementModelDto::CreateDefault();
}

RoboSDP::Requirement::Validation::RequirementValidationResult RequirementService::Validate(
    const RoboSDP::Requirement::Dto::RequirementModelDto& model) const
{
    return m_validator.Validate(model);
}

RequirementSaveResult RequirementService::SaveDraft(
    const QString& projectRootPath,
    const RoboSDP::Requirement::Dto::RequirementModelDto& model) const
{
    RequirementSaveResult result;
    result.file_path = m_storage.BuildAbsoluteFilePath(projectRootPath);
    result.validation_result = m_validator.Validate(model);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法保存 Requirement 草稿。");
    }
    else
    {
        result.error_code = m_storage.Save(projectRootPath, model);
        if (result.IsSuccess())
        {
            result.message = QStringLiteral("Requirement 草稿已保存到：%1").arg(result.file_path);
        }
        else
        {
            result.message = QStringLiteral("Requirement 草稿保存失败：%1")
                                 .arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        }
    }

    if (m_logger != nullptr)
    {
        m_logger->Log(
            result.IsSuccess() ? RoboSDP::Logging::LogLevel::Info : RoboSDP::Logging::LogLevel::Error,
            result.message,
            result.error_code,
            {QStringLiteral("Requirement"), QStringLiteral("SaveDraft"), result.file_path});
    }

    return result;
}

RequirementLoadResult RequirementService::LoadDraft(const QString& projectRootPath) const
{
    RequirementLoadResult result;
    result.file_path = m_storage.BuildAbsoluteFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法加载 Requirement JSON。");
    }
    else
    {
        result.error_code = m_storage.Load(projectRootPath, result.model);
        if (result.IsSuccess())
        {
            result.validation_result = m_validator.Validate(result.model);
            result.message = QStringLiteral("Requirement 已从 JSON 重新加载：%1").arg(result.file_path);
        }
        else
        {
            result.message = QStringLiteral("Requirement 加载失败：%1")
                                 .arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        }
    }

    if (m_logger != nullptr)
    {
        m_logger->Log(
            result.IsSuccess() ? RoboSDP::Logging::LogLevel::Info : RoboSDP::Logging::LogLevel::Error,
            result.message,
            result.error_code,
            {QStringLiteral("Requirement"), QStringLiteral("LoadDraft"), result.file_path});
    }

    return result;
}

} // namespace RoboSDP::Requirement::Service
