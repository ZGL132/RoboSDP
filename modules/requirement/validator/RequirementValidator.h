#pragma once

#include "modules/requirement/dto/RequirementModelDto.h"

#include <QList>
#include <QString>

namespace RoboSDP::Requirement::Validation
{

/// 校验结果严重级别。
enum class ValidationSeverity
{
    Info = 0,
    Warning,
    Error
};

/// Requirement 单条校验问题。
struct ValidationIssue
{
    QString field;
    QString code;
    QString message_zh;
    ValidationSeverity severity = ValidationSeverity::Error;
};

/**
 * @brief Requirement 校验结果集合。
 *
 * 该结构直接供 UI 展示，包含字段路径、中文错误说明和严重级别。
 */
struct RequirementValidationResult
{
    QList<ValidationIssue> issues;

    bool IsValid() const;
    int ErrorCount() const;
    int WarningCount() const;
};

/// 将严重级别转换为稳定字符串，便于界面展示。
QString ToString(ValidationSeverity severity);

/**
 * @brief Requirement 校验器。
 *
 * 本轮只实现字段合法性和最小模块级一致性校验，
 * 不引入工况高级推导或跨模块依赖检查。
 */
class RequirementValidator
{
public:
    RequirementValidationResult Validate(const RoboSDP::Requirement::Dto::RequirementModelDto& model) const;
};

} // namespace RoboSDP::Requirement::Validation
