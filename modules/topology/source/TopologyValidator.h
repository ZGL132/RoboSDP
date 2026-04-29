#pragma once

#include "modules/topology/dto/RobotTopologyModelDto.h"

#include <QList>
#include <QString>

namespace RoboSDP::Topology::Validation
{

/// Topology 校验问题严重级别。
enum class ValidationSeverity
{
    Info = 0,
    Warning,
    Error
};

/// Topology 单条校验问题。
struct ValidationIssue
{
    QString field;
    QString code;
    QString message_zh;
    ValidationSeverity severity = ValidationSeverity::Error;
};

/**
 * @brief Topology 校验结果集合。
 *
 * 结果对象与 Requirement 模块保持一致，便于 UI 复用展示逻辑，
 * 也便于后续把错误信息统一纳入日志与导出链路。
 */
struct TopologyValidationResult
{
    QList<ValidationIssue> issues;

    bool IsValid() const;
    int ErrorCount() const;
    int WarningCount() const;
};

/// 将校验级别转换为稳定字符串。
QString ToString(ValidationSeverity severity);

/**
 * @brief Topology 基础校验器。
 *
 * 当前阶段只校验模板化 6R 串联构型的基础结构正确性，
 * 不提前进入可达性、干涉、动力学或结构强度分析。
 */
class TopologyValidator
{
public:
    TopologyValidationResult Validate(const RoboSDP::Topology::Dto::RobotTopologyModelDto& model) const;
};

} // namespace RoboSDP::Topology::Validation
