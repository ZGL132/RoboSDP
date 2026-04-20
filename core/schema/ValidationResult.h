#pragma once

#include "core/schema/ValidationMessage.h"

#include <QList>

namespace RoboSDP::Schema
{

/**
 * @brief 统一校验结果集合。
 *
 * 该结构只负责承载统一格式的消息，不耦合具体模块 DTO，
 * 方便 Requirement / Topology 现有 validator 与新模块最小规则共用一套输出口径。
 */
struct ValidationResult
{
    QList<ValidationMessage> messages;

    void Add(
        const QString& field,
        const QString& code,
        const QString& messageZh,
        ValidationSeverity severity = ValidationSeverity::Error)
    {
        messages.push_back({field, code, messageZh, severity});
    }

    void Append(const ValidationResult& other)
    {
        for (const auto& message : other.messages)
        {
            messages.push_back(message);
        }
    }

    bool IsValid() const
    {
        return ErrorCount() == 0;
    }

    int ErrorCount() const
    {
        int count = 0;
        for (const auto& message : messages)
        {
            if (message.severity == ValidationSeverity::Error)
            {
                ++count;
            }
        }
        return count;
    }

    int WarningCount() const
    {
        int count = 0;
        for (const auto& message : messages)
        {
            if (message.severity == ValidationSeverity::Warning)
            {
                ++count;
            }
        }
        return count;
    }
};

} // namespace RoboSDP::Schema
