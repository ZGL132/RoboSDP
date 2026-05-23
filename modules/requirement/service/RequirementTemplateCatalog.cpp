#include "modules/requirement/service/RequirementTemplateCatalog.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>

namespace RoboSDP::Requirement::Service
{

namespace
{

QString ReadString(const QJsonObject& object, const QString& key, const QString& defaultValue = {})
{
    return object.contains(key) ? object.value(key).toString(defaultValue) : defaultValue;
}

double ReadDouble(const QJsonObject& object, const QString& key, double defaultValue = 0.0)
{
    return object.contains(key) ? object.value(key).toDouble(defaultValue) : defaultValue;
}

double PositiveOverSpecPenalty(double requiredValue, double templateValue)
{
    if (requiredValue <= 0.0 || templateValue <= 0.0 || templateValue < requiredValue)
    {
        return 0.0;
    }

    return (templateValue - requiredValue) / requiredValue;
}

} // namespace

RequirementTemplateCatalog::RequirementTemplateCatalog(const QString& templateDirectory)
    : m_template_directory(templateDirectory)
{
}

std::vector<RequirementTemplateSummary> RequirementTemplateCatalog::LoadSummaries() const
{
    std::vector<RequirementTemplateSummary> summaries;
    LoadFromDirectory(ResolveTemplateDirectory(), summaries);
    std::sort(
        summaries.begin(),
        summaries.end(),
        [](const RequirementTemplateSummary& left, const RequirementTemplateSummary& right) {
            if (left.rated_payload_kg == right.rated_payload_kg)
            {
                return left.reach_m < right.reach_m;
            }
            return left.rated_payload_kg < right.rated_payload_kg;
        });
    return summaries;
}

std::vector<RequirementTemplateSummary> RequirementTemplateCatalog::FilterByRequirement(
    double ratedPayloadKg,
    double maxPayloadKg,
    double maxRadiusM,
    double repeatabilityMm) const
{
    auto summaries = LoadSummaries();
    const double requiredPayload = std::max(ratedPayloadKg, maxPayloadKg);

    std::stable_sort(
        summaries.begin(),
        summaries.end(),
        [requiredPayload, maxRadiusM, repeatabilityMm](
            const RequirementTemplateSummary& left,
            const RequirementTemplateSummary& right) {
            auto scoreTemplate = [requiredPayload, maxRadiusM, repeatabilityMm](
                                     const RequirementTemplateSummary& item) {
                double score = 0.0;
                if (requiredPayload > 0.0 && item.rated_payload_kg > 0.0)
                {
                    score += item.rated_payload_kg >= requiredPayload ? 100.0 : -100.0;
                    score -= PositiveOverSpecPenalty(requiredPayload, item.rated_payload_kg) * 20.0;
                }
                if (maxRadiusM > 0.0 && item.reach_m > 0.0)
                {
                    score += item.reach_m >= maxRadiusM ? 60.0 : -80.0;
                    score -= PositiveOverSpecPenalty(maxRadiusM, item.reach_m) * 12.0;
                }
                if (repeatabilityMm > 0.0 && item.repeatability_mm > 0.0)
                {
                    score += item.repeatability_mm <= repeatabilityMm ? 20.0 : -15.0;
                }
                return score;
            };

            return scoreTemplate(left) > scoreTemplate(right);
        });

    std::vector<RequirementTemplateSummary> filtered;
    for (const auto& summary : summaries)
    {
        const bool payloadOk =
            requiredPayload <= 0.0 ||
            summary.rated_payload_kg <= 0.0 ||
            summary.rated_payload_kg >= requiredPayload * 0.85;
        const bool reachOk =
            maxRadiusM <= 0.0 ||
            summary.reach_m <= 0.0 ||
            summary.reach_m >= maxRadiusM * 0.85;
        if (payloadOk && reachOk)
        {
            filtered.push_back(summary);
        }
        if (filtered.size() >= 12)
        {
            break;
        }
    }

    if (filtered.empty())
    {
        filtered = summaries;
        if (filtered.size() > 12)
        {
            filtered.resize(12);
        }
    }

    return filtered;
}

QString RequirementTemplateCatalog::ResolveTemplateDirectory() const
{
    if (!m_template_directory.trimmed().isEmpty())
    {
        return QDir::fromNativeSeparators(m_template_directory.trimmed());
    }

#ifdef ROBOSDP_SOURCE_DIR
    const QString sourceDirectory =
        QDir(QString::fromUtf8(ROBOSDP_SOURCE_DIR)).filePath(QStringLiteral("resources/topology/templates"));
    if (QDir(sourceDirectory).exists())
    {
        return sourceDirectory;
    }
#endif

    const QString currentDirectory =
        QDir::current().filePath(QStringLiteral("resources/topology/templates"));
    if (QDir(currentDirectory).exists())
    {
        return currentDirectory;
    }

    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("../resources/topology/templates"));
}

void RequirementTemplateCatalog::LoadFromDirectory(
    const QString& directoryPath,
    std::vector<RequirementTemplateSummary>& summaries) const
{
    const QDir directory(directoryPath);
    if (!directory.exists())
    {
        return;
    }

    const QFileInfoList fileInfos =
        directory.entryInfoList(QStringList {QStringLiteral("*.json")}, QDir::Files | QDir::Readable, QDir::Name);
    for (const QFileInfo& fileInfo : fileInfos)
    {
        QFile file(fileInfo.absoluteFilePath());
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            continue;
        }

        QJsonParseError parseError;
        const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
        file.close();
        if (parseError.error != QJsonParseError::NoError || !document.isObject())
        {
            continue;
        }

        const QJsonObject rootObject = document.object();
        const QJsonObject specsObject = rootObject.value(QStringLiteral("specs")).toObject();
        RequirementTemplateSummary summary;
        summary.template_id = ReadString(rootObject, QStringLiteral("template_id"));
        summary.display_name = ReadString(rootObject, QStringLiteral("display_name"));
        summary.description = ReadString(rootObject, QStringLiteral("description"));
        summary.brand = ReadString(rootObject, QStringLiteral("vendor"));
        summary.model_name = ReadString(rootObject, QStringLiteral("model_name"), summary.display_name);
        summary.rated_payload_kg = ReadDouble(specsObject, QStringLiteral("rated_payload_kg"));
        summary.max_payload_kg = ReadDouble(specsObject, QStringLiteral("max_payload_kg"));
        summary.reach_m = ReadDouble(specsObject, QStringLiteral("reach_m"));
        summary.repeatability_mm = ReadDouble(specsObject, QStringLiteral("repeatability_mm"));
        if (!summary.template_id.trimmed().isEmpty())
        {
            summaries.push_back(summary);
        }
    }

    const QFileInfoList childDirectories = directory.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    for (const QFileInfo& childDirectory : childDirectories)
    {
        LoadFromDirectory(childDirectory.absoluteFilePath(), summaries);
    }
}

} // namespace RoboSDP::Requirement::Service
