#pragma once

#include <QString>

#include <vector>

namespace RoboSDP::Requirement::Service
{

struct RequirementTemplateSummary
{
    QString template_id;
    QString display_name;
    QString brand;
    QString model_name;
    QString description;
    double rated_payload_kg = 0.0;
    double max_payload_kg = 0.0;
    double reach_m = 0.0;
    double repeatability_mm = 0.0;
};

class RequirementTemplateCatalog
{
public:
    explicit RequirementTemplateCatalog(const QString& templateDirectory = {});

    std::vector<RequirementTemplateSummary> LoadSummaries() const;
    std::vector<RequirementTemplateSummary> FilterByRequirement(
        double ratedPayloadKg,
        double maxPayloadKg,
        double maxRadiusM,
        double repeatabilityMm) const;

private:
    QString ResolveTemplateDirectory() const;
    void LoadFromDirectory(const QString& directoryPath, std::vector<RequirementTemplateSummary>& summaries) const;

private:
    QString m_template_directory;
};

} // namespace RoboSDP::Requirement::Service
