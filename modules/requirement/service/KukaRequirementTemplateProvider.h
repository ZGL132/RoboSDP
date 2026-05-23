#pragma once

#include "modules/requirement/dto/RequirementModelDto.h"

namespace RoboSDP::Requirement::Service
{

struct KukaRequirementTemplate
{
    QString model_name;
    double rated_payload_kg = 0.0;
    double max_payload_kg = 0.0;
    double reach_m = 0.0;
    double repeatability_mm = 0.0;
};

class KukaRequirementTemplateProvider
{
public:
    static KukaRequirementTemplate SelectByPayload(double ratedPayloadKg);
    static RoboSDP::Requirement::Dto::RequirementModelDto BuildDefaultModel(double ratedPayloadKg);
};

} // namespace RoboSDP::Requirement::Service
