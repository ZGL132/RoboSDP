#include "modules/requirement/service/KukaRequirementTemplateProvider.h"

#include <algorithm>
#include <array>

namespace RoboSDP::Requirement::Service
{

namespace
{

const std::array<KukaRequirementTemplate, 7> kKuka6AxisTemplates {{
    {QStringLiteral("KUKA KR 6 R900-2"), 6.0, 6.7, 0.901, 0.02},
    {QStringLiteral("KUKA KR 10 R1100-2"), 10.0, 11.1, 1.101, 0.02},
    {QStringLiteral("KUKA KR 20 R1810-2"), 20.0, 23.9, 1.813, 0.04},
    {QStringLiteral("KUKA KR 70 R2100"), 70.0, 70.0, 2.101, 0.05},
    {QStringLiteral("KUKA KR 120 R2700-2"), 120.0, 167.0, 2.701, 0.05},
    {QStringLiteral("KUKA KR 240 R2900-2"), 240.0, 240.0, 2.896, 0.06},
    {QStringLiteral("KUKA KR 500 R2830"), 500.0, 500.0, 2.826, 0.08}
}};

} // namespace

KukaRequirementTemplate KukaRequirementTemplateProvider::SelectByPayload(double ratedPayloadKg)
{
    const double requiredPayload = std::max(0.0, ratedPayloadKg);
    for (const auto& templateRecord : kKuka6AxisTemplates)
    {
        if (templateRecord.rated_payload_kg >= requiredPayload)
        {
            return templateRecord;
        }
    }

    return kKuka6AxisTemplates.back();
}

RoboSDP::Requirement::Dto::RequirementModelDto KukaRequirementTemplateProvider::BuildDefaultModel(
    double ratedPayloadKg)
{
    auto model = RoboSDP::Requirement::Dto::RequirementModelDto::CreateDefault();
    const auto templateRecord = SelectByPayload(ratedPayloadKg);

    model.project_meta.scenario_type = QStringLiteral("handling");
    model.project_meta.description =
        QStringLiteral("参考 %1 级别 6R 工业机器人任务需求默认值").arg(templateRecord.model_name);

    model.load_requirements.rated_payload = templateRecord.rated_payload_kg;
    model.load_requirements.max_payload = templateRecord.max_payload_kg;
    model.load_requirements.tool_mass = std::max(1.0, templateRecord.rated_payload_kg * 0.2);
    model.load_requirements.fixture_mass = std::max(0.5, templateRecord.rated_payload_kg * 0.1);
    model.load_requirements.payload_cog = {0.0, 0.0, std::min(0.25, 0.04 + templateRecord.reach_m * 0.04)};
    model.load_requirements.payload_inertia = {
        templateRecord.rated_payload_kg * 0.008,
        templateRecord.rated_payload_kg * 0.008,
        templateRecord.rated_payload_kg * 0.004,
        0.0,
        0.0,
        0.0};
    model.load_requirements.cable_drag_load = std::max(10.0, templateRecord.rated_payload_kg * 1.5);

    model.workspace_requirements.max_radius = templateRecord.reach_m;
    model.workspace_requirements.min_radius = std::max(0.12, templateRecord.reach_m * 0.16);
    model.workspace_requirements.max_height = templateRecord.reach_m * 1.05;
    model.workspace_requirements.min_height = -templateRecord.reach_m * 0.32;
    model.workspace_requirements.base_constraints.insert(QStringLiteral("base_mount_type"), QStringLiteral("floor"));
    model.workspace_requirements.base_constraints.insert(QStringLiteral("hollow_wrist_required"), false);
    model.workspace_requirements.base_constraints.insert(
        QStringLiteral("reserved_channel_diameter_mm"),
        std::clamp(templateRecord.rated_payload_kg * 0.8 + 25.0, 30.0, 100.0));

    if (!model.workspace_requirements.key_poses.empty())
    {
        auto& keyPose = model.workspace_requirements.key_poses.front();
        keyPose.pose = {
            templateRecord.reach_m * 0.58,
            0.0,
            templateRecord.reach_m * 0.40,
            180.0,
            0.0,
            180.0};
        keyPose.position_tol = std::max(0.1, templateRecord.repeatability_mm * 10.0);
        keyPose.orientation_tol = 0.5;
    }

    model.motion_requirements.max_linear_speed = templateRecord.rated_payload_kg <= 20.0 ? 2.0 : 1.2;
    model.motion_requirements.max_angular_speed = templateRecord.rated_payload_kg <= 20.0 ? 360.0 : 240.0;
    model.motion_requirements.max_acceleration = templateRecord.rated_payload_kg <= 20.0 ? 8.0 : 4.0;
    model.motion_requirements.max_angular_acceleration = templateRecord.rated_payload_kg <= 20.0 ? 720.0 : 420.0;
    model.motion_requirements.jerk_limit = templateRecord.rated_payload_kg <= 20.0 ? 100.0 : 60.0;
    model.motion_requirements.takt_time = templateRecord.rated_payload_kg <= 20.0 ? 1.2 : 2.0;

    model.accuracy_requirements.absolute_accuracy = std::max(0.2, templateRecord.repeatability_mm * 10.0);
    model.accuracy_requirements.repeatability = templateRecord.repeatability_mm;
    model.accuracy_requirements.tracking_accuracy = std::max(0.15, templateRecord.repeatability_mm * 6.0);
    model.accuracy_requirements.orientation_accuracy = 0.1;
    model.accuracy_requirements.tcp_position_tol = std::max(0.2, templateRecord.repeatability_mm * 10.0);
    model.accuracy_requirements.tcp_orientation_tol = 0.5;

    model.reliability_requirements.design_life = 80000.0;
    model.reliability_requirements.cycle_count = templateRecord.rated_payload_kg <= 20.0 ? 10000000 : 5000000;
    model.reliability_requirements.duty_cycle = 0.6;
    model.reliability_requirements.operating_hours_per_day = 16.0;
    model.reliability_requirements.mtbf_target = 20000.0;

    return model;
}

} // namespace RoboSDP::Requirement::Service
