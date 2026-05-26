#include "modules/topology/service/TopologyService.h"

#include "core/errors/ErrorCode.h"
#include "modules/requirement/dto/RequirementModelDto.h"

#include <QJsonObject>

#include <algorithm>
#include <cmath>

namespace RoboSDP::Topology::Service
{

namespace
{

bool ContainsTag(const std::vector<QString>& tags, const QString& expectedTag)
{
    for (const QString& tag : tags)
    {
        if (tag == expectedTag)
        {
            return true;
        }
    }

    return false;
}

QString NormalizeOptionalTemplateId(const QString& selectedTemplateId)
{
    const QString normalized = selectedTemplateId.trimmed();
    return normalized == QStringLiteral("__all__") ? QString() : normalized;
}

double PositiveDifferencePenalty(double requiredValue, double templateValue, double weight)
{
    if (requiredValue <= 0.0 || templateValue <= 0.0)
    {
        return 0.0;
    }

    if (templateValue >= requiredValue)
    {
        return std::min(weight, (templateValue - requiredValue) / requiredValue * weight * 0.25);
    }

    return (requiredValue - templateValue) / requiredValue * weight;
}

double Clamp(double value, double minimum, double maximum)
{
    return std::max(minimum, std::min(maximum, value));
}

double SumReachDimensions(const RoboSDP::Topology::Dto::RobotDefinitionDto& definition)
{
    return definition.shoulder_offset_m +
           definition.upper_arm_length_m +
           definition.forearm_length_m +
           definition.wrist_offset_m;
}

} // namespace

TopologyService::TopologyService(
    RoboSDP::Topology::Persistence::TopologyJsonStorage& storage,
    RoboSDP::Topology::Validation::TopologyValidator& validator,
    RoboSDP::Topology::Service::TopologyTemplateLoader& templateLoader,
    RoboSDP::Requirement::Persistence::RequirementJsonStorage& requirementStorage,
    RoboSDP::Logging::ILogger* logger)
    : m_storage(storage)
    , m_validator(validator)
    , m_template_loader(templateLoader)
    , m_requirement_storage(requirementStorage)
    , m_logger(logger)
{
}

RoboSDP::Topology::Dto::RobotTopologyModelDto TopologyService::CreateDefaultModel() const
{
    return RoboSDP::Topology::Dto::RobotTopologyModelDto::CreateDefault();
}

RoboSDP::Topology::Dto::TopologyWorkspaceStateDto TopologyService::CreateDefaultState() const
{
    return RoboSDP::Topology::Dto::TopologyWorkspaceStateDto::CreateDefault();
}

RoboSDP::Topology::Validation::TopologyValidationResult TopologyService::Validate(
    const RoboSDP::Topology::Dto::RobotTopologyModelDto& model) const
{
    return m_validator.Validate(model);
}

std::vector<RoboSDP::Topology::Service::TopologyTemplateSummary> TopologyService::ListTemplates() const
{
    std::vector<RoboSDP::Topology::Service::TopologyTemplateSummary> summaries;
    m_template_loader.LoadTemplateSummaries(summaries);
    return summaries;
}

TopologyGenerateResult TopologyService::GenerateCandidatesFromRequirement(
    const QString& projectRootPath,
    const QString& selectedTemplateId) const
{
    TopologyGenerateResult result;
    result.state = CreateDefaultState();
    result.requirement_file_path = m_requirement_storage.BuildAbsoluteFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法读取 Requirement 并生成 Topology。");
        return result;
    }

    RequirementTopologyConstraints constraints;
    QString requirementFailureMessage;
    if (!TryLoadRequirementConstraints(
            projectRootPath,
            constraints,
            result.requirement_file_path,
            requirementFailureMessage))
    {
        result.error_code = RoboSDP::Errors::ErrorCode::RepositoryDocumentNotFound;
        result.message = requirementFailureMessage;

        if (m_logger != nullptr)
        {
            m_logger->Log(
                RoboSDP::Logging::LogLevel::Error,
                result.message,
                result.error_code,
                {QStringLiteral("Topology"), QStringLiteral("GenerateCandidates"), result.requirement_file_path});
        }

        return result;
    }

    std::vector<TopologyTemplateRecord> templates;
    QString normalizedTemplateId = NormalizeOptionalTemplateId(selectedTemplateId);
    if (normalizedTemplateId.isEmpty())
    {
        normalizedTemplateId = NormalizeOptionalTemplateId(constraints.selected_template_id);
    }

    if (normalizedTemplateId.isEmpty())
    {
        result.error_code = m_template_loader.LoadTemplates(templates);
    }
    else
    {
        TopologyTemplateRecord templateRecord;
        result.error_code = m_template_loader.LoadTemplate(normalizedTemplateId, templateRecord);
        if (result.error_code == RoboSDP::Errors::ErrorCode::Ok)
        {
            templates.push_back(templateRecord);
        }
    }

    if (result.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.message = QStringLiteral("构型模板读取失败：%1")
                             .arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        return result;
    }

    result.state.candidates.clear();
    for (const auto& templateRecord : templates)
    {
        result.state.candidates.push_back(BuildCandidate(templateRecord, constraints));
    }

    if (result.state.candidates.empty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::RepositoryDocumentNotFound;
        result.message = QStringLiteral("未找到可用于生成候选构型的模板。");
        return result;
    }

    result.state.recommendation = BuildRecommendation(result.state.candidates, true, true);

    for (auto& candidate : result.state.candidates)
    {
        candidate.model.meta.status =
            candidate.candidate_id == result.state.recommendation.recommended_candidate_id
                ? QStringLiteral("ready")
                : QStringLiteral("draft");
    }

    for (const auto& candidate : result.state.candidates)
    {
        if (candidate.candidate_id == result.state.recommendation.recommended_candidate_id)
        {
            result.state.current_model = candidate.model;
            break;
        }
    }

    result.validation_result = m_validator.Validate(result.state.current_model);
    result.message = QStringLiteral("已基于 Requirement 约束生成 %1 个候选构型，并给出规则推荐。")
                         .arg(result.state.candidates.size());

    if (m_logger != nullptr)
    {
        m_logger->Log(
            RoboSDP::Logging::LogLevel::Info,
            result.message,
            result.error_code,
            {QStringLiteral("Topology"), QStringLiteral("GenerateCandidates"), result.requirement_file_path});
    }

    return result;
}

TopologySaveResult TopologyService::SaveDraft(
    const QString& projectRootPath,
    const RoboSDP::Topology::Dto::TopologyWorkspaceStateDto& state) const
{
    TopologySaveResult result;
    result.file_path = m_storage.BuildAbsoluteFilePath(projectRootPath);
    result.validation_result = m_validator.Validate(state.current_model);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法保存 Topology 草稿。");
    }
    else
    {
        result.error_code = m_storage.Save(projectRootPath, state);
        result.message = result.IsSuccess()
            ? QStringLiteral("Topology 草稿已保存到：%1").arg(result.file_path)
            : QStringLiteral("Topology 草稿保存失败：%1")
                  .arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
    }

    if (m_logger != nullptr)
    {
        m_logger->Log(
            result.IsSuccess() ? RoboSDP::Logging::LogLevel::Info : RoboSDP::Logging::LogLevel::Error,
            result.message,
            result.error_code,
            {QStringLiteral("Topology"), QStringLiteral("SaveDraft"), result.file_path});
    }

    return result;
}

TopologyLoadResult TopologyService::LoadDraft(const QString& projectRootPath) const
{
    TopologyLoadResult result;
    result.file_path = m_storage.BuildAbsoluteFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法加载 Topology JSON。");
    }
    else
    {
        result.error_code = m_storage.Load(projectRootPath, result.state);
        if (result.IsSuccess())
        {
            result.validation_result = m_validator.Validate(result.state.current_model);
            result.message = QStringLiteral("Topology 已从 JSON 重新加载：%1").arg(result.file_path);
        }
        else
        {
            result.message = QStringLiteral("Topology 加载失败：%1")
                                 .arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        }
    }

    if (m_logger != nullptr)
    {
        m_logger->Log(
            result.IsSuccess() ? RoboSDP::Logging::LogLevel::Info : RoboSDP::Logging::LogLevel::Error,
            result.message,
            result.error_code,
            {QStringLiteral("Topology"), QStringLiteral("LoadDraft"), result.file_path});
    }

    return result;
}

bool TopologyService::TryLoadRequirementConstraints(
    const QString& projectRootPath,
    RequirementTopologyConstraints& constraints,
    QString& requirementFilePath,
    QString& failureMessage) const
{
    RoboSDP::Requirement::Dto::RequirementModelDto requirementModel;
    requirementFilePath = m_requirement_storage.BuildAbsoluteFilePath(projectRootPath);
    const RoboSDP::Errors::ErrorCode loadError =
        m_requirement_storage.Load(projectRootPath, requirementModel);

    if (loadError != RoboSDP::Errors::ErrorCode::Ok)
    {
        failureMessage = QStringLiteral("读取 Requirement 约束失败，请先在 Requirement 页面保存草稿：%1")
                             .arg(RoboSDP::Errors::ToChineseMessage(loadError));
        return false;
    }

    constraints.requirement_name = requirementModel.project_meta.project_name.trimmed();
    constraints.scenario_type = requirementModel.project_meta.scenario_type.trimmed();
    constraints.selected_template_id = requirementModel.project_meta.selected_template_id.trimmed();
    constraints.rated_payload_kg = requirementModel.load_requirements.rated_payload;
    constraints.max_payload_kg = requirementModel.load_requirements.max_payload;
    constraints.max_radius_m = requirementModel.workspace_requirements.max_radius;
    constraints.max_height_m = requirementModel.workspace_requirements.max_height;
    constraints.min_height_m = requirementModel.workspace_requirements.min_height;
    constraints.repeatability_mm = requirementModel.accuracy_requirements.repeatability;

    const QJsonObject baseConstraints = requirementModel.workspace_requirements.base_constraints;
    if (baseConstraints.contains(QStringLiteral("base_mount_type")))
    {
        constraints.preferred_base_mount_type =
            baseConstraints.value(QStringLiteral("base_mount_type")).toString();
        constraints.base_mount_specified = !constraints.preferred_base_mount_type.trimmed().isEmpty();
    }

    if (baseConstraints.contains(QStringLiteral("hollow_wrist_required")))
    {
        constraints.hollow_wrist_required =
            baseConstraints.value(QStringLiteral("hollow_wrist_required")).toBool(false);
        constraints.hollow_wrist_specified = true;
    }

    if (baseConstraints.contains(QStringLiteral("reserved_channel_diameter_mm")))
    {
        constraints.reserved_channel_diameter_mm =
            baseConstraints.value(QStringLiteral("reserved_channel_diameter_mm")).toDouble(0.0);
        constraints.reserved_channel_specified = true;
    }

    return true;
}

RoboSDP::Topology::Dto::TopologyCandidateDto TopologyService::BuildCandidate(
    const RoboSDP::Topology::Service::TopologyTemplateRecord& templateRecord,
    const RequirementTopologyConstraints& constraints) const
{
    using namespace RoboSDP::Topology::Dto;

    TopologyCandidateDto candidate;
    candidate.candidate_id = QStringLiteral("candidate_%1").arg(templateRecord.summary.template_id);
    candidate.title = templateRecord.summary.display_name;
    candidate.template_id = templateRecord.summary.template_id;
    candidate.model = templateRecord.model;
    candidate.model.meta.topology_id = QStringLiteral("topology_%1").arg(templateRecord.summary.template_id);
    candidate.model.meta.name = templateRecord.summary.display_name;
    candidate.model.meta.source = QStringLiteral("template");
    candidate.model.meta.template_id = templateRecord.summary.template_id;
    candidate.model.meta.status = QStringLiteral("draft");
    candidate.model.meta.requirement_ref = constraints.requirement_name;

    ApplyRequirementDrivenKinematicDimensions(candidate.model, templateRecord.summary, constraints);

    candidate.score += 20.0;
    candidate.recommendation_reason.push_back(QStringLiteral("候选来自已登记的 6R 串联构型模板。"));
    candidate.recommendation_reason.push_back(
        QStringLiteral("已根据 Requirement 的工作空间、负载、精度和安装方式估算初始运动学关键尺寸。"));

    const double requiredPayload = std::max(constraints.rated_payload_kg, constraints.max_payload_kg);
    if (templateRecord.summary.rated_payload_kg > 0.0 && requiredPayload > 0.0)
    {
        if (templateRecord.summary.rated_payload_kg >= constraints.rated_payload_kg &&
            (constraints.max_payload_kg <= 0.0 || templateRecord.summary.max_payload_kg >= constraints.max_payload_kg))
        {
            candidate.score += 45.0;
            candidate.recommendation_reason.push_back(QStringLiteral("参考型号负载等级覆盖当前任务需求。"));
        }
        else
        {
            candidate.score -= 80.0;
            candidate.recommendation_reason.push_back(QStringLiteral("参考型号负载等级低于当前任务需求，降级为备选。"));
        }

        candidate.score -= PositiveDifferencePenalty(requiredPayload, templateRecord.summary.rated_payload_kg, 18.0);
    }

    if (templateRecord.summary.reach_m > 0.0 && constraints.max_radius_m > 0.0)
    {
        if (templateRecord.summary.reach_m >= constraints.max_radius_m)
        {
            candidate.score += 30.0;
            candidate.recommendation_reason.push_back(QStringLiteral("参考型号臂展覆盖当前工作空间半径。"));
        }
        else
        {
            candidate.score -= 50.0;
            candidate.recommendation_reason.push_back(QStringLiteral("参考型号臂展小于当前工作空间半径。"));
        }
        candidate.score -= PositiveDifferencePenalty(constraints.max_radius_m, templateRecord.summary.reach_m, 12.0);
    }

    if (templateRecord.summary.repeatability_mm > 0.0 && constraints.repeatability_mm > 0.0)
    {
        if (templateRecord.summary.repeatability_mm <= constraints.repeatability_mm)
        {
            candidate.score += 15.0;
            candidate.recommendation_reason.push_back(QStringLiteral("参考型号重复定位精度满足当前精度需求。"));
        }
        else
        {
            candidate.score -= 20.0;
            candidate.recommendation_reason.push_back(QStringLiteral("参考型号重复定位精度弱于当前需求。"));
        }
    }

    if (!constraints.scenario_type.trimmed().isEmpty() &&
        !ContainsTag(candidate.model.robot_definition.application_tags, constraints.scenario_type))
    {
        candidate.model.robot_definition.application_tags.push_back(constraints.scenario_type);
    }

    if (constraints.reserved_channel_specified &&
        constraints.reserved_channel_diameter_mm > candidate.model.layout.reserved_channel_diameter_mm)
    {
        candidate.model.layout.reserved_channel_diameter_mm = constraints.reserved_channel_diameter_mm;
        candidate.recommendation_reason.push_back(QStringLiteral("已根据 Requirement 预留通道约束抬高模板通道直径。"));
    }

    if (constraints.base_mount_specified)
    {
        if (candidate.model.robot_definition.base_mount_type == constraints.preferred_base_mount_type)
        {
            candidate.score += 35.0;
            candidate.recommendation_reason.push_back(QStringLiteral("基座安装方式与 Requirement 约束匹配。"));
        }
        else
        {
            candidate.recommendation_reason.push_back(
                QStringLiteral("基座安装方式与 Requirement 偏好不一致，保留为低优先级候选。"));
        }
    }
    else
    {
        candidate.score += 20.0;
        candidate.recommendation_reason.push_back(
            QStringLiteral("Requirement 未指定基座安装方式，模板默认方案可直接参与比较。"));
    }

    if (constraints.hollow_wrist_specified)
    {
        if (!constraints.hollow_wrist_required || candidate.model.layout.hollow_wrist_required)
        {
            candidate.score += 25.0;
            candidate.recommendation_reason.push_back(QStringLiteral("中空腕约束满足当前 Requirement。"));
        }
        else
        {
            candidate.recommendation_reason.push_back(QStringLiteral("当前模板不满足 Requirement 的中空腕需求。"));
        }
    }
    else
    {
        candidate.score += 10.0;
        candidate.recommendation_reason.push_back(QStringLiteral("Requirement 未指定中空腕约束，按模板默认腕部形式保留。"));
    }

    if (!constraints.scenario_type.trimmed().isEmpty() &&
        ContainsTag(candidate.model.robot_definition.application_tags, constraints.scenario_type))
    {
        candidate.score += 20.0;
        candidate.recommendation_reason.push_back(QStringLiteral("模板适用场景包含当前 Requirement 场景标签。"));
    }

    const RoboSDP::Topology::Validation::TopologyValidationResult validationResult =
        m_validator.Validate(candidate.model);
    candidate.matches_requirement = validationResult.IsValid();
    if (validationResult.IsValid())
    {
        candidate.score += 10.0;
        candidate.recommendation_reason.push_back(QStringLiteral("拓扑结构校验通过。"));
        candidate.model.meta.status = QStringLiteral("ready");
    }
    else
    {
        candidate.model.meta.status = QStringLiteral("invalid");
        candidate.recommendation_reason.push_back(QStringLiteral("拓扑结构存在校验问题，仅作为保留候选。"));
    }

    return candidate;
}

void TopologyService::ApplyRequirementDrivenKinematicDimensions(
    RoboSDP::Topology::Dto::RobotTopologyModelDto& model,
    const RoboSDP::Topology::Service::TopologyTemplateSummary& templateSummary,
    const RequirementTopologyConstraints& constraints) const
{
    auto& definition = model.robot_definition;
    const double requiredPayload = std::max(constraints.rated_payload_kg, constraints.max_payload_kg);
    const double templateDimensionSum = SumReachDimensions(definition);
    const double templateReach = templateSummary.reach_m > 0.0 ? templateSummary.reach_m : templateDimensionSum;
    const double reachScaleFromTemplate =
        templateReach > 0.0 && templateDimensionSum > 0.0 ? templateDimensionSum / templateReach : 0.86;

    double targetReach = constraints.max_radius_m > 0.0 ? constraints.max_radius_m : templateReach;
    if (templateSummary.reach_m > 0.0 && constraints.max_radius_m > 0.0)
    {
        targetReach = std::min(templateSummary.reach_m, std::max(constraints.max_radius_m * 1.02, constraints.max_radius_m));
    }
    targetReach = Clamp(targetReach, 0.35, 6.0);

    double targetDimensionSum = Clamp(targetReach * reachScaleFromTemplate, 0.30, 5.5);
    if (requiredPayload >= 100.0)
    {
        targetDimensionSum *= 0.98;
    }
    if (constraints.repeatability_mm > 0.0 && constraints.repeatability_mm <= 0.03)
    {
        targetDimensionSum *= 0.97;
    }

    const double currentSum = templateDimensionSum > 0.0 ? templateDimensionSum : 1.0;
    double shoulderRatio = definition.shoulder_offset_m / currentSum;
    double upperRatio = definition.upper_arm_length_m / currentSum;
    double forearmRatio = definition.forearm_length_m / currentSum;
    double wristRatio = definition.wrist_offset_m / currentSum;
    if (shoulderRatio <= 0.0 || upperRatio <= 0.0 || forearmRatio <= 0.0 || wristRatio <= 0.0)
    {
        shoulderRatio = 0.11;
        upperRatio = 0.43;
        forearmRatio = 0.36;
        wristRatio = 0.10;
    }

    if (requiredPayload >= 100.0)
    {
        wristRatio *= 0.85;
        shoulderRatio *= 1.08;
    }
    if (constraints.repeatability_mm > 0.0 && constraints.repeatability_mm <= 0.03)
    {
        wristRatio *= 0.90;
        upperRatio *= 0.98;
        forearmRatio *= 0.98;
    }

    const double ratioSum = shoulderRatio + upperRatio + forearmRatio + wristRatio;
    shoulderRatio /= ratioSum;
    upperRatio /= ratioSum;
    forearmRatio /= ratioSum;
    wristRatio /= ratioSum;

    definition.shoulder_offset_m = Clamp(targetDimensionSum * shoulderRatio, 0.04, 0.80);
    definition.upper_arm_length_m = Clamp(targetDimensionSum * upperRatio, 0.12, 2.20);
    definition.forearm_length_m = Clamp(targetDimensionSum * forearmRatio, 0.12, 2.20);
    definition.wrist_offset_m = Clamp(targetDimensionSum * wristRatio, 0.04, 0.60);

    if (constraints.max_height_m > constraints.min_height_m)
    {
        const double heightSpan = constraints.max_height_m - constraints.min_height_m;
        const double heightDrivenBase = Clamp(
            constraints.max_height_m * 0.16 + std::max(0.0, -constraints.min_height_m) * 0.10 + heightSpan * 0.04,
            0.18,
            targetReach * 0.45);
        definition.base_height_m = std::max(definition.base_height_m, heightDrivenBase);
    }

    if (constraints.base_mount_specified)
    {
        definition.base_mount_type = constraints.preferred_base_mount_type;
        if (constraints.preferred_base_mount_type == QStringLiteral("wall"))
        {
            definition.base_orientation = {0.0, 90.0, 0.0};
            definition.base_height_m = std::max(definition.base_height_m, targetReach * 0.35);
        }
        else if (constraints.preferred_base_mount_type == QStringLiteral("ceiling"))
        {
            definition.base_orientation = {180.0, 0.0, 0.0};
            definition.base_height_m = std::max(definition.base_height_m, std::max(0.0, constraints.max_height_m));
        }
        else if (constraints.preferred_base_mount_type == QStringLiteral("pedestal"))
        {
            definition.base_orientation = {0.0, 0.0, 0.0};
            definition.base_height_m = std::max(definition.base_height_m, targetReach * 0.28);
        }
        else
        {
            definition.base_orientation = {0.0, 0.0, 0.0};
        }
    }

    if (definition.j1_rotation_range_deg[0] >= definition.j1_rotation_range_deg[1])
    {
        definition.j1_rotation_range_deg = {-185.0, 185.0};
    }

    model.meta.remarks = QStringLiteral(
        "初始尺寸由 Requirement 工作空间、负载、精度和安装方式估算；模板尺寸用于比例参考，非厂商标定 DH 参数。");
}

RoboSDP::Topology::Dto::TopologyRecommendationDto TopologyService::BuildRecommendation(
    const std::vector<RoboSDP::Topology::Dto::TopologyCandidateDto>& candidates,
    bool requirementLoaded,
    bool requirementConstraintsApplied) const
{
    using namespace RoboSDP::Topology::Dto;

    TopologyRecommendationDto recommendation;
    recommendation.requirement_loaded = requirementLoaded;
    recommendation.requirement_constraints_applied = requirementConstraintsApplied;

    if (candidates.empty())
    {
        recommendation.recommendation_reason.push_back(QStringLiteral("当前没有可推荐的候选构型。"));
        return recommendation;
    }

    const TopologyCandidateDto* bestCandidate = &candidates.front();
    for (const auto& candidate : candidates)
    {
        if (candidate.score > bestCandidate->score)
        {
            bestCandidate = &candidate;
        }
    }

    recommendation.recommended_candidate_id = bestCandidate->candidate_id;
    recommendation.recommended_topology_id = bestCandidate->model.meta.topology_id;
    recommendation.recommended_template_id = bestCandidate->template_id;
    recommendation.combined_score = bestCandidate->score;
    recommendation.recommendation_reason = bestCandidate->recommendation_reason;
    recommendation.recommendation_reason.push_back(
        QStringLiteral("当前推荐基于参考型号负载、臂展、重复定位精度、安装方式、需求驱动尺寸估算和基础校验的规则评分。"));

    return recommendation;
}

} // namespace RoboSDP::Topology::Service
