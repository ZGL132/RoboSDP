#pragma once

#include "core/errors/ErrorCode.h"
#include "core/logging/ILogger.h"
#include "modules/requirement/persistence/RequirementJsonStorage.h"
#include "modules/topology/dto/TopologyRecommendationDto.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"
#include "modules/topology/service/TopologyTemplateLoader.h"
#include "modules/topology/validator/TopologyValidator.h"

namespace RoboSDP::Topology::Service
{

/// Topology 候选生成结果。
struct TopologyGenerateResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString requirement_file_path;
    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto state;
    RoboSDP::Topology::Validation::TopologyValidationResult validation_result;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// Topology 保存结果。
struct TopologySaveResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString file_path;
    RoboSDP::Topology::Validation::TopologyValidationResult validation_result;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// Topology 加载结果。
struct TopologyLoadResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString file_path;
    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto state;
    RoboSDP::Topology::Validation::TopologyValidationResult validation_result;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/**
 * @brief Topology 服务层。
 *
 * 该服务负责串起 Requirement 约束读取、模板候选生成、最小规则推荐、
 * Topology 校验以及保存/加载流程，不把复杂业务逻辑下沉到 UI。
 */
class TopologyService
{
public:
    TopologyService(
        RoboSDP::Topology::Persistence::TopologyJsonStorage& storage,
        RoboSDP::Topology::Validation::TopologyValidator& validator,
        RoboSDP::Topology::Service::TopologyTemplateLoader& templateLoader,
        RoboSDP::Requirement::Persistence::RequirementJsonStorage& requirementStorage,
        RoboSDP::Logging::ILogger* logger = nullptr);

    RoboSDP::Topology::Dto::RobotTopologyModelDto CreateDefaultModel() const;
    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto CreateDefaultState() const;

    RoboSDP::Topology::Validation::TopologyValidationResult Validate(
        const RoboSDP::Topology::Dto::RobotTopologyModelDto& model) const;

    std::vector<RoboSDP::Topology::Service::TopologyTemplateSummary> ListTemplates() const;

    TopologyGenerateResult GenerateCandidatesFromRequirement(
        const QString& projectRootPath,
        const QString& selectedTemplateId = {}) const;

    TopologySaveResult SaveDraft(
        const QString& projectRootPath,
        const RoboSDP::Topology::Dto::TopologyWorkspaceStateDto& state) const;

    TopologyLoadResult LoadDraft(const QString& projectRootPath) const;

private:
    /// 从 Requirement 中抽取本轮 Topology 需要的最小约束集合。
    struct RequirementTopologyConstraints
    {
        QString requirement_name;
        QString scenario_type;
        QString preferred_base_mount_type;
        bool base_mount_specified = false;
        bool hollow_wrist_required = false;
        bool hollow_wrist_specified = false;
        double reserved_channel_diameter_mm = 0.0;
        bool reserved_channel_specified = false;
    };

    bool TryLoadRequirementConstraints(
        const QString& projectRootPath,
        RequirementTopologyConstraints& constraints,
        QString& requirementFilePath,
        QString& failureMessage) const;

    RoboSDP::Topology::Dto::TopologyCandidateDto BuildCandidate(
        const RoboSDP::Topology::Service::TopologyTemplateRecord& templateRecord,
        const RequirementTopologyConstraints& constraints) const;

    RoboSDP::Topology::Dto::TopologyRecommendationDto BuildRecommendation(
        const std::vector<RoboSDP::Topology::Dto::TopologyCandidateDto>& candidates,
        bool requirementLoaded,
        bool requirementConstraintsApplied) const;

private:
    RoboSDP::Topology::Persistence::TopologyJsonStorage& m_storage;
    RoboSDP::Topology::Validation::TopologyValidator& m_validator;
    RoboSDP::Topology::Service::TopologyTemplateLoader& m_template_loader;
    RoboSDP::Requirement::Persistence::RequirementJsonStorage& m_requirement_storage;
    RoboSDP::Logging::ILogger* m_logger = nullptr;
};

} // namespace RoboSDP::Topology::Service
