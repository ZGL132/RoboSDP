#include "core/schema/ModuleValidationRegistry.h"

#include "core/errors/ErrorCode.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/planning/persistence/PlanningJsonStorage.h"
#include "modules/requirement/persistence/RequirementJsonStorage.h"
#include "modules/requirement/validator/RequirementValidator.h"
#include "modules/scheme/persistence/SchemeJsonStorage.h"
#include "modules/selection/persistence/SelectionJsonStorage.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"
#include "modules/topology/validator/TopologyValidator.h"

namespace RoboSDP::Schema
{

namespace
{

template <typename ResultType>
void AddStorageLoadIssue(
    ValidationResult& result,
    const QString& fieldPrefix,
    const QString& filePath,
    RoboSDP::Errors::ErrorCode errorCode)
{
    if (errorCode == RoboSDP::Errors::ErrorCode::RepositoryDocumentNotFound)
    {
        // 第一阶段允许项目只跑到部分阶段，因此缺文件先给 warning，不阻塞后续阶段开发。
        result.Add(
            fieldPrefix,
            QStringLiteral("MODULE_JSON_MISSING"),
            QStringLiteral("未找到模块 JSON：%1").arg(filePath),
            ValidationSeverity::Warning);
        return;
    }

    result.Add(
        fieldPrefix,
        QStringLiteral("MODULE_JSON_LOAD_FAILED"),
        QStringLiteral("读取模块 JSON 失败：%1").arg(RoboSDP::Errors::ToChineseMessage(errorCode)),
        ValidationSeverity::Error);
}

void MapRequirementIssues(
    const RoboSDP::Requirement::Validation::RequirementValidationResult& source,
    ValidationResult& target)
{
    for (const auto& issue : source.issues)
    {
        ValidationSeverity severity = ValidationSeverity::Error;
        if (issue.severity == RoboSDP::Requirement::Validation::ValidationSeverity::Info)
        {
            severity = ValidationSeverity::Info;
        }
        else if (issue.severity == RoboSDP::Requirement::Validation::ValidationSeverity::Warning)
        {
            severity = ValidationSeverity::Warning;
        }

        target.Add(
            QStringLiteral("requirement.%1").arg(issue.field),
            issue.code,
            issue.message_zh,
            severity);
    }
}

void MapTopologyIssues(
    const RoboSDP::Topology::Validation::TopologyValidationResult& source,
    ValidationResult& target)
{
    for (const auto& issue : source.issues)
    {
        ValidationSeverity severity = ValidationSeverity::Error;
        if (issue.severity == RoboSDP::Topology::Validation::ValidationSeverity::Info)
        {
            severity = ValidationSeverity::Info;
        }
        else if (issue.severity == RoboSDP::Topology::Validation::ValidationSeverity::Warning)
        {
            severity = ValidationSeverity::Warning;
        }

        target.Add(
            QStringLiteral("topology.%1").arg(issue.field),
            issue.code,
            issue.message_zh,
            severity);
    }
}

ValidationResult ValidateKinematicsState(const RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& state)
{
    ValidationResult result;

    // 规则 1：运动学模型必须有稳定 ID，便于后续 Dynamics / Planning 引用。
    if (state.current_model.meta.kinematic_id.trimmed().isEmpty())
    {
        result.Add(
            QStringLiteral("kinematics.meta.kinematic_id"),
            QStringLiteral("KINEMATIC_ID_REQUIRED"),
            QStringLiteral("KinematicModel 的 kinematic_id 不能为空。"));
    }

    // 规则 2：当前第一阶段只接受正关节数，避免空模型继续流入后续主链。
    if (state.current_model.joint_count <= 0)
    {
        result.Add(
            QStringLiteral("kinematics.joint_count"),
            QStringLiteral("KINEMATIC_JOINT_COUNT_INVALID"),
            QStringLiteral("joint_count 必须大于 0。"));
    }

    // 规则 3：连杆参数数量应与 joint_count 对齐，避免 FK/IK 与持久化结构脱节。
    if (static_cast<int>(state.current_model.links.size()) != state.current_model.joint_count)
    {
        result.Add(
            QStringLiteral("kinematics.links"),
            QStringLiteral("KINEMATIC_LINK_COUNT_MISMATCH"),
            QStringLiteral("links 数量必须与 joint_count 一致。"));
    }

    // 规则 4：关节限位数量应与 joint_count 对齐，确保 IK 和工作空间采样有完整边界。
    if (static_cast<int>(state.current_model.joint_limits.size()) != state.current_model.joint_count)
    {
        result.Add(
            QStringLiteral("kinematics.joint_limits"),
            QStringLiteral("KINEMATIC_LIMIT_COUNT_MISMATCH"),
            QStringLiteral("joint_limits 数量必须与 joint_count 一致。"));
    }

    // 规则 5：当前最小路径只支持 DH / MDH 两种参数约定。
    const QString convention = state.current_model.parameter_convention.trimmed();
    if (convention != QStringLiteral("DH") && convention != QStringLiteral("MDH"))
    {
        result.Add(
            QStringLiteral("kinematics.parameter_convention"),
            QStringLiteral("KINEMATIC_PARAMETER_CONVENTION_INVALID"),
            QStringLiteral("parameter_convention 只允许为 DH 或 MDH。"));
    }

    return result;
}

ValidationResult ValidateDynamicsState(const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& state)
{
    ValidationResult result;

    // 规则 1：动力学模型必须保留稳定 ID，便于 Selection 和 Scheme 引用。
    if (state.current_model.meta.dynamic_id.trimmed().isEmpty())
    {
        result.Add(
            QStringLiteral("dynamics.meta.dynamic_id"),
            QStringLiteral("DYNAMIC_ID_REQUIRED"),
            QStringLiteral("DynamicModel 的 dynamic_id 不能为空。"));
    }

    // 规则 2：最小逆动力学主链至少要有连杆、关节和基准轨迹。
    if (state.current_model.links.empty())
    {
        result.Add(
            QStringLiteral("dynamics.links"),
            QStringLiteral("DYNAMIC_LINKS_EMPTY"),
            QStringLiteral("Dynamics 模型至少需要 1 条连杆。"));
    }

    if (state.current_model.joints.empty())
    {
        result.Add(
            QStringLiteral("dynamics.joints"),
            QStringLiteral("DYNAMIC_JOINTS_EMPTY"),
            QStringLiteral("Dynamics 模型至少需要 1 个关节传动参数。"));
    }

    if (state.current_model.trajectories.empty())
    {
        result.Add(
            QStringLiteral("dynamics.trajectories"),
            QStringLiteral("DYNAMIC_TRAJECTORIES_EMPTY"),
            QStringLiteral("Dynamics 模型至少需要 1 条基准轨迹。"));
    }

    // 规则 3：如果已经执行过逆动力学，结果中必须显式标记求解后端。
    for (std::size_t index = 0; index < state.trajectory_results.size(); ++index)
    {
        const auto& item = state.trajectory_results.at(index);
        if (item.solver_backend.trimmed().isEmpty())
        {
            result.Add(
                QStringLiteral("dynamics.trajectory_results[%1].solver_backend").arg(index),
                QStringLiteral("DYNAMIC_SOLVER_BACKEND_EMPTY"),
                QStringLiteral("逆动力学结果缺少 solver_backend 标记。"));
        }
    }

    return result;
}

ValidationResult ValidateSelectionState(const RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state)
{
    ValidationResult result;

    // 规则 1：若驱动链已形成结果，则必须保留上游引用，避免方案聚合时丢失来源。
    const auto& driveTrain = state.drive_train_result;
    if (driveTrain.success || !driveTrain.joint_selections.empty())
    {
        if (driveTrain.dynamic_ref.trimmed().isEmpty())
        {
            result.Add(
                QStringLiteral("selection.drive_train_result.dynamic_ref"),
                QStringLiteral("SELECTION_DYNAMIC_REF_REQUIRED"),
                QStringLiteral("驱动链结果缺少 dynamic_ref。"));
        }

        if (driveTrain.kinematic_ref.trimmed().isEmpty())
        {
            result.Add(
                QStringLiteral("selection.drive_train_result.kinematic_ref"),
                QStringLiteral("SELECTION_KINEMATIC_REF_REQUIRED"),
                QStringLiteral("驱动链结果缺少 kinematic_ref。"));
        }
    }
    else
    {
        result.Add(
            QStringLiteral("selection.drive_train_result"),
            QStringLiteral("SELECTION_RESULT_EMPTY"),
            QStringLiteral("当前尚未形成驱动链选型结果。"),
            ValidationSeverity::Warning);
    }

    return result;
}

ValidationResult ValidatePlanningState(const RoboSDP::Planning::Dto::PlanningWorkspaceStateDto& state)
{
    ValidationResult result;

    // 规则 1：PlanningScene 必须具备稳定场景 ID，便于保存/加载与 Scheme 聚合。
    if (state.current_scene.meta.planning_scene_id.trimmed().isEmpty())
    {
        result.Add(
            QStringLiteral("planning.current_scene.meta.planning_scene_id"),
            QStringLiteral("PLANNING_SCENE_ID_REQUIRED"),
            QStringLiteral("PlanningScene 的 planning_scene_id 不能为空。"));
    }

    // 规则 2：最小点到点规划路径必须有 motion_group 和 planner_id。
    if (state.current_scene.planning_config.motion_group.trimmed().isEmpty())
    {
        result.Add(
            QStringLiteral("planning.current_scene.planning_config.motion_group"),
            QStringLiteral("PLANNING_MOTION_GROUP_REQUIRED"),
            QStringLiteral("motion_group 不能为空。"));
    }

    if (state.current_scene.planning_config.planner_id.trimmed().isEmpty())
    {
        result.Add(
            QStringLiteral("planning.current_scene.planning_config.planner_id"),
            QStringLiteral("PLANNING_PLANNER_ID_REQUIRED"),
            QStringLiteral("planner_id 不能为空。"));
    }

    // 规则 3：若已有验证结果，则每条结果都应有 verification_id。
    for (std::size_t index = 0; index < state.results.size(); ++index)
    {
        if (state.results[index].verification_id.trimmed().isEmpty())
        {
            result.Add(
                QStringLiteral("planning.results[%1].verification_id").arg(index),
                QStringLiteral("PLANNING_VERIFICATION_ID_REQUIRED"),
                QStringLiteral("规划验证结果缺少 verification_id。"));
        }
    }

    return result;
}

ValidationResult ValidateSchemeSnapshot(const RoboSDP::Scheme::Dto::SchemeSnapshotDto& snapshot)
{
    ValidationResult result;

    // 规则 1：Scheme 快照必须具备稳定 ID，便于导出与回读。
    if (snapshot.meta.scheme_id.trimmed().isEmpty())
    {
        result.Add(
            QStringLiteral("scheme.meta.scheme_id"),
            QStringLiteral("SCHEME_ID_REQUIRED"),
            QStringLiteral("SchemeSnapshot 的 scheme_id 不能为空。"));
    }

    // 规则 2：最小聚合对象至少要保留默认导出格式和模块计数。
    if (snapshot.aggregate.export_meta.default_export_format.trimmed().isEmpty())
    {
        result.Add(
            QStringLiteral("scheme.aggregate.export_meta.default_export_format"),
            QStringLiteral("SCHEME_EXPORT_FORMAT_REQUIRED"),
            QStringLiteral("Scheme 聚合对象缺少 default_export_format。"));
    }

    if (snapshot.aggregate.available_module_count < 0)
    {
        result.Add(
            QStringLiteral("scheme.aggregate.available_module_count"),
            QStringLiteral("SCHEME_AVAILABLE_MODULE_COUNT_INVALID"),
            QStringLiteral("available_module_count 不能为负数。"));
    }

    return result;
}

} // namespace

ModuleValidationRegistry::ModuleValidationRegistry(RoboSDP::Repository::IJsonRepository& repository)
    : m_repository(repository)
{
}

ValidationResult ModuleValidationRegistry::ValidateProject(const QString& projectRootPath) const
{
    ValidationResult result;
    if (projectRootPath.trimmed().isEmpty())
    {
        result.Add(
            QStringLiteral("project_root"),
            QStringLiteral("VALIDATION_PROJECT_ROOT_REQUIRED"),
            QStringLiteral("项目目录不能为空，无法执行统一 validator。"));
        return result;
    }

    RoboSDP::Requirement::Persistence::RequirementJsonStorage requirementStorage(m_repository);
    RoboSDP::Topology::Persistence::TopologyJsonStorage topologyStorage(m_repository);
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage kinematicStorage(m_repository);
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage dynamicStorage(m_repository);
    RoboSDP::Selection::Persistence::SelectionJsonStorage selectionStorage(m_repository);
    RoboSDP::Planning::Persistence::PlanningJsonStorage planningStorage(m_repository);
    RoboSDP::Scheme::Persistence::SchemeJsonStorage schemeStorage(m_repository);

    RoboSDP::Requirement::Dto::RequirementModelDto requirementModel;
    auto errorCode = requirementStorage.Load(projectRootPath, requirementModel);
    if (errorCode == RoboSDP::Errors::ErrorCode::Ok)
    {
        RoboSDP::Requirement::Validation::RequirementValidator validator;
        MapRequirementIssues(validator.Validate(requirementModel), result);
    }
    else
    {
        AddStorageLoadIssue<RoboSDP::Requirement::Dto::RequirementModelDto>(
            result,
            QStringLiteral("requirement"),
            requirementStorage.BuildAbsoluteFilePath(projectRootPath),
            errorCode);
    }

    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto topologyState;
    errorCode = topologyStorage.Load(projectRootPath, topologyState);
    if (errorCode == RoboSDP::Errors::ErrorCode::Ok)
    {
        RoboSDP::Topology::Validation::TopologyValidator validator;
        MapTopologyIssues(validator.Validate(topologyState.current_model), result);
    }
    else
    {
        AddStorageLoadIssue<RoboSDP::Topology::Dto::TopologyWorkspaceStateDto>(
            result,
            QStringLiteral("topology"),
            topologyStorage.BuildAbsoluteFilePath(projectRootPath),
            errorCode);
    }

    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto kinematicsState;
    errorCode = kinematicStorage.LoadModel(projectRootPath, kinematicsState);
    if (errorCode == RoboSDP::Errors::ErrorCode::Ok)
    {
        result.Append(ValidateKinematicsState(kinematicsState));
    }
    else
    {
        AddStorageLoadIssue<RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto>(
            result,
            QStringLiteral("kinematics"),
            kinematicStorage.BuildAbsoluteModelFilePath(projectRootPath),
            errorCode);
    }

    RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto dynamicsState;
    errorCode = dynamicStorage.Load(projectRootPath, dynamicsState);
    if (errorCode == RoboSDP::Errors::ErrorCode::Ok)
    {
        result.Append(ValidateDynamicsState(dynamicsState));
    }
    else
    {
        AddStorageLoadIssue<RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto>(
            result,
            QStringLiteral("dynamics"),
            dynamicStorage.BuildAbsoluteModelFilePath(projectRootPath),
            errorCode);
    }

    RoboSDP::Selection::Dto::SelectionWorkspaceStateDto selectionState;
    errorCode = selectionStorage.Load(projectRootPath, selectionState);
    if (errorCode == RoboSDP::Errors::ErrorCode::Ok)
    {
        result.Append(ValidateSelectionState(selectionState));
    }
    else
    {
        AddStorageLoadIssue<RoboSDP::Selection::Dto::SelectionWorkspaceStateDto>(
            result,
            QStringLiteral("selection"),
            selectionStorage.BuildAbsoluteDriveTrainFilePath(projectRootPath),
            errorCode);
    }

    RoboSDP::Planning::Dto::PlanningWorkspaceStateDto planningState;
    errorCode = planningStorage.Load(projectRootPath, planningState);
    if (errorCode == RoboSDP::Errors::ErrorCode::Ok)
    {
        result.Append(ValidatePlanningState(planningState));
    }
    else
    {
        AddStorageLoadIssue<RoboSDP::Planning::Dto::PlanningWorkspaceStateDto>(
            result,
            QStringLiteral("planning"),
            planningStorage.BuildAbsoluteSceneFilePath(projectRootPath),
            errorCode);
    }

    RoboSDP::Scheme::Dto::SchemeSnapshotDto schemeSnapshot;
    errorCode = schemeStorage.LoadSnapshot(projectRootPath, schemeSnapshot);
    if (errorCode == RoboSDP::Errors::ErrorCode::Ok)
    {
        result.Append(ValidateSchemeSnapshot(schemeSnapshot));
    }
    else
    {
        AddStorageLoadIssue<RoboSDP::Scheme::Dto::SchemeSnapshotDto>(
            result,
            QStringLiteral("scheme"),
            schemeStorage.BuildAbsoluteSnapshotFilePath(projectRootPath),
            errorCode);
    }

    return result;
}

} // namespace RoboSDP::Schema
