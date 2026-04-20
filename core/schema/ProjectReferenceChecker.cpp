#include "core/schema/ProjectReferenceChecker.h"

#include "core/errors/ErrorCode.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/planning/persistence/PlanningJsonStorage.h"
#include "modules/requirement/persistence/RequirementJsonStorage.h"
#include "modules/scheme/persistence/SchemeJsonStorage.h"
#include "modules/selection/persistence/SelectionJsonStorage.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

namespace RoboSDP::Schema
{

namespace
{

void AddMissingModuleWarning(
    ValidationResult& result,
    const QString& field,
    const QString& moduleName,
    RoboSDP::Errors::ErrorCode errorCode)
{
    if (errorCode == RoboSDP::Errors::ErrorCode::RepositoryDocumentNotFound)
    {
        result.Add(
            field,
            QStringLiteral("REFERENCE_SOURCE_MISSING"),
            QStringLiteral("缺少 %1 结果文件，暂无法校验该模块引用。").arg(moduleName),
            ValidationSeverity::Warning);
        return;
    }

    result.Add(
        field,
        QStringLiteral("REFERENCE_SOURCE_LOAD_FAILED"),
        QStringLiteral("读取 %1 结果失败：%2")
            .arg(moduleName, RoboSDP::Errors::ToChineseMessage(errorCode)));
}

void CheckReferenceAgainstCandidates(
    ValidationResult& result,
    const QString& field,
    const QString& actualRef,
    const QStringList& acceptedRefs,
    const QString& message)
{
    if (actualRef.trimmed().isEmpty())
    {
        result.Add(
            field,
            QStringLiteral("REFERENCE_EMPTY"),
            QStringLiteral("%1：引用不能为空。").arg(message),
            ValidationSeverity::Warning);
        return;
    }

    if (!acceptedRefs.contains(actualRef))
    {
        result.Add(
            field,
            QStringLiteral("REFERENCE_MISMATCH"),
            QStringLiteral("%1：当前值“%2”未指向已知对象。").arg(message, actualRef));
    }
}

} // namespace

ProjectReferenceChecker::ProjectReferenceChecker(RoboSDP::Repository::IJsonRepository& repository)
    : m_repository(repository)
{
}

ValidationResult ProjectReferenceChecker::CheckProject(const QString& projectRootPath) const
{
    ValidationResult result;
    if (projectRootPath.trimmed().isEmpty())
    {
        result.Add(
            QStringLiteral("project.root"),
            QStringLiteral("REFERENCE_PROJECT_ROOT_REQUIRED"),
            QStringLiteral("项目目录不能为空，无法检查引用关系。"));
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
    const auto requirementError = requirementStorage.Load(projectRootPath, requirementModel);
    if (requirementError != RoboSDP::Errors::ErrorCode::Ok)
    {
        AddMissingModuleWarning(result, QStringLiteral("requirement"), QStringLiteral("Requirement"), requirementError);
    }

    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto topologyState;
    const auto topologyError = topologyStorage.Load(projectRootPath, topologyState);
    if (topologyError != RoboSDP::Errors::ErrorCode::Ok)
    {
        AddMissingModuleWarning(result, QStringLiteral("topology"), QStringLiteral("Topology"), topologyError);
    }

    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto kinematicsState;
    const auto kinematicsError = kinematicStorage.LoadModel(projectRootPath, kinematicsState);
    if (kinematicsError != RoboSDP::Errors::ErrorCode::Ok)
    {
        AddMissingModuleWarning(result, QStringLiteral("kinematics"), QStringLiteral("Kinematics"), kinematicsError);
    }

    RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto dynamicsState;
    const auto dynamicsError = dynamicStorage.Load(projectRootPath, dynamicsState);
    if (dynamicsError != RoboSDP::Errors::ErrorCode::Ok)
    {
        AddMissingModuleWarning(result, QStringLiteral("dynamics"), QStringLiteral("Dynamics"), dynamicsError);
    }

    RoboSDP::Selection::Dto::SelectionWorkspaceStateDto selectionState;
    const auto selectionError = selectionStorage.Load(projectRootPath, selectionState);
    if (selectionError != RoboSDP::Errors::ErrorCode::Ok)
    {
        AddMissingModuleWarning(result, QStringLiteral("selection"), QStringLiteral("Selection"), selectionError);
    }

    RoboSDP::Planning::Dto::PlanningWorkspaceStateDto planningState;
    const auto planningError = planningStorage.Load(projectRootPath, planningState);
    if (planningError != RoboSDP::Errors::ErrorCode::Ok)
    {
        AddMissingModuleWarning(result, QStringLiteral("planning"), QStringLiteral("Planning"), planningError);
    }

    RoboSDP::Scheme::Dto::SchemeSnapshotDto schemeSnapshot;
    const auto schemeError = schemeStorage.LoadSnapshot(projectRootPath, schemeSnapshot);
    if (schemeError != RoboSDP::Errors::ErrorCode::Ok)
    {
        AddMissingModuleWarning(result, QStringLiteral("scheme"), QStringLiteral("Scheme"), schemeError);
    }

    if (requirementError == RoboSDP::Errors::ErrorCode::Ok &&
        topologyError == RoboSDP::Errors::ErrorCode::Ok)
    {
        CheckReferenceAgainstCandidates(
            result,
            QStringLiteral("topology.meta.requirement_ref"),
            topologyState.current_model.meta.requirement_ref,
            {
                requirementStorage.RelativeFilePath(),
                requirementModel.project_meta.project_name},
            QStringLiteral("Topology -> Requirement"));
    }

    if (topologyError == RoboSDP::Errors::ErrorCode::Ok &&
        kinematicsError == RoboSDP::Errors::ErrorCode::Ok)
    {
        CheckReferenceAgainstCandidates(
            result,
            QStringLiteral("kinematics.meta.topology_ref"),
            kinematicsState.current_model.meta.topology_ref,
            {
                topologyState.current_model.meta.topology_id,
                topologyStorage.RelativeFilePath()},
            QStringLiteral("Kinematics -> Topology"));
    }

    if (requirementError == RoboSDP::Errors::ErrorCode::Ok &&
        kinematicsError == RoboSDP::Errors::ErrorCode::Ok)
    {
        CheckReferenceAgainstCandidates(
            result,
            QStringLiteral("kinematics.meta.requirement_ref"),
            kinematicsState.current_model.meta.requirement_ref,
            {
                requirementStorage.RelativeFilePath(),
                requirementModel.project_meta.project_name},
            QStringLiteral("Kinematics -> Requirement"));
    }

    if (kinematicsError == RoboSDP::Errors::ErrorCode::Ok &&
        dynamicsError == RoboSDP::Errors::ErrorCode::Ok)
    {
        CheckReferenceAgainstCandidates(
            result,
            QStringLiteral("dynamics.meta.kinematic_ref"),
            dynamicsState.current_model.meta.kinematic_ref,
            {
                kinematicsState.current_model.meta.kinematic_id,
                kinematicStorage.RelativeModelFilePath()},
            QStringLiteral("Dynamics -> Kinematics"));
    }

    if (topologyError == RoboSDP::Errors::ErrorCode::Ok &&
        dynamicsError == RoboSDP::Errors::ErrorCode::Ok)
    {
        CheckReferenceAgainstCandidates(
            result,
            QStringLiteral("dynamics.meta.topology_ref"),
            dynamicsState.current_model.meta.topology_ref,
            {
                topologyState.current_model.meta.topology_id,
                topologyStorage.RelativeFilePath()},
            QStringLiteral("Dynamics -> Topology"));
    }

    if (requirementError == RoboSDP::Errors::ErrorCode::Ok &&
        dynamicsError == RoboSDP::Errors::ErrorCode::Ok)
    {
        CheckReferenceAgainstCandidates(
            result,
            QStringLiteral("dynamics.meta.requirement_ref"),
            dynamicsState.current_model.meta.requirement_ref,
            {
                requirementStorage.RelativeFilePath(),
                requirementModel.project_meta.project_name},
            QStringLiteral("Dynamics -> Requirement"));
    }

    if (dynamicsError == RoboSDP::Errors::ErrorCode::Ok &&
        selectionError == RoboSDP::Errors::ErrorCode::Ok)
    {
        CheckReferenceAgainstCandidates(
            result,
            QStringLiteral("selection.drive_train_result.dynamic_ref"),
            selectionState.drive_train_result.dynamic_ref,
            {
                dynamicsState.current_model.meta.dynamic_id,
                dynamicStorage.RelativeModelFilePath()},
            QStringLiteral("Selection -> Dynamics"));
    }

    if (kinematicsError == RoboSDP::Errors::ErrorCode::Ok &&
        selectionError == RoboSDP::Errors::ErrorCode::Ok)
    {
        CheckReferenceAgainstCandidates(
            result,
            QStringLiteral("selection.drive_train_result.kinematic_ref"),
            selectionState.drive_train_result.kinematic_ref,
            {
                kinematicsState.current_model.meta.kinematic_id,
                kinematicStorage.RelativeModelFilePath()},
            QStringLiteral("Selection -> Kinematics"));
    }

    if (topologyError == RoboSDP::Errors::ErrorCode::Ok &&
        selectionError == RoboSDP::Errors::ErrorCode::Ok)
    {
        CheckReferenceAgainstCandidates(
            result,
            QStringLiteral("selection.drive_train_result.topology_ref"),
            selectionState.drive_train_result.topology_ref,
            {
                topologyState.current_model.meta.topology_id,
                topologyStorage.RelativeFilePath()},
            QStringLiteral("Selection -> Topology"));
    }

    if (requirementError == RoboSDP::Errors::ErrorCode::Ok &&
        selectionError == RoboSDP::Errors::ErrorCode::Ok)
    {
        CheckReferenceAgainstCandidates(
            result,
            QStringLiteral("selection.drive_train_result.requirement_ref"),
            selectionState.drive_train_result.requirement_ref,
            {
                requirementStorage.RelativeFilePath(),
                requirementModel.project_meta.project_name},
            QStringLiteral("Selection -> Requirement"));
    }

    if (kinematicsError == RoboSDP::Errors::ErrorCode::Ok &&
        planningError == RoboSDP::Errors::ErrorCode::Ok)
    {
        CheckReferenceAgainstCandidates(
            result,
            QStringLiteral("planning.current_scene.meta.kinematic_ref"),
            planningState.current_scene.meta.kinematic_ref,
            {
                kinematicsState.current_model.meta.kinematic_id,
                kinematicStorage.RelativeModelFilePath()},
            QStringLiteral("Planning -> Kinematics"));
    }

    if (dynamicsError == RoboSDP::Errors::ErrorCode::Ok &&
        planningError == RoboSDP::Errors::ErrorCode::Ok)
    {
        CheckReferenceAgainstCandidates(
            result,
            QStringLiteral("planning.current_scene.meta.dynamic_ref"),
            planningState.current_scene.meta.dynamic_ref,
            {
                dynamicsState.current_model.meta.dynamic_id,
                dynamicStorage.RelativeModelFilePath()},
            QStringLiteral("Planning -> Dynamics"));
    }

    if (selectionError == RoboSDP::Errors::ErrorCode::Ok &&
        planningError == RoboSDP::Errors::ErrorCode::Ok)
    {
        CheckReferenceAgainstCandidates(
            result,
            QStringLiteral("planning.current_scene.meta.selection_ref"),
            planningState.current_scene.meta.selection_ref,
            {
                selectionStorage.RelativeDriveTrainFilePath()},
            QStringLiteral("Planning -> Selection"));
    }

    if (requirementError == RoboSDP::Errors::ErrorCode::Ok &&
        planningError == RoboSDP::Errors::ErrorCode::Ok)
    {
        CheckReferenceAgainstCandidates(
            result,
            QStringLiteral("planning.current_scene.meta.requirement_ref"),
            planningState.current_scene.meta.requirement_ref,
            {
                requirementStorage.RelativeFilePath(),
                requirementModel.project_meta.project_name},
            QStringLiteral("Planning -> Requirement"));
    }

    if (schemeError == RoboSDP::Errors::ErrorCode::Ok)
    {
        const auto& refs = schemeSnapshot.aggregate.refs;

        if (requirementError == RoboSDP::Errors::ErrorCode::Ok)
        {
            CheckReferenceAgainstCandidates(
                result,
                QStringLiteral("scheme.aggregate.refs.requirement_ref"),
                refs.requirement_ref,
                {
                    requirementStorage.RelativeFilePath(),
                    requirementModel.project_meta.project_name},
                QStringLiteral("Scheme -> Requirement"));
        }

        if (topologyError == RoboSDP::Errors::ErrorCode::Ok)
        {
            CheckReferenceAgainstCandidates(
                result,
                QStringLiteral("scheme.aggregate.refs.topology_ref"),
                refs.topology_ref,
                {
                    topologyState.current_model.meta.topology_id,
                    topologyStorage.RelativeFilePath()},
                QStringLiteral("Scheme -> Topology"));
        }

        if (kinematicsError == RoboSDP::Errors::ErrorCode::Ok)
        {
            CheckReferenceAgainstCandidates(
                result,
                QStringLiteral("scheme.aggregate.refs.kinematic_ref"),
                refs.kinematic_ref,
                {
                    kinematicsState.current_model.meta.kinematic_id,
                    kinematicStorage.RelativeModelFilePath()},
                QStringLiteral("Scheme -> Kinematics"));
        }

        if (dynamicsError == RoboSDP::Errors::ErrorCode::Ok)
        {
            CheckReferenceAgainstCandidates(
                result,
                QStringLiteral("scheme.aggregate.refs.dynamic_ref"),
                refs.dynamic_ref,
                {
                    dynamicsState.current_model.meta.dynamic_id,
                    dynamicStorage.RelativeModelFilePath()},
                QStringLiteral("Scheme -> Dynamics"));
        }

        if (selectionError == RoboSDP::Errors::ErrorCode::Ok)
        {
            CheckReferenceAgainstCandidates(
                result,
                QStringLiteral("scheme.aggregate.refs.selection_ref"),
                refs.selection_ref,
                {
                    selectionStorage.RelativeDriveTrainFilePath()},
                QStringLiteral("Scheme -> Selection"));
        }

        if (planningError == RoboSDP::Errors::ErrorCode::Ok)
        {
            CheckReferenceAgainstCandidates(
                result,
                QStringLiteral("scheme.aggregate.refs.planning_scene_ref"),
                refs.planning_scene_ref,
                {
                    planningState.current_scene.meta.planning_scene_id,
                    planningStorage.RelativeSceneFilePath()},
                QStringLiteral("Scheme -> PlanningScene"));

            QStringList planningResultCandidates {planningStorage.RelativeResultFilePath()};
            if (!planningState.results.empty())
            {
                planningResultCandidates.push_back(planningState.results.front().verification_id);
            }
            CheckReferenceAgainstCandidates(
                result,
                QStringLiteral("scheme.aggregate.refs.planning_result_ref"),
                refs.planning_result_ref,
                planningResultCandidates,
                QStringLiteral("Scheme -> PlanningResult"));
        }
    }

    return result;
}

} // namespace RoboSDP::Schema
