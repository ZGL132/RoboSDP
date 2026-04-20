#include "modules/scheme/service/SchemeSnapshotService.h"

#include <algorithm>
#include <QDateTime>
#include <QDir>
#include <QStringList>

namespace RoboSDP::Scheme::Service
{

namespace
{

QString BuildTimestamp()
{
    return QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
}

bool IsMissingDocumentError(RoboSDP::Errors::ErrorCode errorCode)
{
    return errorCode == RoboSDP::Errors::ErrorCode::RepositoryDocumentNotFound;
}

QString RequirementRefFallback()
{
    return QStringLiteral("requirements/requirement-model.json");
}

QString SelectionRefFallback()
{
    return QStringLiteral("selection/drivetrain-selection.json");
}

} // namespace

SchemeSnapshotService::SchemeSnapshotService(
    RoboSDP::Scheme::Persistence::SchemeJsonStorage& storage,
    RoboSDP::Requirement::Persistence::RequirementJsonStorage& requirementStorage,
    RoboSDP::Topology::Persistence::TopologyJsonStorage& topologyStorage,
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage& kinematicStorage,
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage& dynamicStorage,
    RoboSDP::Selection::Persistence::SelectionJsonStorage& selectionStorage,
    RoboSDP::Planning::Persistence::PlanningJsonStorage& planningStorage,
    RoboSDP::Logging::ILogger* logger)
    : m_storage(storage)
    , m_requirement_storage(requirementStorage)
    , m_topology_storage(topologyStorage)
    , m_kinematic_storage(kinematicStorage)
    , m_dynamic_storage(dynamicStorage)
    , m_selection_storage(selectionStorage)
    , m_planning_storage(planningStorage)
    , m_logger(logger)
{
}

RoboSDP::Scheme::Dto::SchemeSnapshotDto SchemeSnapshotService::CreateDefaultSnapshot() const
{
    RoboSDP::Scheme::Dto::SchemeSnapshotDto snapshot =
        RoboSDP::Scheme::Dto::SchemeSnapshotDto::CreateDefault();
    snapshot.aggregate.export_meta.default_export_format = QStringLiteral("json");
    snapshot.aggregate.export_meta.default_export_relative_path = QStringLiteral("exports/scheme-export.json");
    return snapshot;
}

SchemeBuildResult SchemeSnapshotService::BuildSnapshot(const QString& projectRootPath) const
{
    SchemeBuildResult result;
    result.snapshot = CreateDefaultSnapshot();
    result.snapshot.meta.source_project_root = QDir(projectRootPath).absolutePath();
    result.snapshot.meta.created_at = BuildTimestamp();
    result.snapshot.meta.updated_at = result.snapshot.meta.created_at;

    QStringList loadedModules;
    QStringList missingModules;

    const auto processLoad =
        [&loadedModules, &missingModules](const QString& moduleName, RoboSDP::Errors::ErrorCode errorCode) {
            if (errorCode == RoboSDP::Errors::ErrorCode::Ok)
            {
                loadedModules.push_back(moduleName);
                return RoboSDP::Errors::ErrorCode::Ok;
            }

            if (IsMissingDocumentError(errorCode))
            {
                missingModules.push_back(moduleName);
                return RoboSDP::Errors::ErrorCode::Ok;
            }

            return errorCode;
        };

    RoboSDP::Errors::ErrorCode error = processLoad(
        QStringLiteral("Requirement"),
        TryLoadRequirementSummary(projectRootPath, result.snapshot.aggregate));
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.error_code = error;
        result.message = QStringLiteral("读取 Requirement 草稿失败。");
        return result;
    }

    error = processLoad(
        QStringLiteral("Topology"),
        TryLoadTopologySummary(projectRootPath, result.snapshot.aggregate));
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.error_code = error;
        result.message = QStringLiteral("读取 Topology 草稿失败。");
        return result;
    }

    error = processLoad(
        QStringLiteral("Kinematics"),
        TryLoadKinematicsSummary(projectRootPath, result.snapshot.aggregate));
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.error_code = error;
        result.message = QStringLiteral("读取 Kinematics 草稿失败。");
        return result;
    }

    error = processLoad(
        QStringLiteral("Dynamics"),
        TryLoadDynamicsSummary(projectRootPath, result.snapshot.aggregate));
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.error_code = error;
        result.message = QStringLiteral("读取 Dynamics 草稿失败。");
        return result;
    }

    error = processLoad(
        QStringLiteral("Selection"),
        TryLoadSelectionSummary(projectRootPath, result.snapshot.aggregate));
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.error_code = error;
        result.message = QStringLiteral("读取 Selection 草稿失败。");
        return result;
    }

    error = processLoad(
        QStringLiteral("Planning"),
        TryLoadPlanningSummary(projectRootPath, result.snapshot.aggregate));
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.error_code = error;
        result.message = QStringLiteral("读取 Planning 草稿失败。");
        return result;
    }

    result.snapshot.aggregate.available_module_count = loadedModules.size();
    if (loadedModules.isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::RepositoryDocumentNotFound;
        result.message = QStringLiteral("未找到可聚合的上游模块草稿。");
        return result;
    }

    result.snapshot.meta.scheme_id =
        QStringLiteral("scheme_snapshot_%1").arg(QDateTime::currentSecsSinceEpoch());
    result.snapshot.meta.name =
        QStringLiteral("阶段 1 方案快照（%1）").arg(result.snapshot.aggregate.available_module_count);

    result.snapshot.aggregate.completeness_summary =
        missingModules.isEmpty()
            ? QStringLiteral("已聚合全部 6 个上游模块结果。")
            : QStringLiteral("已聚合 %1 个模块，缺失：%2。")
                  .arg(loadedModules.size())
                  .arg(missingModules.join(QStringLiteral("、")));

    result.snapshot_file_path = m_storage.BuildAbsoluteSnapshotFilePath(projectRootPath);
    result.message = result.snapshot.aggregate.completeness_summary;
    return result;
}

SchemeSaveResult SchemeSnapshotService::SaveSnapshot(
    const QString& projectRootPath,
    const RoboSDP::Scheme::Dto::SchemeSnapshotDto& snapshot) const
{
    SchemeSaveResult result;
    RoboSDP::Scheme::Dto::SchemeSnapshotDto snapshotToSave = snapshot;
    snapshotToSave.meta.updated_at = BuildTimestamp();

    result.error_code = m_storage.SaveSnapshot(projectRootPath, snapshotToSave);
    result.snapshot_file_path = m_storage.BuildAbsoluteSnapshotFilePath(projectRootPath);
    result.message = RoboSDP::Errors::ToChineseMessage(result.error_code);
    return result;
}

SchemeLoadResult SchemeSnapshotService::LoadSnapshot(const QString& projectRootPath) const
{
    SchemeLoadResult result;
    result.error_code = m_storage.LoadSnapshot(projectRootPath, result.snapshot);
    result.snapshot_file_path = m_storage.BuildAbsoluteSnapshotFilePath(projectRootPath);
    result.message = RoboSDP::Errors::ToChineseMessage(result.error_code);
    return result;
}

RoboSDP::Errors::ErrorCode SchemeSnapshotService::TryLoadRequirementSummary(
    const QString& projectRootPath,
    RoboSDP::Scheme::Dto::SchemeAggregateDto& aggregate) const
{
    RoboSDP::Requirement::Dto::RequirementModelDto requirementModel;
    const RoboSDP::Errors::ErrorCode error =
        m_requirement_storage.Load(projectRootPath, requirementModel);
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    aggregate.refs.requirement_ref = RequirementRefFallback();
    aggregate.requirement_summary.available = true;
    aggregate.requirement_summary.project_name = requirementModel.project_meta.project_name;
    aggregate.requirement_summary.scenario_type = requirementModel.project_meta.scenario_type;
    aggregate.requirement_summary.rated_payload_kg = requirementModel.load_requirements.rated_payload;
    aggregate.requirement_summary.max_payload_kg = requirementModel.load_requirements.max_payload;
    aggregate.requirement_summary.key_pose_count =
        static_cast<int>(requirementModel.workspace_requirements.key_poses.size());
    aggregate.requirement_summary.takt_time_s = requirementModel.motion_requirements.takt_time;
    aggregate.requirement_summary.summary_text =
        QStringLiteral("%1 / 额定负载 %2 kg / 节拍 %3 s")
            .arg(aggregate.requirement_summary.scenario_type)
            .arg(aggregate.requirement_summary.rated_payload_kg, 0, 'f', 2)
            .arg(aggregate.requirement_summary.takt_time_s, 0, 'f', 2);
    return RoboSDP::Errors::ErrorCode::Ok;
}

RoboSDP::Errors::ErrorCode SchemeSnapshotService::TryLoadTopologySummary(
    const QString& projectRootPath,
    RoboSDP::Scheme::Dto::SchemeAggregateDto& aggregate) const
{
    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto topologyState;
    const RoboSDP::Errors::ErrorCode error = m_topology_storage.Load(projectRootPath, topologyState);
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    aggregate.refs.topology_ref = topologyState.current_model.meta.topology_id;
    if (aggregate.refs.requirement_ref.isEmpty())
    {
        aggregate.refs.requirement_ref = topologyState.current_model.meta.requirement_ref;
    }

    aggregate.topology_summary.available = true;
    aggregate.topology_summary.topology_id = topologyState.current_model.meta.topology_id;
    aggregate.topology_summary.topology_name = topologyState.current_model.meta.name;
    aggregate.topology_summary.template_id = topologyState.current_model.meta.template_id;
    aggregate.topology_summary.robot_type = topologyState.current_model.robot_definition.robot_type;
    aggregate.topology_summary.joint_count = topologyState.current_model.robot_definition.joint_count;
    aggregate.topology_summary.candidate_count = static_cast<int>(topologyState.candidates.size());
    aggregate.topology_summary.recommended_candidate_id =
        topologyState.recommendation.recommended_candidate_id;
    aggregate.topology_summary.summary_text =
        QStringLiteral("%1 / %2 轴 / 推荐候选 %3")
            .arg(aggregate.topology_summary.robot_type)
            .arg(aggregate.topology_summary.joint_count)
            .arg(aggregate.topology_summary.recommended_candidate_id);
    return RoboSDP::Errors::ErrorCode::Ok;
}

RoboSDP::Errors::ErrorCode SchemeSnapshotService::TryLoadKinematicsSummary(
    const QString& projectRootPath,
    RoboSDP::Scheme::Dto::SchemeAggregateDto& aggregate) const
{
    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto kinematicState;
    const RoboSDP::Errors::ErrorCode error =
        m_kinematic_storage.LoadModel(projectRootPath, kinematicState);
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    aggregate.refs.kinematic_ref = kinematicState.current_model.meta.kinematic_id;
    if (aggregate.refs.topology_ref.isEmpty())
    {
        aggregate.refs.topology_ref = kinematicState.current_model.meta.topology_ref;
    }
    if (aggregate.refs.requirement_ref.isEmpty())
    {
        aggregate.refs.requirement_ref = kinematicState.current_model.meta.requirement_ref;
    }

    aggregate.kinematics_summary.available = true;
    aggregate.kinematics_summary.kinematic_id = kinematicState.current_model.meta.kinematic_id;
    aggregate.kinematics_summary.parameter_convention =
        kinematicState.current_model.parameter_convention;
    aggregate.kinematics_summary.joint_count = kinematicState.current_model.joint_count;
    aggregate.kinematics_summary.last_fk_success = kinematicState.last_fk_result.success;
    aggregate.kinematics_summary.last_ik_success = kinematicState.last_ik_result.success;
    aggregate.kinematics_summary.workspace_reachable_sample_count =
        kinematicState.last_workspace_result.reachable_sample_count;
    aggregate.kinematics_summary.workspace_max_radius_m =
        kinematicState.last_workspace_result.max_radius_m;
    aggregate.kinematics_summary.summary_text =
        QStringLiteral("%1 / 可达样本 %2 / 最大半径 %3 m")
            .arg(aggregate.kinematics_summary.parameter_convention)
            .arg(aggregate.kinematics_summary.workspace_reachable_sample_count)
            .arg(aggregate.kinematics_summary.workspace_max_radius_m, 0, 'f', 3);
    return RoboSDP::Errors::ErrorCode::Ok;
}

RoboSDP::Errors::ErrorCode SchemeSnapshotService::TryLoadDynamicsSummary(
    const QString& projectRootPath,
    RoboSDP::Scheme::Dto::SchemeAggregateDto& aggregate) const
{
    RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto dynamicsState;
    const RoboSDP::Errors::ErrorCode error = m_dynamic_storage.Load(projectRootPath, dynamicsState);
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    aggregate.refs.dynamic_ref = dynamicsState.current_model.meta.dynamic_id;
    if (aggregate.refs.kinematic_ref.isEmpty())
    {
        aggregate.refs.kinematic_ref = dynamicsState.current_model.meta.kinematic_ref;
    }
    if (aggregate.refs.topology_ref.isEmpty())
    {
        aggregate.refs.topology_ref = dynamicsState.current_model.meta.topology_ref;
    }
    if (aggregate.refs.requirement_ref.isEmpty())
    {
        aggregate.refs.requirement_ref = dynamicsState.current_model.meta.requirement_ref;
    }

    double maxPeakTorqueNm = 0.0;
    for (const auto& jointEnvelope : dynamicsState.load_envelope.joints)
    {
        maxPeakTorqueNm = std::max(maxPeakTorqueNm, jointEnvelope.peak_torque_nm);
    }

    aggregate.dynamics_summary.available = true;
    aggregate.dynamics_summary.dynamic_id = dynamicsState.current_model.meta.dynamic_id;
    aggregate.dynamics_summary.trajectory_count =
        static_cast<int>(dynamicsState.parameterized_trajectories.size());
    aggregate.dynamics_summary.peak_stat_count =
        static_cast<int>(dynamicsState.peak_stats.size());
    aggregate.dynamics_summary.rms_stat_count =
        static_cast<int>(dynamicsState.rms_stats.size());
    aggregate.dynamics_summary.envelope_joint_count =
        static_cast<int>(dynamicsState.load_envelope.joints.size());
    aggregate.dynamics_summary.max_peak_torque_nm = maxPeakTorqueNm;
    aggregate.dynamics_summary.summary_text =
        QStringLiteral("轨迹 %1 条 / 包络关节 %2 个 / 最大峰值 %3 Nm")
            .arg(aggregate.dynamics_summary.trajectory_count)
            .arg(aggregate.dynamics_summary.envelope_joint_count)
            .arg(aggregate.dynamics_summary.max_peak_torque_nm, 0, 'f', 2);
    return RoboSDP::Errors::ErrorCode::Ok;
}

RoboSDP::Errors::ErrorCode SchemeSnapshotService::TryLoadSelectionSummary(
    const QString& projectRootPath,
    RoboSDP::Scheme::Dto::SchemeAggregateDto& aggregate) const
{
    RoboSDP::Selection::Dto::SelectionWorkspaceStateDto selectionState;
    const RoboSDP::Errors::ErrorCode error = m_selection_storage.Load(projectRootPath, selectionState);
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    aggregate.refs.selection_ref = SelectionRefFallback();
    if (aggregate.refs.dynamic_ref.isEmpty())
    {
        aggregate.refs.dynamic_ref = selectionState.drive_train_result.dynamic_ref;
    }
    if (aggregate.refs.kinematic_ref.isEmpty())
    {
        aggregate.refs.kinematic_ref = selectionState.drive_train_result.kinematic_ref;
    }
    if (aggregate.refs.topology_ref.isEmpty())
    {
        aggregate.refs.topology_ref = selectionState.drive_train_result.topology_ref;
    }
    if (aggregate.refs.requirement_ref.isEmpty())
    {
        aggregate.refs.requirement_ref = selectionState.drive_train_result.requirement_ref;
    }

    int recommendedJointCount = 0;
    for (const auto& jointSelection : selectionState.drive_train_result.joint_selections)
    {
        if (jointSelection.has_recommendation)
        {
            ++recommendedJointCount;
        }
    }

    aggregate.selection_summary.available = true;
    aggregate.selection_summary.success = selectionState.drive_train_result.success;
    aggregate.selection_summary.joint_selection_count =
        static_cast<int>(selectionState.drive_train_result.joint_selections.size());
    aggregate.selection_summary.recommended_joint_count = recommendedJointCount;
    aggregate.selection_summary.summary_text =
        QStringLiteral("联合驱动链 %1 / 推荐覆盖 %2 个关节")
            .arg(aggregate.selection_summary.success ? QStringLiteral("成功") : QStringLiteral("失败"))
            .arg(aggregate.selection_summary.recommended_joint_count);
    return RoboSDP::Errors::ErrorCode::Ok;
}

RoboSDP::Errors::ErrorCode SchemeSnapshotService::TryLoadPlanningSummary(
    const QString& projectRootPath,
    RoboSDP::Scheme::Dto::SchemeAggregateDto& aggregate) const
{
    RoboSDP::Planning::Dto::PlanningWorkspaceStateDto planningState;
    const RoboSDP::Errors::ErrorCode error = m_planning_storage.Load(projectRootPath, planningState);
    if (error != RoboSDP::Errors::ErrorCode::Ok)
    {
        return error;
    }

    aggregate.refs.planning_scene_ref = planningState.current_scene.meta.planning_scene_id;
    if (!planningState.results.empty())
    {
        aggregate.refs.planning_result_ref = planningState.results.front().verification_id;
    }
    if (aggregate.refs.kinematic_ref.isEmpty())
    {
        aggregate.refs.kinematic_ref = planningState.current_scene.meta.kinematic_ref;
    }
    if (aggregate.refs.dynamic_ref.isEmpty())
    {
        aggregate.refs.dynamic_ref = planningState.current_scene.meta.dynamic_ref;
    }
    if (aggregate.refs.selection_ref == SelectionRefFallback() &&
        !planningState.current_scene.meta.selection_ref.isEmpty())
    {
        aggregate.refs.selection_ref = planningState.current_scene.meta.selection_ref;
    }
    if (aggregate.refs.requirement_ref.isEmpty())
    {
        aggregate.refs.requirement_ref = planningState.current_scene.meta.requirement_ref;
    }

    int collisionIssueCount = 0;
    int selfCollisionIssueCount = 0;
    bool overallSuccess = !planningState.results.empty();
    bool cycleTimeWithinTarget = false;
    for (const auto& result : planningState.results)
    {
        overallSuccess = overallSuccess && result.success;
        for (const auto& collision : result.collision_results)
        {
            if (collision.in_collision)
            {
                ++collisionIssueCount;
            }
        }
        for (const auto& selfCollision : result.self_collision_results)
        {
            if (selfCollision.in_self_collision)
            {
                ++selfCollisionIssueCount;
            }
        }
        cycleTimeWithinTarget = cycleTimeWithinTarget || result.cycle_time_result.within_target;
    }

    aggregate.planning_summary.available = true;
    aggregate.planning_summary.success = overallSuccess;
    aggregate.planning_summary.request_count = static_cast<int>(planningState.requests.size());
    aggregate.planning_summary.verification_count = static_cast<int>(planningState.results.size());
    aggregate.planning_summary.collision_issue_count = collisionIssueCount;
    aggregate.planning_summary.self_collision_issue_count = selfCollisionIssueCount;
    aggregate.planning_summary.cycle_time_within_target = cycleTimeWithinTarget;
    aggregate.planning_summary.summary_text =
        QStringLiteral("验证 %1 条 / 碰撞问题 %2 / 自碰撞问题 %3")
            .arg(aggregate.planning_summary.verification_count)
            .arg(aggregate.planning_summary.collision_issue_count)
            .arg(aggregate.planning_summary.self_collision_issue_count);
    return RoboSDP::Errors::ErrorCode::Ok;
}

} // namespace RoboSDP::Scheme::Service
