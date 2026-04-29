#pragma once

#include "core/errors/ErrorCode.h"
#include "core/logging/ILogger.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/planning/adapter/MoveItGrpcAdapter.h"
#include "modules/planning/persistence/PlanningJsonStorage.h"
#include "modules/selection/persistence/SelectionJsonStorage.h"

namespace RoboSDP::Planning::Service
{

/// @brief 构建 PlanningScene 的结果。
struct PlanningSceneBuildResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString kinematic_file_path;
    QString selection_file_path;
    /// @brief 标记本次场景构建时上游统一工程主链快照是否已经就绪。
    bool upstream_snapshot_ready = false;
    /// @brief 保存本次场景构建读取到的统一主链摘要，便于 UI 与日志使用。
    QString upstream_snapshot_summary;
    /// @brief 标记当前依赖的派生产物是否物理存在。
    bool upstream_derived_artifact_exists = false;
    /// @brief 标记当前依赖的派生产物是否与上游主链快照新鲜对齐。
    bool upstream_derived_artifact_fresh = false;
    RoboSDP::Planning::Dto::PlanningWorkspaceStateDto state;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// @brief 执行规划验证的结果。
struct PlanningRunResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString request_file_path;
    QString result_file_path;
    RoboSDP::Planning::Dto::PlanningWorkspaceStateDto state;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// @brief 保存 Planning 草稿的结果。
struct PlanningSaveResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString scene_file_path;
    QString request_file_path;
    QString result_file_path;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// @brief 加载 Planning 草稿的结果。
struct PlanningLoadResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString scene_file_path;
    QString request_file_path;
    QString result_file_path;
    RoboSDP::Planning::Dto::PlanningWorkspaceStateDto state;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/**
 * @brief PlanningVerification 服务层。
 *
 * 当前服务层只覆盖第 6 阶段最小闭环：
 * 1. 从上游 Kinematics / Selection 草稿构建 PlanningScene；
 * 2. 生成点到点规划请求；
 * 3. 调用 MoveIt gRPC Adapter 骨架；
 * 4. 汇总碰撞 / 自碰撞 / 节拍结果；
 * 5. 保存与加载 Planning JSON。
 */
class PlanningVerificationService
{
public:
    PlanningVerificationService(
        RoboSDP::Planning::Persistence::PlanningJsonStorage& storage,
        RoboSDP::Kinematics::Persistence::KinematicJsonStorage& kinematicStorage,
        RoboSDP::Selection::Persistence::SelectionJsonStorage& selectionStorage,
        RoboSDP::Logging::ILogger* logger = nullptr);

    RoboSDP::Planning::Dto::PlanningWorkspaceStateDto CreateDefaultState() const;

    PlanningSceneBuildResult BuildPlanningScene(const QString& projectRootPath) const;

    PlanningRunResult RunPointToPointVerification(
        const QString& projectRootPath,
        const RoboSDP::Planning::Dto::PlanningWorkspaceStateDto& inputState) const;

    PlanningSaveResult SaveDraft(
        const QString& projectRootPath,
        const RoboSDP::Planning::Dto::PlanningWorkspaceStateDto& state) const;

    PlanningLoadResult LoadDraft(const QString& projectRootPath) const;

private:
    RoboSDP::Planning::Dto::PlanningSceneDto BuildSceneFromUpstream(
        const RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& kinematicState,
        const RoboSDP::Selection::Dto::SelectionWorkspaceStateDto* selectionState) const;

    RoboSDP::Planning::Dto::PlanningRequestDto BuildRequestFromScene(
        const RoboSDP::Planning::Dto::PlanningSceneDto& scene) const;

    RoboSDP::Planning::Dto::PlanningVerificationResultDto BuildVerificationResult(
        const RoboSDP::Planning::Dto::PlanningSceneDto& scene,
        const RoboSDP::Planning::Dto::PlanningRequestDto& request,
        const RoboSDP::Planning::Dto::PlanningResponseDto& response) const;

private:
    RoboSDP::Planning::Persistence::PlanningJsonStorage& m_storage;
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage& m_kinematic_storage;
    RoboSDP::Selection::Persistence::SelectionJsonStorage& m_selection_storage;
    RoboSDP::Logging::ILogger* m_logger = nullptr;
    RoboSDP::Planning::Adapter::MoveItGrpcAdapter m_adapter;
};

} // namespace RoboSDP::Planning::Service
