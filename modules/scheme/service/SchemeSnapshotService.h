#pragma once

#include "core/errors/ErrorCode.h"
#include "core/logging/ILogger.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/planning/persistence/PlanningJsonStorage.h"
#include "modules/requirement/persistence/RequirementJsonStorage.h"
#include "modules/scheme/dto/SchemeSnapshotDto.h"
#include "modules/scheme/persistence/SchemeJsonStorage.h"
#include "modules/selection/persistence/SelectionJsonStorage.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

namespace RoboSDP::Scheme::Service
{

/// 方案快照聚合结果。
struct SchemeBuildResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    RoboSDP::Scheme::Dto::SchemeSnapshotDto snapshot;
    QString snapshot_file_path;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// 方案快照保存结果。
struct SchemeSaveResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString snapshot_file_path;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// 方案快照加载结果。
struct SchemeLoadResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    RoboSDP::Scheme::Dto::SchemeSnapshotDto snapshot;
    QString snapshot_file_path;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/**
 * @brief 方案快照服务。
 *
 * 该服务只负责：
 * 1. 通过各模块现有 Storage 读取已有结果；
 * 2. 生成轻量方案聚合摘要；
 * 3. 保存/加载 SchemeSnapshot。
 * 不复制上游模块原始对象，也不承载复杂对比或 UI 逻辑。
 */
class SchemeSnapshotService
{
public:
    SchemeSnapshotService(
        RoboSDP::Scheme::Persistence::SchemeJsonStorage& storage,
        RoboSDP::Requirement::Persistence::RequirementJsonStorage& requirementStorage,
        RoboSDP::Topology::Persistence::TopologyJsonStorage& topologyStorage,
        RoboSDP::Kinematics::Persistence::KinematicJsonStorage& kinematicStorage,
        RoboSDP::Dynamics::Persistence::DynamicJsonStorage& dynamicStorage,
        RoboSDP::Selection::Persistence::SelectionJsonStorage& selectionStorage,
        RoboSDP::Planning::Persistence::PlanningJsonStorage& planningStorage,
        RoboSDP::Logging::ILogger* logger = nullptr);

    RoboSDP::Scheme::Dto::SchemeSnapshotDto CreateDefaultSnapshot() const;

    SchemeBuildResult BuildSnapshot(const QString& projectRootPath) const;
    SchemeSaveResult SaveSnapshot(
        const QString& projectRootPath,
        const RoboSDP::Scheme::Dto::SchemeSnapshotDto& snapshot) const;
    SchemeLoadResult LoadSnapshot(const QString& projectRootPath) const;

private:
    RoboSDP::Errors::ErrorCode TryLoadRequirementSummary(
        const QString& projectRootPath,
        RoboSDP::Scheme::Dto::SchemeAggregateDto& aggregate) const;
    RoboSDP::Errors::ErrorCode TryLoadTopologySummary(
        const QString& projectRootPath,
        RoboSDP::Scheme::Dto::SchemeAggregateDto& aggregate) const;
    RoboSDP::Errors::ErrorCode TryLoadKinematicsSummary(
        const QString& projectRootPath,
        RoboSDP::Scheme::Dto::SchemeAggregateDto& aggregate) const;
    RoboSDP::Errors::ErrorCode TryLoadDynamicsSummary(
        const QString& projectRootPath,
        RoboSDP::Scheme::Dto::SchemeAggregateDto& aggregate) const;
    RoboSDP::Errors::ErrorCode TryLoadSelectionSummary(
        const QString& projectRootPath,
        RoboSDP::Scheme::Dto::SchemeAggregateDto& aggregate) const;
    RoboSDP::Errors::ErrorCode TryLoadPlanningSummary(
        const QString& projectRootPath,
        RoboSDP::Scheme::Dto::SchemeAggregateDto& aggregate) const;

private:
    RoboSDP::Scheme::Persistence::SchemeJsonStorage& m_storage;
    RoboSDP::Requirement::Persistence::RequirementJsonStorage& m_requirement_storage;
    RoboSDP::Topology::Persistence::TopologyJsonStorage& m_topology_storage;
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage& m_kinematic_storage;
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage& m_dynamic_storage;
    RoboSDP::Selection::Persistence::SelectionJsonStorage& m_selection_storage;
    RoboSDP::Planning::Persistence::PlanningJsonStorage& m_planning_storage;
    RoboSDP::Logging::ILogger* m_logger = nullptr;
};

} // namespace RoboSDP::Scheme::Service
