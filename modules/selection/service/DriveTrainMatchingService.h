#pragma once

#include "core/errors/ErrorCode.h"
#include "core/logging/ILogger.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/selection/catalog/JsonComponentCatalog.h"
#include "modules/selection/persistence/SelectionJsonStorage.h"
#include "modules/selection/service/MotorSelectionService.h"
#include "modules/selection/service/ReducerSelectionService.h"

namespace RoboSDP::Selection::Service
{

/// 驱动链运行结果。
struct DriveTrainRunResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString dynamic_file_path;
    QString catalog_directory_path;
    RoboSDP::Selection::Dto::SelectionWorkspaceStateDto state;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// 驱动链保存结果。
struct DriveTrainSaveResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString motor_file_path;
    QString reducer_file_path;
    QString drivetrain_file_path;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// 驱动链加载结果。
struct DriveTrainLoadResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString motor_file_path;
    QString reducer_file_path;
    QString drivetrain_file_path;
    RoboSDP::Selection::Dto::SelectionWorkspaceStateDto state;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/**
 * @brief 联合驱动链匹配服务。
 *
 * 该服务负责：
 * 1. 从 Dynamics 草稿读取关节负载包络；
 * 2. 构造电机与减速器选型请求；
 * 3. 组合候选并执行基础抱闸与输出能力校核；
 * 4. 生成最小联合推荐结果并支持保存/加载。
 */
class DriveTrainMatchingService
{
public:
    DriveTrainMatchingService(
        RoboSDP::Selection::Persistence::SelectionJsonStorage& storage,
        RoboSDP::Dynamics::Persistence::DynamicJsonStorage& dynamicStorage,
        RoboSDP::Logging::ILogger* logger = nullptr);

    RoboSDP::Selection::Dto::SelectionWorkspaceStateDto CreateDefaultState() const;

    DriveTrainRunResult RunSelection(
        const QString& projectRootPath,
        const QString& catalogRootPath = QString()) const;
    DriveTrainSaveResult SaveDraft(
        const QString& projectRootPath,
        const RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const;
    DriveTrainLoadResult LoadDraft(const QString& projectRootPath) const;

private:
    /// 解析样例目录路径，并兼容从 build 目录直接启动桌面程序的场景。
    QString ResolveCatalogRootPath(const QString& catalogRootPath) const;

    RoboSDP::Selection::Dto::MotorSelectionRequestDto BuildMotorRequest(
        const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& dynamicState,
        const RoboSDP::Dynamics::Dto::LoadEnvelopeJointDto& loadEnvelopeJoint) const;

    RoboSDP::Selection::Dto::ReducerSelectionRequestDto BuildReducerRequest(
        const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& dynamicState,
        const RoboSDP::Dynamics::Dto::LoadEnvelopeJointDto& loadEnvelopeJoint) const;

private:
    RoboSDP::Selection::Persistence::SelectionJsonStorage& m_storage;
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage& m_dynamic_storage;
    RoboSDP::Logging::ILogger* m_logger = nullptr;
};

} // namespace RoboSDP::Selection::Service
