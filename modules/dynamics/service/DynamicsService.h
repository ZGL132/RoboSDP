#pragma once

#include "core/errors/ErrorCode.h"
#include "core/logging/ILogger.h"
#include "modules/dynamics/adapter/PinocchioDynamicsBackendAdapter.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/dynamics/trajectory/BenchmarkTrajectoryFactory.h"
#include "modules/dynamics/trajectory/TimeParameterizationSkeleton.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"

namespace RoboSDP::Dynamics::Service
{

/// @brief 从 Kinematics 构建 DynamicModel 的结果。
struct DynamicsBuildResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString kinematic_file_path;
    RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto state;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// @brief Dynamics 主链分析结果。
struct DynamicsAnalyzeResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString kinematic_file_path;
    RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto state;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// @brief Dynamics 草稿保存结果。
struct DynamicsSaveResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString model_file_path;
    QString mass_file_path;
    QString envelope_file_path;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/// @brief Dynamics 草稿加载结果。
struct DynamicsLoadResult
{
    RoboSDP::Errors::ErrorCode error_code = RoboSDP::Errors::ErrorCode::Ok;
    QString message;
    QString model_file_path;
    QString mass_file_path;
    QString envelope_file_path;
    RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto state;

    bool IsSuccess() const
    {
        return error_code == RoboSDP::Errors::ErrorCode::Ok;
    }
};

/**
 * @brief Dynamics 服务层。
 * @details
 * 当前服务层负责：
 * 1. 从 Kinematics 草稿构建最小 DynamicModel；
 * 2. 生成基准轨迹并完成时间参数化；
 * 3. 调用共享 Pinocchio 内核执行轨迹逆动力学；
 * 4. 汇总峰值、RMS 与负载包络；
 * 5. 负责草稿保存、加载与干跑诊断透传。
 */
class DynamicsService
{
public:
    DynamicsService(
        RoboSDP::Dynamics::Persistence::DynamicJsonStorage& storage,
        RoboSDP::Kinematics::Persistence::KinematicJsonStorage& kinematicStorage,
        RoboSDP::Logging::ILogger* logger = nullptr);

    RoboSDP::Dynamics::Dto::DynamicModelDto CreateDefaultModel() const;
    RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto CreateDefaultState() const;

    DynamicsBuildResult BuildFromKinematics(const QString& projectRootPath) const;

    DynamicsAnalyzeResult RunInverseDynamicsChain(
        const QString& projectRootPath,
        const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& inputState) const;

    DynamicsSaveResult SaveDraft(
        const QString& projectRootPath,
        const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& state) const;

    DynamicsLoadResult LoadDraft(const QString& projectRootPath) const;

    /// @brief 仅供测试与诊断使用的 Pinocchio 原生 RNEA 干跑入口。
    RoboSDP::Dynamics::Dto::NativeRneaDryRunResultDto InspectNativeRneaDryRun(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& kinematicModel,
        const RoboSDP::Dynamics::Dto::DynamicModelDto& dynamicModel,
        const std::vector<double>& joint_positions_deg,
        const std::vector<double>& joint_velocities_deg_s,
        const std::vector<double>& joint_accelerations_deg_s2) const;

private:
    RoboSDP::Dynamics::Dto::DynamicModelDto BuildModelFromKinematics(
        const RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& kinematicState) const;

private:
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage& m_storage;
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage& m_kinematic_storage;
    RoboSDP::Logging::ILogger* m_logger = nullptr;
    RoboSDP::Dynamics::Trajectory::BenchmarkTrajectoryFactory m_trajectory_factory;
    RoboSDP::Dynamics::Trajectory::TimeParameterizationSkeleton m_time_parameterization;
    RoboSDP::Dynamics::Adapter::PinocchioDynamicsBackendAdapter m_pinocchio_dynamics_backend_adapter;
};

} // namespace RoboSDP::Dynamics::Service
