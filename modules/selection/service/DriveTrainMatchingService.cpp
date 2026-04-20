#include "modules/selection/service/DriveTrainMatchingService.h"

#include <QCoreApplication>
#include <QDir>

#include <algorithm>
#include <cmath>

namespace RoboSDP::Selection::Service
{

namespace
{

double PeakSpeedDegPerSecondForJoint(
    const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& dynamicState,
    const QString& jointId)
{
    double value = 0.0;
    for (const auto& peakStat : dynamicState.peak_stats)
    {
        if (peakStat.joint_id == jointId)
        {
            value = std::max(value, peakStat.peak_speed_deg_s);
        }
    }
    return value;
}

double PeakPowerForJoint(
    const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& dynamicState,
    const QString& jointId)
{
    double value = 0.0;
    for (const auto& peakStat : dynamicState.peak_stats)
    {
        if (peakStat.joint_id == jointId)
        {
            value = std::max(value, peakStat.peak_power_w);
        }
    }
    return value;
}

double FindTargetRatio(
    const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& dynamicState,
    const QString& jointId)
{
    for (const auto& joint : dynamicState.current_model.joints)
    {
        if (joint.joint_id == jointId)
        {
            return std::max(10.0, joint.transmission_ratio);
        }
    }
    return 80.0;
}

bool IsBrakeRequired(const QString& jointId)
{
    return jointId == QStringLiteral("joint_2") || jointId == QStringLiteral("joint_3");
}

} // namespace

DriveTrainMatchingService::DriveTrainMatchingService(
    RoboSDP::Selection::Persistence::SelectionJsonStorage& storage,
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage& dynamicStorage,
    RoboSDP::Logging::ILogger* logger)
    : m_storage(storage)
    , m_dynamic_storage(dynamicStorage)
    , m_logger(logger)
{
}

RoboSDP::Selection::Dto::SelectionWorkspaceStateDto DriveTrainMatchingService::CreateDefaultState() const
{
    return RoboSDP::Selection::Dto::SelectionWorkspaceStateDto::CreateDefault();
}

DriveTrainRunResult DriveTrainMatchingService::RunSelection(
    const QString& projectRootPath,
    const QString& catalogRootPath) const
{
    DriveTrainRunResult result;
    result.state = CreateDefaultState();
    result.dynamic_file_path = m_dynamic_storage.BuildAbsoluteModelFilePath(projectRootPath);
    result.catalog_directory_path = ResolveCatalogRootPath(catalogRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法执行驱动选型。");
        return result;
    }

    RoboSDP::Selection::Catalog::JsonComponentCatalog catalog;
    const auto catalogLoadResult = catalog.LoadFromDirectory(result.catalog_directory_path);
    if (!catalogLoadResult.IsSuccess())
    {
        result.error_code = catalogLoadResult.error_code;
        result.message = catalogLoadResult.message;
        return result;
    }

    RoboSDP::Selection::Service::MotorSelectionService motorService(catalog);
    RoboSDP::Selection::Service::ReducerSelectionService reducerService(catalog);

    RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto dynamicState;
    result.error_code = m_dynamic_storage.Load(projectRootPath, dynamicState);
    if (result.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.message = QStringLiteral("读取 Dynamics 草稿失败，无法执行驱动选型：%1")
                             .arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        return result;
    }

    result.state.drive_train_result.dynamic_ref = dynamicState.current_model.meta.dynamic_id;
    result.state.drive_train_result.kinematic_ref = dynamicState.current_model.meta.kinematic_ref;
    result.state.drive_train_result.topology_ref = dynamicState.current_model.meta.topology_ref;
    result.state.drive_train_result.requirement_ref = dynamicState.current_model.meta.requirement_ref;

    bool allJointsRecommended = true;
    for (const auto& envelopeJoint : dynamicState.load_envelope.joints)
    {
        const auto motorRequest = BuildMotorRequest(dynamicState, envelopeJoint);
        const auto reducerRequest = BuildReducerRequest(dynamicState, envelopeJoint);

        const auto motorResult = motorService.Select(motorRequest);
        const auto reducerResult = reducerService.Select(reducerRequest);
        result.state.motor_results.push_back(motorResult);
        result.state.reducer_results.push_back(reducerResult);

        RoboSDP::Selection::Dto::DriveTrainJointSelectionDto jointSelection;
        jointSelection.joint_id = envelopeJoint.joint_id;

        const double requiredPeakSpeed = PeakSpeedDegPerSecondForJoint(dynamicState, envelopeJoint.joint_id);
        const double requiredBrakeTorque = motorRequest.required_brake_output_torque_nm;
        const double targetRatio = reducerRequest.target_ratio;

        for (const auto& motorCandidate : motorResult.candidates)
        {
            for (const auto& reducerCandidate : reducerResult.candidates)
            {
                RoboSDP::Selection::Dto::DriveTrainMatchCandidateDto candidate;
                candidate.joint_id = envelopeJoint.joint_id;
                candidate.motor_id = motorCandidate.motor.motor_id;
                candidate.reducer_id = reducerCandidate.reducer.reducer_id;
                candidate.motor_name = motorCandidate.motor.name;
                candidate.reducer_name = reducerCandidate.reducer.name;
                candidate.combined_ratio = reducerCandidate.reducer.ratio;
                candidate.total_efficiency = motorCandidate.motor.efficiency * reducerCandidate.reducer.efficiency;
                candidate.output_rated_torque_nm =
                    motorCandidate.motor.rated_torque_nm * reducerCandidate.reducer.ratio * reducerCandidate.reducer.efficiency;
                candidate.output_peak_torque_nm =
                    motorCandidate.motor.peak_torque_nm * reducerCandidate.reducer.ratio * reducerCandidate.reducer.efficiency;
                candidate.output_peak_speed_deg_s =
                    motorCandidate.motor.max_speed_rpm * 6.0 / std::max(1.0, reducerCandidate.reducer.ratio);
                candidate.brake_check_passed = !motorRequest.brake_required ||
                    (motorCandidate.motor.has_brake &&
                     motorCandidate.motor.brake_holding_torque_nm * reducerCandidate.reducer.ratio * reducerCandidate.reducer.efficiency >=
                         requiredBrakeTorque);

                const bool torqueRulePassed =
                    candidate.output_rated_torque_nm + 1e-6 >= envelopeJoint.rms_torque_nm &&
                    candidate.output_peak_torque_nm + 1e-6 >= envelopeJoint.peak_torque_nm;
                const bool speedRulePassed = candidate.output_peak_speed_deg_s + 1e-6 >= requiredPeakSpeed;

                if (!torqueRulePassed || !speedRulePassed || !candidate.brake_check_passed)
                {
                    continue;
                }

                const double ratioDeviation = std::abs(candidate.combined_ratio - targetRatio);
                const double oversizePenalty =
                    std::max(0.0, candidate.output_peak_torque_nm - envelopeJoint.peak_torque_nm) * 0.01;
                candidate.score = candidate.total_efficiency * 100.0 - ratioDeviation * 0.8 - oversizePenalty;
                candidate.recommendation_reason = QStringLiteral("满足输出扭矩、速度与基础抱闸校核，且综合效率较优。");
                jointSelection.candidates.push_back(candidate);
            }
        }

        std::sort(
            jointSelection.candidates.begin(),
            jointSelection.candidates.end(),
            [](const auto& left, const auto& right) { return left.score > right.score; });

        if (!jointSelection.candidates.empty())
        {
            jointSelection.has_recommendation = true;
            jointSelection.recommended = jointSelection.candidates.front();
            jointSelection.message = QStringLiteral("已生成联合驱动链推荐。");
        }
        else
        {
            jointSelection.message = QStringLiteral("未找到满足基础联合规则的驱动链组合。");
            allJointsRecommended = false;
        }

        result.state.drive_train_result.joint_selections.push_back(jointSelection);
    }

    result.state.drive_train_result.success =
        allJointsRecommended && !result.state.drive_train_result.joint_selections.empty();
    result.state.drive_train_result.message = result.state.drive_train_result.success
        ? QStringLiteral("驱动选型最小闭环执行完成。")
        : QStringLiteral("驱动选型执行完成，但存在未命中推荐的关节。");
    result.message = QStringLiteral("%1 样例目录：%2")
                         .arg(result.state.drive_train_result.message, result.catalog_directory_path);
    return result;
}

QString DriveTrainMatchingService::ResolveCatalogRootPath(const QString& catalogRootPath) const
{
    if (!catalogRootPath.trimmed().isEmpty())
    {
        return QDir(catalogRootPath).absolutePath();
    }

    const QString currentRoot = QDir::current().absolutePath();
    const QString applicationRoot = QCoreApplication::applicationDirPath();
    const QStringList candidates {
        QDir(currentRoot).absoluteFilePath(QStringLiteral("resources/selection")),
        QDir(applicationRoot).absoluteFilePath(QStringLiteral("resources/selection")),
        QDir(applicationRoot).absoluteFilePath(QStringLiteral("../resources/selection")),
        QDir(applicationRoot).absoluteFilePath(QStringLiteral("../../resources/selection")),
        QDir(applicationRoot).absoluteFilePath(QStringLiteral("../../../resources/selection")),
        QDir(applicationRoot).absoluteFilePath(QStringLiteral("../../../../resources/selection"))};

    for (const QString& candidate : candidates)
    {
        if (QDir(candidate).exists())
        {
            return QDir(candidate).absolutePath();
        }
    }

    return QDir(currentRoot).absoluteFilePath(QStringLiteral("resources/selection"));
}

DriveTrainSaveResult DriveTrainMatchingService::SaveDraft(
    const QString& projectRootPath,
    const RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const
{
    DriveTrainSaveResult result;
    result.motor_file_path = m_storage.BuildAbsoluteMotorFilePath(projectRootPath);
    result.reducer_file_path = m_storage.BuildAbsoluteReducerFilePath(projectRootPath);
    result.drivetrain_file_path = m_storage.BuildAbsoluteDriveTrainFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法保存选型结果。");
        return result;
    }

    result.error_code = m_storage.Save(projectRootPath, state);
    result.message = result.IsSuccess()
        ? QStringLiteral("驱动选型结果已保存。")
        : QStringLiteral("驱动选型结果保存失败：%1").arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
    return result;
}

DriveTrainLoadResult DriveTrainMatchingService::LoadDraft(const QString& projectRootPath) const
{
    DriveTrainLoadResult result;
    result.motor_file_path = m_storage.BuildAbsoluteMotorFilePath(projectRootPath);
    result.reducer_file_path = m_storage.BuildAbsoluteReducerFilePath(projectRootPath);
    result.drivetrain_file_path = m_storage.BuildAbsoluteDriveTrainFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法加载选型结果。");
        return result;
    }

    result.error_code = m_storage.Load(projectRootPath, result.state);
    result.message = result.IsSuccess()
        ? QStringLiteral("驱动选型结果已从 JSON 重新加载。")
        : QStringLiteral("驱动选型结果加载失败：%1").arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
    return result;
}

RoboSDP::Selection::Dto::MotorSelectionRequestDto DriveTrainMatchingService::BuildMotorRequest(
    const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& dynamicState,
    const RoboSDP::Dynamics::Dto::LoadEnvelopeJointDto& loadEnvelopeJoint) const
{
    RoboSDP::Selection::Dto::MotorSelectionRequestDto request;
    request.dynamic_ref = dynamicState.current_model.meta.dynamic_id;
    request.kinematic_ref = dynamicState.current_model.meta.kinematic_ref;
    request.topology_ref = dynamicState.current_model.meta.topology_ref;
    request.requirement_ref = dynamicState.current_model.meta.requirement_ref;
    request.joint_id = loadEnvelopeJoint.joint_id;
    request.peak_output_torque_nm = loadEnvelopeJoint.peak_torque_nm;
    request.rms_output_torque_nm = loadEnvelopeJoint.rms_torque_nm;
    request.peak_output_speed_deg_s = PeakSpeedDegPerSecondForJoint(dynamicState, loadEnvelopeJoint.joint_id);
    request.peak_output_power_w = PeakPowerForJoint(dynamicState, loadEnvelopeJoint.joint_id);

    const double targetRatio = FindTargetRatio(dynamicState, loadEnvelopeJoint.joint_id);
    request.preferred_ratio_min = std::max(10.0, targetRatio * 0.80);
    request.preferred_ratio_max = std::max(request.preferred_ratio_min, targetRatio * 1.20);
    request.brake_required = IsBrakeRequired(loadEnvelopeJoint.joint_id);
    request.required_brake_output_torque_nm = request.brake_required ? loadEnvelopeJoint.rms_torque_nm * 1.20 : 0.0;
    return request;
}

RoboSDP::Selection::Dto::ReducerSelectionRequestDto DriveTrainMatchingService::BuildReducerRequest(
    const RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto& dynamicState,
    const RoboSDP::Dynamics::Dto::LoadEnvelopeJointDto& loadEnvelopeJoint) const
{
    RoboSDP::Selection::Dto::ReducerSelectionRequestDto request;
    request.dynamic_ref = dynamicState.current_model.meta.dynamic_id;
    request.kinematic_ref = dynamicState.current_model.meta.kinematic_ref;
    request.topology_ref = dynamicState.current_model.meta.topology_ref;
    request.requirement_ref = dynamicState.current_model.meta.requirement_ref;
    request.joint_id = loadEnvelopeJoint.joint_id;
    request.peak_output_torque_nm = loadEnvelopeJoint.peak_torque_nm;
    request.rms_output_torque_nm = loadEnvelopeJoint.rms_torque_nm;
    request.peak_output_speed_deg_s = PeakSpeedDegPerSecondForJoint(dynamicState, loadEnvelopeJoint.joint_id);

    const double targetRatio = FindTargetRatio(dynamicState, loadEnvelopeJoint.joint_id);
    request.target_ratio = targetRatio;
    request.target_ratio_min = std::max(10.0, targetRatio * 0.80);
    request.target_ratio_max = std::max(request.target_ratio_min, targetRatio * 1.20);
    request.motor_speed_limit_rpm = request.peak_output_speed_deg_s / 6.0 * request.target_ratio_max;
    return request;
}

} // namespace RoboSDP::Selection::Service
