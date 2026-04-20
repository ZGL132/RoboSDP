#include "modules/selection/persistence/SelectionJsonStorage.h"

#include <QDir>
#include <QJsonArray>

namespace RoboSDP::Selection::Persistence
{

namespace
{

QJsonObject ToMotorCatalogJson(const RoboSDP::Selection::Dto::MotorCatalogItemDto& motor)
{
    return {
        {QStringLiteral("motor_id"), motor.motor_id},
        {QStringLiteral("name"), motor.name},
        {QStringLiteral("vendor"), motor.vendor},
        {QStringLiteral("rated_torque_nm"), motor.rated_torque_nm},
        {QStringLiteral("peak_torque_nm"), motor.peak_torque_nm},
        {QStringLiteral("rated_speed_rpm"), motor.rated_speed_rpm},
        {QStringLiteral("max_speed_rpm"), motor.max_speed_rpm},
        {QStringLiteral("rated_power_w"), motor.rated_power_w},
        {QStringLiteral("rotor_inertia_kg_m2"), motor.rotor_inertia_kg_m2},
        {QStringLiteral("efficiency"), motor.efficiency},
        {QStringLiteral("has_brake"), motor.has_brake},
        {QStringLiteral("brake_holding_torque_nm"), motor.brake_holding_torque_nm}};
}

RoboSDP::Selection::Dto::MotorCatalogItemDto MotorFromJson(const QJsonObject& jsonObject)
{
    RoboSDP::Selection::Dto::MotorCatalogItemDto motor;
    motor.motor_id = jsonObject.value(QStringLiteral("motor_id")).toString();
    motor.name = jsonObject.value(QStringLiteral("name")).toString();
    motor.vendor = jsonObject.value(QStringLiteral("vendor")).toString();
    motor.rated_torque_nm = jsonObject.value(QStringLiteral("rated_torque_nm")).toDouble();
    motor.peak_torque_nm = jsonObject.value(QStringLiteral("peak_torque_nm")).toDouble();
    motor.rated_speed_rpm = jsonObject.value(QStringLiteral("rated_speed_rpm")).toDouble();
    motor.max_speed_rpm = jsonObject.value(QStringLiteral("max_speed_rpm")).toDouble();
    motor.rated_power_w = jsonObject.value(QStringLiteral("rated_power_w")).toDouble();
    motor.rotor_inertia_kg_m2 = jsonObject.value(QStringLiteral("rotor_inertia_kg_m2")).toDouble();
    motor.efficiency = jsonObject.value(QStringLiteral("efficiency")).toDouble();
    motor.has_brake = jsonObject.value(QStringLiteral("has_brake")).toBool();
    motor.brake_holding_torque_nm = jsonObject.value(QStringLiteral("brake_holding_torque_nm")).toDouble();
    return motor;
}

QJsonObject ToReducerCatalogJson(const RoboSDP::Selection::Dto::ReducerCatalogItemDto& reducer)
{
    return {
        {QStringLiteral("reducer_id"), reducer.reducer_id},
        {QStringLiteral("name"), reducer.name},
        {QStringLiteral("vendor"), reducer.vendor},
        {QStringLiteral("ratio"), reducer.ratio},
        {QStringLiteral("rated_output_torque_nm"), reducer.rated_output_torque_nm},
        {QStringLiteral("peak_output_torque_nm"), reducer.peak_output_torque_nm},
        {QStringLiteral("max_input_speed_rpm"), reducer.max_input_speed_rpm},
        {QStringLiteral("efficiency"), reducer.efficiency},
        {QStringLiteral("backlash_arcmin"), reducer.backlash_arcmin}};
}

RoboSDP::Selection::Dto::ReducerCatalogItemDto ReducerFromJson(const QJsonObject& jsonObject)
{
    RoboSDP::Selection::Dto::ReducerCatalogItemDto reducer;
    reducer.reducer_id = jsonObject.value(QStringLiteral("reducer_id")).toString();
    reducer.name = jsonObject.value(QStringLiteral("name")).toString();
    reducer.vendor = jsonObject.value(QStringLiteral("vendor")).toString();
    reducer.ratio = jsonObject.value(QStringLiteral("ratio")).toDouble();
    reducer.rated_output_torque_nm = jsonObject.value(QStringLiteral("rated_output_torque_nm")).toDouble();
    reducer.peak_output_torque_nm = jsonObject.value(QStringLiteral("peak_output_torque_nm")).toDouble();
    reducer.max_input_speed_rpm = jsonObject.value(QStringLiteral("max_input_speed_rpm")).toDouble();
    reducer.efficiency = jsonObject.value(QStringLiteral("efficiency")).toDouble();
    reducer.backlash_arcmin = jsonObject.value(QStringLiteral("backlash_arcmin")).toDouble();
    return reducer;
}

QJsonObject ToMotorResultJson(const RoboSDP::Selection::Dto::MotorSelectionResultDto& result)
{
    QJsonArray candidates;
    for (const auto& candidate : result.candidates)
    {
        candidates.push_back(QJsonObject {
            {QStringLiteral("motor"), ToMotorCatalogJson(candidate.motor)},
            {QStringLiteral("estimated_peak_output_torque_nm"), candidate.estimated_peak_output_torque_nm},
            {QStringLiteral("estimated_rated_output_torque_nm"), candidate.estimated_rated_output_torque_nm},
            {QStringLiteral("estimated_peak_output_speed_deg_s"), candidate.estimated_peak_output_speed_deg_s},
            {QStringLiteral("torque_margin_nm"), candidate.torque_margin_nm},
            {QStringLiteral("speed_margin_deg_s"), candidate.speed_margin_deg_s},
            {QStringLiteral("power_margin_w"), candidate.power_margin_w},
            {QStringLiteral("brake_rule_passed"), candidate.brake_rule_passed},
            {QStringLiteral("recommendation_reason"), candidate.recommendation_reason}});
    }

    return QJsonObject {
        {QStringLiteral("dynamic_ref"), result.request.dynamic_ref},
        {QStringLiteral("kinematic_ref"), result.request.kinematic_ref},
        {QStringLiteral("topology_ref"), result.request.topology_ref},
        {QStringLiteral("requirement_ref"), result.request.requirement_ref},
        {QStringLiteral("joint_id"), result.request.joint_id},
        {QStringLiteral("peak_output_torque_nm"), result.request.peak_output_torque_nm},
        {QStringLiteral("rms_output_torque_nm"), result.request.rms_output_torque_nm},
        {QStringLiteral("peak_output_speed_deg_s"), result.request.peak_output_speed_deg_s},
        {QStringLiteral("peak_output_power_w"), result.request.peak_output_power_w},
        {QStringLiteral("preferred_ratio_min"), result.request.preferred_ratio_min},
        {QStringLiteral("preferred_ratio_max"), result.request.preferred_ratio_max},
        {QStringLiteral("brake_required"), result.request.brake_required},
        {QStringLiteral("required_brake_output_torque_nm"), result.request.required_brake_output_torque_nm},
        {QStringLiteral("candidates"), candidates},
        {QStringLiteral("has_recommendation"), result.has_recommendation},
        {QStringLiteral("recommended_index"), result.candidates.empty() ? -1 : 0},
        {QStringLiteral("message"), result.message}};
}

RoboSDP::Selection::Dto::MotorSelectionResultDto MotorResultFromJson(const QJsonObject& jsonObject)
{
    RoboSDP::Selection::Dto::MotorSelectionResultDto result;
    result.request.dynamic_ref = jsonObject.value(QStringLiteral("dynamic_ref")).toString();
    result.request.kinematic_ref = jsonObject.value(QStringLiteral("kinematic_ref")).toString();
    result.request.topology_ref = jsonObject.value(QStringLiteral("topology_ref")).toString();
    result.request.requirement_ref = jsonObject.value(QStringLiteral("requirement_ref")).toString();
    result.request.joint_id = jsonObject.value(QStringLiteral("joint_id")).toString();
    result.request.peak_output_torque_nm = jsonObject.value(QStringLiteral("peak_output_torque_nm")).toDouble();
    result.request.rms_output_torque_nm = jsonObject.value(QStringLiteral("rms_output_torque_nm")).toDouble();
    result.request.peak_output_speed_deg_s = jsonObject.value(QStringLiteral("peak_output_speed_deg_s")).toDouble();
    result.request.peak_output_power_w = jsonObject.value(QStringLiteral("peak_output_power_w")).toDouble();
    result.request.preferred_ratio_min = jsonObject.value(QStringLiteral("preferred_ratio_min")).toDouble();
    result.request.preferred_ratio_max = jsonObject.value(QStringLiteral("preferred_ratio_max")).toDouble();
    result.request.brake_required = jsonObject.value(QStringLiteral("brake_required")).toBool();
    result.request.required_brake_output_torque_nm =
        jsonObject.value(QStringLiteral("required_brake_output_torque_nm")).toDouble();
    result.has_recommendation = jsonObject.value(QStringLiteral("has_recommendation")).toBool();
    result.message = jsonObject.value(QStringLiteral("message")).toString();

    for (const auto& value : jsonObject.value(QStringLiteral("candidates")).toArray())
    {
        const QJsonObject object = value.toObject();
        RoboSDP::Selection::Dto::MotorSelectionCandidateDto candidate;
        candidate.motor = MotorFromJson(object.value(QStringLiteral("motor")).toObject());
        candidate.estimated_peak_output_torque_nm = object.value(QStringLiteral("estimated_peak_output_torque_nm")).toDouble();
        candidate.estimated_rated_output_torque_nm = object.value(QStringLiteral("estimated_rated_output_torque_nm")).toDouble();
        candidate.estimated_peak_output_speed_deg_s = object.value(QStringLiteral("estimated_peak_output_speed_deg_s")).toDouble();
        candidate.torque_margin_nm = object.value(QStringLiteral("torque_margin_nm")).toDouble();
        candidate.speed_margin_deg_s = object.value(QStringLiteral("speed_margin_deg_s")).toDouble();
        candidate.power_margin_w = object.value(QStringLiteral("power_margin_w")).toDouble();
        candidate.brake_rule_passed = object.value(QStringLiteral("brake_rule_passed")).toBool();
        candidate.recommendation_reason = object.value(QStringLiteral("recommendation_reason")).toString();
        result.candidates.push_back(candidate);
    }

    if (result.has_recommendation && !result.candidates.empty())
    {
        result.recommended = result.candidates.front();
    }
    return result;
}

QJsonObject ToReducerResultJson(const RoboSDP::Selection::Dto::ReducerSelectionResultDto& result)
{
    QJsonArray candidates;
    for (const auto& candidate : result.candidates)
    {
        candidates.push_back(QJsonObject {
            {QStringLiteral("reducer"), ToReducerCatalogJson(candidate.reducer)},
            {QStringLiteral("ratio_deviation"), candidate.ratio_deviation},
            {QStringLiteral("rated_torque_margin_nm"), candidate.rated_torque_margin_nm},
            {QStringLiteral("peak_torque_margin_nm"), candidate.peak_torque_margin_nm},
            {QStringLiteral("ratio_rule_passed"), candidate.ratio_rule_passed},
            {QStringLiteral("recommendation_reason"), candidate.recommendation_reason}});
    }

    return QJsonObject {
        {QStringLiteral("dynamic_ref"), result.request.dynamic_ref},
        {QStringLiteral("kinematic_ref"), result.request.kinematic_ref},
        {QStringLiteral("topology_ref"), result.request.topology_ref},
        {QStringLiteral("requirement_ref"), result.request.requirement_ref},
        {QStringLiteral("joint_id"), result.request.joint_id},
        {QStringLiteral("peak_output_torque_nm"), result.request.peak_output_torque_nm},
        {QStringLiteral("rms_output_torque_nm"), result.request.rms_output_torque_nm},
        {QStringLiteral("peak_output_speed_deg_s"), result.request.peak_output_speed_deg_s},
        {QStringLiteral("target_ratio"), result.request.target_ratio},
        {QStringLiteral("target_ratio_min"), result.request.target_ratio_min},
        {QStringLiteral("target_ratio_max"), result.request.target_ratio_max},
        {QStringLiteral("motor_speed_limit_rpm"), result.request.motor_speed_limit_rpm},
        {QStringLiteral("candidates"), candidates},
        {QStringLiteral("has_recommendation"), result.has_recommendation},
        {QStringLiteral("message"), result.message}};
}

RoboSDP::Selection::Dto::ReducerSelectionResultDto ReducerResultFromJson(const QJsonObject& jsonObject)
{
    RoboSDP::Selection::Dto::ReducerSelectionResultDto result;
    result.request.dynamic_ref = jsonObject.value(QStringLiteral("dynamic_ref")).toString();
    result.request.kinematic_ref = jsonObject.value(QStringLiteral("kinematic_ref")).toString();
    result.request.topology_ref = jsonObject.value(QStringLiteral("topology_ref")).toString();
    result.request.requirement_ref = jsonObject.value(QStringLiteral("requirement_ref")).toString();
    result.request.joint_id = jsonObject.value(QStringLiteral("joint_id")).toString();
    result.request.peak_output_torque_nm = jsonObject.value(QStringLiteral("peak_output_torque_nm")).toDouble();
    result.request.rms_output_torque_nm = jsonObject.value(QStringLiteral("rms_output_torque_nm")).toDouble();
    result.request.peak_output_speed_deg_s = jsonObject.value(QStringLiteral("peak_output_speed_deg_s")).toDouble();
    result.request.target_ratio = jsonObject.value(QStringLiteral("target_ratio")).toDouble();
    result.request.target_ratio_min = jsonObject.value(QStringLiteral("target_ratio_min")).toDouble();
    result.request.target_ratio_max = jsonObject.value(QStringLiteral("target_ratio_max")).toDouble();
    result.request.motor_speed_limit_rpm = jsonObject.value(QStringLiteral("motor_speed_limit_rpm")).toDouble();
    result.has_recommendation = jsonObject.value(QStringLiteral("has_recommendation")).toBool();
    result.message = jsonObject.value(QStringLiteral("message")).toString();

    for (const auto& value : jsonObject.value(QStringLiteral("candidates")).toArray())
    {
        const QJsonObject object = value.toObject();
        RoboSDP::Selection::Dto::ReducerSelectionCandidateDto candidate;
        candidate.reducer = ReducerFromJson(object.value(QStringLiteral("reducer")).toObject());
        candidate.ratio_deviation = object.value(QStringLiteral("ratio_deviation")).toDouble();
        candidate.rated_torque_margin_nm = object.value(QStringLiteral("rated_torque_margin_nm")).toDouble();
        candidate.peak_torque_margin_nm = object.value(QStringLiteral("peak_torque_margin_nm")).toDouble();
        candidate.ratio_rule_passed = object.value(QStringLiteral("ratio_rule_passed")).toBool();
        candidate.recommendation_reason = object.value(QStringLiteral("recommendation_reason")).toString();
        result.candidates.push_back(candidate);
    }

    if (result.has_recommendation && !result.candidates.empty())
    {
        result.recommended = result.candidates.front();
    }
    return result;
}

QJsonObject ToDriveTrainJson(const RoboSDP::Selection::Dto::DriveTrainSelectionResultDto& result)
{
    QJsonArray joints;
    for (const auto& joint : result.joint_selections)
    {
        QJsonArray candidates;
        for (const auto& candidate : joint.candidates)
        {
            candidates.push_back(QJsonObject {
                {QStringLiteral("joint_id"), candidate.joint_id},
                {QStringLiteral("motor_id"), candidate.motor_id},
                {QStringLiteral("reducer_id"), candidate.reducer_id},
                {QStringLiteral("motor_name"), candidate.motor_name},
                {QStringLiteral("reducer_name"), candidate.reducer_name},
                {QStringLiteral("combined_ratio"), candidate.combined_ratio},
                {QStringLiteral("total_efficiency"), candidate.total_efficiency},
                {QStringLiteral("output_rated_torque_nm"), candidate.output_rated_torque_nm},
                {QStringLiteral("output_peak_torque_nm"), candidate.output_peak_torque_nm},
                {QStringLiteral("output_peak_speed_deg_s"), candidate.output_peak_speed_deg_s},
                {QStringLiteral("brake_check_passed"), candidate.brake_check_passed},
                {QStringLiteral("score"), candidate.score},
                {QStringLiteral("recommendation_reason"), candidate.recommendation_reason}});
        }

        joints.push_back(QJsonObject {
            {QStringLiteral("joint_id"), joint.joint_id},
            {QStringLiteral("candidates"), candidates},
            {QStringLiteral("has_recommendation"), joint.has_recommendation},
            {QStringLiteral("message"), joint.message}});
    }

    return QJsonObject {
        {QStringLiteral("dynamic_ref"), result.dynamic_ref},
        {QStringLiteral("kinematic_ref"), result.kinematic_ref},
        {QStringLiteral("topology_ref"), result.topology_ref},
        {QStringLiteral("requirement_ref"), result.requirement_ref},
        {QStringLiteral("joint_selections"), joints},
        {QStringLiteral("success"), result.success},
        {QStringLiteral("message"), result.message}};
}

RoboSDP::Selection::Dto::DriveTrainSelectionResultDto DriveTrainFromJson(const QJsonObject& jsonObject)
{
    RoboSDP::Selection::Dto::DriveTrainSelectionResultDto result;
    result.dynamic_ref = jsonObject.value(QStringLiteral("dynamic_ref")).toString();
    result.kinematic_ref = jsonObject.value(QStringLiteral("kinematic_ref")).toString();
    result.topology_ref = jsonObject.value(QStringLiteral("topology_ref")).toString();
    result.requirement_ref = jsonObject.value(QStringLiteral("requirement_ref")).toString();
    result.success = jsonObject.value(QStringLiteral("success")).toBool();
    result.message = jsonObject.value(QStringLiteral("message")).toString();

    for (const auto& value : jsonObject.value(QStringLiteral("joint_selections")).toArray())
    {
        const QJsonObject jointObject = value.toObject();
        RoboSDP::Selection::Dto::DriveTrainJointSelectionDto joint;
        joint.joint_id = jointObject.value(QStringLiteral("joint_id")).toString();
        joint.has_recommendation = jointObject.value(QStringLiteral("has_recommendation")).toBool();
        joint.message = jointObject.value(QStringLiteral("message")).toString();

        for (const auto& candidateValue : jointObject.value(QStringLiteral("candidates")).toArray())
        {
            const QJsonObject object = candidateValue.toObject();
            RoboSDP::Selection::Dto::DriveTrainMatchCandidateDto candidate;
            candidate.joint_id = object.value(QStringLiteral("joint_id")).toString();
            candidate.motor_id = object.value(QStringLiteral("motor_id")).toString();
            candidate.reducer_id = object.value(QStringLiteral("reducer_id")).toString();
            candidate.motor_name = object.value(QStringLiteral("motor_name")).toString();
            candidate.reducer_name = object.value(QStringLiteral("reducer_name")).toString();
            candidate.combined_ratio = object.value(QStringLiteral("combined_ratio")).toDouble();
            candidate.total_efficiency = object.value(QStringLiteral("total_efficiency")).toDouble();
            candidate.output_rated_torque_nm = object.value(QStringLiteral("output_rated_torque_nm")).toDouble();
            candidate.output_peak_torque_nm = object.value(QStringLiteral("output_peak_torque_nm")).toDouble();
            candidate.output_peak_speed_deg_s = object.value(QStringLiteral("output_peak_speed_deg_s")).toDouble();
            candidate.brake_check_passed = object.value(QStringLiteral("brake_check_passed")).toBool();
            candidate.score = object.value(QStringLiteral("score")).toDouble();
            candidate.recommendation_reason = object.value(QStringLiteral("recommendation_reason")).toString();
            joint.candidates.push_back(candidate);
        }

        if (joint.has_recommendation && !joint.candidates.empty())
        {
            joint.recommended = joint.candidates.front();
        }
        result.joint_selections.push_back(joint);
    }
    return result;
}

} // namespace

SelectionJsonStorage::SelectionJsonStorage(RoboSDP::Repository::IJsonRepository& repository)
    : m_repository(repository)
{
}

RoboSDP::Errors::ErrorCode SelectionJsonStorage::Save(
    const QString& projectRootPath,
    const RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const
{
    auto errorCode = m_repository.OpenProject(projectRootPath);
    if (errorCode != RoboSDP::Errors::ErrorCode::Ok)
    {
        return errorCode;
    }

    errorCode = m_repository.WriteDocument(RelativeMotorFilePath(), ToMotorJsonObject(state));
    if (errorCode != RoboSDP::Errors::ErrorCode::Ok)
    {
        return errorCode;
    }

    errorCode = m_repository.WriteDocument(RelativeReducerFilePath(), ToReducerJsonObject(state));
    if (errorCode != RoboSDP::Errors::ErrorCode::Ok)
    {
        return errorCode;
    }

    return m_repository.WriteDocument(RelativeDriveTrainFilePath(), ToDriveTrainJsonObject(state));
}

RoboSDP::Errors::ErrorCode SelectionJsonStorage::Load(
    const QString& projectRootPath,
    RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const
{
    auto errorCode = m_repository.OpenProject(projectRootPath);
    if (errorCode != RoboSDP::Errors::ErrorCode::Ok)
    {
        return errorCode;
    }

    QJsonObject motorObject;
    errorCode = m_repository.ReadDocument(RelativeMotorFilePath(), motorObject);
    if (errorCode != RoboSDP::Errors::ErrorCode::Ok)
    {
        return errorCode;
    }

    QJsonObject reducerObject;
    errorCode = m_repository.ReadDocument(RelativeReducerFilePath(), reducerObject);
    if (errorCode != RoboSDP::Errors::ErrorCode::Ok)
    {
        return errorCode;
    }

    QJsonObject driveTrainObject;
    errorCode = m_repository.ReadDocument(RelativeDriveTrainFilePath(), driveTrainObject);
    if (errorCode != RoboSDP::Errors::ErrorCode::Ok)
    {
        return errorCode;
    }

    state = RoboSDP::Selection::Dto::SelectionWorkspaceStateDto::CreateDefault();
    FillMotorResultsFromJsonObject(motorObject, state);
    FillReducerResultsFromJsonObject(reducerObject, state);
    FillDriveTrainFromJsonObject(driveTrainObject, state);
    return RoboSDP::Errors::ErrorCode::Ok;
}

QString SelectionJsonStorage::RelativeMotorFilePath() const
{
    return QStringLiteral("selection/motor-selection.json");
}

QString SelectionJsonStorage::RelativeReducerFilePath() const
{
    return QStringLiteral("selection/reducer-selection.json");
}

QString SelectionJsonStorage::RelativeDriveTrainFilePath() const
{
    return QStringLiteral("selection/drivetrain-selection.json");
}

QString SelectionJsonStorage::BuildAbsoluteMotorFilePath(const QString& projectRootPath) const
{
    return QDir(projectRootPath).absoluteFilePath(RelativeMotorFilePath());
}

QString SelectionJsonStorage::BuildAbsoluteReducerFilePath(const QString& projectRootPath) const
{
    return QDir(projectRootPath).absoluteFilePath(RelativeReducerFilePath());
}

QString SelectionJsonStorage::BuildAbsoluteDriveTrainFilePath(const QString& projectRootPath) const
{
    return QDir(projectRootPath).absoluteFilePath(RelativeDriveTrainFilePath());
}

QJsonObject SelectionJsonStorage::ToMotorJsonObject(
    const RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const
{
    QJsonArray array;
    for (const auto& result : state.motor_results)
    {
        array.push_back(ToMotorResultJson(result));
    }
    return QJsonObject {{QStringLiteral("motor_results"), array}};
}

QJsonObject SelectionJsonStorage::ToReducerJsonObject(
    const RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const
{
    QJsonArray array;
    for (const auto& result : state.reducer_results)
    {
        array.push_back(ToReducerResultJson(result));
    }
    return QJsonObject {{QStringLiteral("reducer_results"), array}};
}

QJsonObject SelectionJsonStorage::ToDriveTrainJsonObject(
    const RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const
{
    return ToDriveTrainJson(state.drive_train_result);
}

void SelectionJsonStorage::FillMotorResultsFromJsonObject(
    const QJsonObject& jsonObject,
    RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const
{
    state.motor_results.clear();
    for (const auto& value : jsonObject.value(QStringLiteral("motor_results")).toArray())
    {
        state.motor_results.push_back(MotorResultFromJson(value.toObject()));
    }
}

void SelectionJsonStorage::FillReducerResultsFromJsonObject(
    const QJsonObject& jsonObject,
    RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const
{
    state.reducer_results.clear();
    for (const auto& value : jsonObject.value(QStringLiteral("reducer_results")).toArray())
    {
        state.reducer_results.push_back(ReducerResultFromJson(value.toObject()));
    }
}

void SelectionJsonStorage::FillDriveTrainFromJsonObject(
    const QJsonObject& jsonObject,
    RoboSDP::Selection::Dto::SelectionWorkspaceStateDto& state) const
{
    state.drive_train_result = DriveTrainFromJson(jsonObject);
}

} // namespace RoboSDP::Selection::Persistence
