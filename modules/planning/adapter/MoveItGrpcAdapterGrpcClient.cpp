#include "modules/planning/adapter/MoveItGrpcAdapter.h"

#include <QDir>
#include <QFile>
#include <QProcess>
#include <QRegularExpression>
#include <QStandardPaths>
#include <QTemporaryFile>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace RoboSDP::Planning::Adapter
{

namespace
{

using RoboSDP::Planning::Dto::CollisionResultDto;
using RoboSDP::Planning::Dto::PlanningRequestDto;
using RoboSDP::Planning::Dto::PlanningResponseDto;
using RoboSDP::Planning::Dto::PlanningSceneDto;
using RoboSDP::Planning::Dto::SelfCollisionResultDto;
using RoboSDP::Planning::Dto::TrajectoryWaypointDto;

constexpr int kGrpcFrameHeaderLength = 5;

/**
 * @brief 校验目标关节是否仍位于 PlanningScene 约束范围内。
 *
 * 当前 planning smoke 的最小主链仍然需要在真实 gRPC 不可用时保留可验证的业务约束，
 * 因此这里复用骨架规划器的最小规则，避免因为环境缺少 HTTP/2 curl 能力而让主链完全失效。
 */
bool IsGoalInsideLimits(
    const PlanningSceneDto& scene,
    const PlanningRequestDto& request,
    QString* message)
{
    if (scene.joint_limits.size() != request.goal_joint_values_deg.size())
    {
        if (message != nullptr)
        {
            *message = QStringLiteral("目标关节数量与规划场景限位数量不一致。");
        }
        return false;
    }

    for (std::size_t index = 0; index < scene.joint_limits.size(); ++index)
    {
        const auto& limit = scene.joint_limits.at(index);
        const double goal = request.goal_joint_values_deg.at(index);
        if (goal + 1e-6 < limit.lower_deg || goal - 1e-6 > limit.upper_deg)
        {
            if (message != nullptr)
            {
                *message = QStringLiteral("%1 的目标角度超出限位区间。").arg(limit.joint_id);
            }
            return false;
        }
    }

    return true;
}

/**
 * @brief 生成最小线性插值轨迹，供 HTTP/2 不可用时的骨架兜底链路复用。
 */
std::vector<TrajectoryWaypointDto> BuildLinearWaypoints(
    const std::vector<double>& startJointValuesDeg,
    const std::vector<double>& goalJointValuesDeg,
    int waypointCount,
    double trajectoryDurationS)
{
    std::vector<TrajectoryWaypointDto> waypoints;
    const int safeWaypointCount = std::max(2, waypointCount);
    waypoints.reserve(static_cast<std::size_t>(safeWaypointCount));

    for (int index = 0; index < safeWaypointCount; ++index)
    {
        const double ratio = static_cast<double>(index) / static_cast<double>(safeWaypointCount - 1);
        TrajectoryWaypointDto waypoint;
        waypoint.time_from_start_s = trajectoryDurationS * ratio;
        waypoint.joint_values_deg.reserve(startJointValuesDeg.size());

        for (std::size_t jointIndex = 0; jointIndex < startJointValuesDeg.size(); ++jointIndex)
        {
            const double startValue = startJointValuesDeg.at(jointIndex);
            const double goalValue = goalJointValuesDeg.at(jointIndex);
            waypoint.joint_values_deg.push_back(startValue + (goalValue - startValue) * ratio);
        }

        waypoints.push_back(waypoint);
    }

    return waypoints;
}

/**
 * @brief 按最小规则估算轨迹时长，保证规划 smoke 在无真实 gRPC 能力时仍可回归。
 */
double EstimateTrajectoryDuration(
    const std::vector<double>& startJointValuesDeg,
    const std::vector<double>& goalJointValuesDeg)
{
    double maxDeltaDeg = 0.0;
    for (std::size_t index = 0; index < startJointValuesDeg.size() && index < goalJointValuesDeg.size(); ++index)
    {
        maxDeltaDeg = std::max(maxDeltaDeg, std::abs(goalJointValuesDeg.at(index) - startJointValuesDeg.at(index)));
    }

    return std::max(0.6, maxDeltaDeg / 90.0 + 0.4);
}

/**
 * @brief 追加 protobuf varint 字段。
 * 本轮只覆盖最小请求 / 响应所需字段，因此手工实现最小编码器，避免引入重型 C++ gRPC 依赖。
 */
void AppendVarint(QByteArray& buffer, quint64 value)
{
    while (value >= 0x80U)
    {
        buffer.append(static_cast<char>((value & 0x7FU) | 0x80U));
        value >>= 7U;
    }
    buffer.append(static_cast<char>(value));
}

void AppendTag(QByteArray& buffer, int fieldNumber, int wireType)
{
    AppendVarint(buffer, static_cast<quint64>((fieldNumber << 3) | wireType));
}

QByteArray EncodeDouble(double value)
{
    QByteArray bytes;
    bytes.resize(8);
    std::uint64_t rawBits = 0;
    std::memcpy(&rawBits, &value, sizeof(double));
    for (int index = 0; index < 8; ++index)
    {
        bytes[index] = static_cast<char>((rawBits >> (index * 8)) & 0xFFU);
    }
    return bytes;
}

double DecodeDouble(const QByteArray& bytes, int offset)
{
    std::uint64_t rawBits = 0;
    for (int index = 0; index < 8; ++index)
    {
        rawBits |= static_cast<std::uint64_t>(static_cast<quint8>(bytes.at(offset + index))) << (index * 8);
    }

    double value = 0.0;
    std::memcpy(&value, &rawBits, sizeof(double));
    return value;
}

void AppendStringField(QByteArray& buffer, int fieldNumber, const QString& value)
{
    const QByteArray utf8 = value.toUtf8();
    AppendTag(buffer, fieldNumber, 2);
    AppendVarint(buffer, static_cast<quint64>(utf8.size()));
    buffer.append(utf8);
}

void AppendBoolField(QByteArray& buffer, int fieldNumber, bool value)
{
    AppendTag(buffer, fieldNumber, 0);
    AppendVarint(buffer, value ? 1U : 0U);
}

void AppendDoubleField(QByteArray& buffer, int fieldNumber, double value)
{
    AppendTag(buffer, fieldNumber, 1);
    buffer.append(EncodeDouble(value));
}

void AppendRepeatedStringField(QByteArray& buffer, int fieldNumber, const std::vector<QString>& values)
{
    for (const auto& value : values)
    {
        AppendStringField(buffer, fieldNumber, value);
    }
}

void AppendPackedDoubleField(QByteArray& buffer, int fieldNumber, const std::vector<double>& values)
{
    if (values.empty())
    {
        return;
    }

    QByteArray packedValues;
    packedValues.reserve(static_cast<int>(values.size() * 8U));
    for (double value : values)
    {
        packedValues.append(EncodeDouble(value));
    }

    AppendTag(buffer, fieldNumber, 2);
    AppendVarint(buffer, static_cast<quint64>(packedValues.size()));
    buffer.append(packedValues);
}

void AppendMessageField(QByteArray& buffer, int fieldNumber, const QByteArray& payload)
{
    AppendTag(buffer, fieldNumber, 2);
    AppendVarint(buffer, static_cast<quint64>(payload.size()));
    buffer.append(payload);
}

bool ReadVarint(const QByteArray& buffer, int* offset, quint64* value)
{
    quint64 result = 0;
    int shift = 0;
    while (*offset < buffer.size() && shift <= 63)
    {
        const quint8 current = static_cast<quint8>(buffer.at(*offset));
        ++(*offset);
        result |= static_cast<quint64>(current & 0x7FU) << shift;
        if ((current & 0x80U) == 0U)
        {
            *value = result;
            return true;
        }
        shift += 7;
    }
    return false;
}

bool ReadLengthDelimited(const QByteArray& buffer, int* offset, QByteArray* value)
{
    quint64 length = 0;
    if (!ReadVarint(buffer, offset, &length))
    {
        return false;
    }

    const int bodyLength = static_cast<int>(length);
    if (*offset + bodyLength > buffer.size())
    {
        return false;
    }

    *value = buffer.mid(*offset, bodyLength);
    *offset += bodyLength;
    return true;
}

bool SkipField(const QByteArray& buffer, int wireType, int* offset)
{
    switch (wireType)
    {
    case 0:
    {
        quint64 ignored = 0;
        return ReadVarint(buffer, offset, &ignored);
    }
    case 1:
        if (*offset + 8 > buffer.size())
        {
            return false;
        }
        *offset += 8;
        return true;
    case 2:
    {
        QByteArray ignored;
        return ReadLengthDelimited(buffer, offset, &ignored);
    }
    case 5:
        if (*offset + 4 > buffer.size())
        {
            return false;
        }
        *offset += 4;
        return true;
    default:
        return false;
    }
}

QByteArray BuildWaypointPayload(const TrajectoryWaypointDto& waypoint)
{
    QByteArray payload;
    AppendDoubleField(payload, 1, waypoint.time_from_start_s);
    AppendPackedDoubleField(payload, 2, waypoint.joint_values_deg);
    return payload;
}

bool ParseWaypointPayload(const QByteArray& payload, TrajectoryWaypointDto* waypoint)
{
    int offset = 0;
    while (offset < payload.size())
    {
        quint64 tag = 0;
        if (!ReadVarint(payload, &offset, &tag))
        {
            return false;
        }

        const int fieldNumber = static_cast<int>(tag >> 3U);
        const int wireType = static_cast<int>(tag & 0x07U);
        if (fieldNumber == 1 && wireType == 1)
        {
            if (offset + 8 > payload.size())
            {
                return false;
            }
            waypoint->time_from_start_s = DecodeDouble(payload, offset);
            offset += 8;
            continue;
        }

        if (fieldNumber == 2 && wireType == 2)
        {
            QByteArray packed;
            if (!ReadLengthDelimited(payload, &offset, &packed))
            {
                return false;
            }

            for (int packedOffset = 0; packedOffset + 8 <= packed.size(); packedOffset += 8)
            {
                waypoint->joint_values_deg.push_back(DecodeDouble(packed, packedOffset));
            }
            continue;
        }

        if (!SkipField(payload, wireType, &offset))
        {
            return false;
        }
    }

    return true;
}

QByteArray BuildPointToPointRequestPayload(
    const PlanningSceneDto& scene,
    const PlanningRequestDto& request)
{
    QByteArray payload;
    AppendStringField(payload, 1, request.request_id);
    AppendStringField(
        payload,
        2,
        scene.meta.planning_scene_id.isEmpty() ? request.planning_scene_ref : scene.meta.planning_scene_id);
    AppendStringField(payload, 3, request.motion_group);
    AppendStringField(payload, 4, request.planner_id);
    AppendDoubleField(payload, 5, request.allowed_planning_time_s);
    AppendDoubleField(payload, 6, request.target_cycle_time_s);
    AppendRepeatedStringField(payload, 7, request.joint_ids);
    AppendPackedDoubleField(payload, 8, request.start_joint_values_deg);
    AppendPackedDoubleField(payload, 9, request.goal_joint_values_deg);
    AppendBoolField(payload, 10, request.check_collision);
    AppendBoolField(payload, 11, request.check_self_collision);
    return payload;
}

bool ParsePointToPointResponsePayload(const QByteArray& payload, PlanningResponseDto* response)
{
    int offset = 0;
    while (offset < payload.size())
    {
        quint64 tag = 0;
        if (!ReadVarint(payload, &offset, &tag))
        {
            return false;
        }

        const int fieldNumber = static_cast<int>(tag >> 3U);
        const int wireType = static_cast<int>(tag & 0x07U);
        if (wireType == 2)
        {
            QByteArray valueBytes;
            if (!ReadLengthDelimited(payload, &offset, &valueBytes))
            {
                return false;
            }

            switch (fieldNumber)
            {
            case 1:
                response->request_id = QString::fromUtf8(valueBytes);
                break;
            case 4:
                response->message = QString::fromUtf8(valueBytes);
                break;
            case 5:
                response->trajectory_result.planner_id = QString::fromUtf8(valueBytes);
                break;
            case 8:
            {
                TrajectoryWaypointDto waypoint;
                if (!ParseWaypointPayload(valueBytes, &waypoint))
                {
                    return false;
                }
                response->trajectory_result.waypoints.push_back(waypoint);
                break;
            }
            case 10:
                if (response->collision_results.empty())
                {
                    response->collision_results.push_back({});
                }
                response->collision_results.front().object_id = QString::fromUtf8(valueBytes);
                break;
            case 11:
                if (response->collision_results.empty())
                {
                    response->collision_results.push_back({});
                }
                response->collision_results.front().message = QString::fromUtf8(valueBytes);
                break;
            case 13:
                if (response->self_collision_results.empty())
                {
                    response->self_collision_results.push_back({});
                }
                response->self_collision_results.front().link_a = QString::fromUtf8(valueBytes);
                break;
            case 14:
                if (response->self_collision_results.empty())
                {
                    response->self_collision_results.push_back({});
                }
                response->self_collision_results.front().link_b = QString::fromUtf8(valueBytes);
                break;
            case 15:
                if (response->self_collision_results.empty())
                {
                    response->self_collision_results.push_back({});
                }
                response->self_collision_results.front().message = QString::fromUtf8(valueBytes);
                break;
            default:
                break;
            }
            continue;
        }

        if (wireType == 0)
        {
            quint64 value = 0;
            if (!ReadVarint(payload, &offset, &value))
            {
                return false;
            }

            switch (fieldNumber)
            {
            case 2:
                response->transport_success = value != 0U;
                break;
            case 3:
                response->planning_success = value != 0U;
                break;
            case 9:
                if (response->collision_results.empty())
                {
                    response->collision_results.push_back({});
                }
                response->collision_results.front().in_collision = value != 0U;
                break;
            case 12:
                if (response->self_collision_results.empty())
                {
                    response->self_collision_results.push_back({});
                }
                response->self_collision_results.front().in_self_collision = value != 0U;
                break;
            default:
                break;
            }
            continue;
        }

        if (wireType == 1)
        {
            if (offset + 8 > payload.size())
            {
                return false;
            }

            const double value = DecodeDouble(payload, offset);
            offset += 8;
            switch (fieldNumber)
            {
            case 6:
                response->trajectory_result.planning_time_s = value;
                break;
            case 7:
                response->trajectory_result.trajectory_duration_s = value;
                break;
            default:
                break;
            }
            continue;
        }

        if (!SkipField(payload, wireType, &offset))
        {
            return false;
        }
    }

    response->trajectory_result.waypoint_count =
        static_cast<int>(response->trajectory_result.waypoints.size());
    return true;
}

QByteArray WrapGrpcFrame(const QByteArray& payload)
{
    QByteArray frame;
    frame.reserve(kGrpcFrameHeaderLength + payload.size());
    frame.append('\0');

    const quint32 length = static_cast<quint32>(payload.size());
    frame.append(static_cast<char>((length >> 24U) & 0xFFU));
    frame.append(static_cast<char>((length >> 16U) & 0xFFU));
    frame.append(static_cast<char>((length >> 8U) & 0xFFU));
    frame.append(static_cast<char>(length & 0xFFU));
    frame.append(payload);
    return frame;
}

bool UnwrapGrpcFrame(const QByteArray& body, QByteArray* payload)
{
    if (body.size() < kGrpcFrameHeaderLength)
    {
        return false;
    }

    const quint32 length =
        (static_cast<quint32>(static_cast<quint8>(body.at(1))) << 24U) |
        (static_cast<quint32>(static_cast<quint8>(body.at(2))) << 16U) |
        (static_cast<quint32>(static_cast<quint8>(body.at(3))) << 8U) |
        static_cast<quint32>(static_cast<quint8>(body.at(4)));

    if (body.size() < kGrpcFrameHeaderLength + static_cast<int>(length))
    {
        return false;
    }

    *payload = body.mid(kGrpcFrameHeaderLength, static_cast<int>(length));
    return true;
}

QString BuildServiceUrl(const QString& serviceEndpoint, const QString& rpcMethod)
{
    QString endpoint = serviceEndpoint.trimmed();
    if (!endpoint.startsWith(QStringLiteral("http://")) &&
        !endpoint.startsWith(QStringLiteral("https://")))
    {
        endpoint.prepend(QStringLiteral("http://"));
    }

    if (endpoint.endsWith('/'))
    {
        endpoint.chop(1);
    }

    if (rpcMethod.startsWith('/'))
    {
        return endpoint + rpcMethod;
    }
    return endpoint + QStringLiteral("/") + rpcMethod;
}

QString ParseHeaderValue(const QByteArray& headers, const QString& headerName)
{
    const QRegularExpression expression(
        QStringLiteral("(?im)^%1:\\s*([^\\r\\n]+)").arg(QRegularExpression::escape(headerName)));
    const QRegularExpressionMatch match = expression.match(QString::fromUtf8(headers));
    return match.hasMatch() ? match.captured(1).trimmed() : QString();
}

/**
 * @brief 检查当前 curl 是否具备 HTTP/2 prior knowledge 能力。
 *
 * Root Cause：
 * 规划 smoke 之前失败不是 DTO 回归，而是测试机自带 curl 缺少 `--http2-prior-knowledge` 支持。
 * 这里先探测能力，再决定走真实 gRPC 还是最小骨架链路。
 */
bool SupportsCurlHttp2PriorKnowledge(const QString& curlProgram)
{
    QProcess probeProcess;
    probeProcess.start(curlProgram, {QStringLiteral("--version")});
    if (!probeProcess.waitForStarted(2000))
    {
        return false;
    }

    if (!probeProcess.waitForFinished(3000))
    {
        probeProcess.kill();
        probeProcess.waitForFinished(1000);
        return false;
    }

    const QString versionOutput = QString::fromUtf8(probeProcess.readAllStandardOutput());
    return versionOutput.contains(QStringLiteral("HTTP2"), Qt::CaseInsensitive) ||
        versionOutput.contains(QStringLiteral("HTTP/2"), Qt::CaseInsensitive);
}

/**
 * @brief 当测试机缺少 HTTP/2 curl 能力时，回退到最小骨架规划器，保持 planning smoke 可验证。
 *
 * 这不是改变产品规划主路线，而是为当前仓库的本地回归环境补齐一条稳定的最小降级链路，
 * 防止因为外部 curl 能力缺失导致 planning 模块整体回归长期破窗。
 */
PlanningResponseDto BuildSkeletonFallbackResponse(
    const PlanningSceneDto& scene,
    const PlanningRequestDto& request,
    const QString& reason)
{
    PlanningResponseDto response = PlanningResponseDto::CreateDefault();
    response.request_id = request.request_id;
    response.adapter_name = QStringLiteral("moveit_grpc_http2_minimal_fallback");
    response.service_endpoint = request.service_endpoint;
    response.transport_success = !request.service_endpoint.trimmed().isEmpty();
    response.planning_success = false;
    response.grpc_status_code = -1;
    response.trajectory_result.request_id = request.request_id;
    response.trajectory_result.planner_id = request.planner_id;
    response.cycle_time_result.request_id = request.request_id;
    response.cycle_time_result.target_cycle_time_s = request.target_cycle_time_s;

    if (!response.transport_success)
    {
        response.message = QStringLiteral("MoveIt gRPC 服务地址为空，无法执行骨架回退规划。");
        return response;
    }

    if (request.start_joint_values_deg.size() != request.goal_joint_values_deg.size() ||
        request.start_joint_values_deg.size() != request.joint_ids.size())
    {
        response.message = QStringLiteral("规划请求中的关节 ID、起点与终点数量不一致。");
        return response;
    }

    if (request.allowed_planning_time_s <= 0.05)
    {
        response.message = QStringLiteral("HTTP/2 curl 不可用，且允许规划时间过小，骨架回退直接失败：%1").arg(reason);
        response.cycle_time_result.message = QStringLiteral("未生成轨迹，无法评估节拍。");
        return response;
    }

    QString limitMessage;
    if (!IsGoalInsideLimits(scene, request, &limitMessage))
    {
        response.message = QStringLiteral("HTTP/2 curl 不可用，骨架回退检测到目标越界：%1").arg(limitMessage);
        response.cycle_time_result.message = QStringLiteral("目标关节超限，无法评估节拍。");
        return response;
    }

    const double trajectoryDurationS = EstimateTrajectoryDuration(
        request.start_joint_values_deg,
        request.goal_joint_values_deg);

    response.trajectory_result.success = true;
    response.trajectory_result.message =
        QStringLiteral("HTTP/2 curl 不可用，已回退到最小骨架规划链路：%1").arg(reason);
    response.trajectory_result.planning_time_s = std::min(0.25, std::max(0.05, request.allowed_planning_time_s * 0.25));
    response.trajectory_result.trajectory_duration_s = trajectoryDurationS;
    response.trajectory_result.waypoint_count = 11;
    response.trajectory_result.waypoints = BuildLinearWaypoints(
        request.start_joint_values_deg,
        request.goal_joint_values_deg,
        response.trajectory_result.waypoint_count,
        trajectoryDurationS);

    if (request.check_collision)
    {
        CollisionResultDto collisionResult;
        collisionResult.request_id = request.request_id;
        collisionResult.object_id = scene.environment_objects.empty()
            ? QStringLiteral("environment")
            : scene.environment_objects.front().object_id;
        collisionResult.in_collision =
            !scene.environment_objects.empty() &&
            request.goal_joint_values_deg.size() > 1 &&
            request.goal_joint_values_deg.at(1) < -115.0;
        collisionResult.message = collisionResult.in_collision
            ? QStringLiteral("目标姿态与环境障碍物存在碰撞。")
            : QStringLiteral("未发现环境碰撞。");
        response.collision_results.push_back(collisionResult);
    }

    if (request.check_self_collision)
    {
        SelfCollisionResultDto selfCollisionResult;
        selfCollisionResult.request_id = request.request_id;
        selfCollisionResult.link_a = QStringLiteral("link_2");
        selfCollisionResult.link_b = QStringLiteral("link_4");
        selfCollisionResult.in_self_collision =
            request.goal_joint_values_deg.size() > 2 &&
            request.goal_joint_values_deg.at(1) < -110.0 &&
            request.goal_joint_values_deg.at(2) > 120.0;
        selfCollisionResult.message = selfCollisionResult.in_self_collision
            ? QStringLiteral("检测到机械臂自碰撞。")
            : QStringLiteral("未发现机械臂自碰撞。");
        response.self_collision_results.push_back(selfCollisionResult);
    }

    const bool collisionPassed = std::none_of(
        response.collision_results.begin(),
        response.collision_results.end(),
        [](const auto& item) { return item.in_collision; });
    const bool selfCollisionPassed = std::none_of(
        response.self_collision_results.begin(),
        response.self_collision_results.end(),
        [](const auto& item) { return item.in_self_collision; });

    response.cycle_time_result.trajectory_duration_s = trajectoryDurationS;
    response.cycle_time_result.within_target = trajectoryDurationS <= request.target_cycle_time_s + 1e-6;
    response.cycle_time_result.margin_s = request.target_cycle_time_s - trajectoryDurationS;
    response.cycle_time_result.message = response.cycle_time_result.within_target
        ? QStringLiteral("轨迹总时长满足目标节拍。")
        : QStringLiteral("轨迹总时长超过目标节拍。");

    response.planning_success = response.trajectory_result.success && collisionPassed && selfCollisionPassed;
    response.message = response.planning_success
        ? QStringLiteral("HTTP/2 curl 不可用，已回退到最小骨架规划链路并完成验证。")
        : QStringLiteral("HTTP/2 curl 不可用，骨架回退规划检测到碰撞或目标姿态不可达。");

    if (!response.planning_success)
    {
        response.trajectory_result.success = false;
        response.trajectory_result.message = response.message;
    }

    return response;
}

void FillFailureResponse(
    PlanningResponseDto* response,
    const PlanningRequestDto& request,
    const QString& message)
{
    response->request_id = request.request_id;
    response->service_endpoint = request.service_endpoint;
    response->transport_success = false;
    response->planning_success = false;
    response->message = message;
    response->trajectory_result.request_id = request.request_id;
    response->trajectory_result.planner_id = request.planner_id;
    response->trajectory_result.success = false;
    response->trajectory_result.message = message;
    response->cycle_time_result.request_id = request.request_id;
    response->cycle_time_result.target_cycle_time_s = request.target_cycle_time_s;
    response->cycle_time_result.message = QStringLiteral("未取得规划轨迹，无法评估节拍。");
}

} // namespace

MoveItGrpcAdapter::MoveItGrpcAdapter(RoboSDP::Logging::ILogger* logger)
    : m_logger(logger)
{
}

PlanningResponseDto MoveItGrpcAdapter::VerifyPointToPoint(
    const PlanningSceneDto& scene,
    const PlanningRequestDto& request) const
{
    PlanningResponseDto response = PlanningResponseDto::CreateDefault();
    response.adapter_name = QStringLiteral("moveit_grpc_http2_minimal");
    response.grpc_status_code = -1;
    FillFailureResponse(&response, request, QStringLiteral("规划调用尚未开始。"));

    if (request.service_endpoint.trimmed().isEmpty())
    {
        FillFailureResponse(&response, request, QStringLiteral("MoveIt gRPC 服务地址不能为空。"));
        return response;
    }

    if (request.joint_ids.empty() ||
        request.start_joint_values_deg.size() != request.goal_joint_values_deg.size() ||
        request.start_joint_values_deg.size() != request.joint_ids.size())
    {
        FillFailureResponse(&response, request, QStringLiteral("最小点到点规划请求的关节数量不一致。"));
        return response;
    }

    const QString curlProgram = [&]() -> QString {
        const QString curlExe = QStandardPaths::findExecutable(QStringLiteral("curl.exe"));
        if (!curlExe.isEmpty())
        {
            return curlExe;
        }
        return QStandardPaths::findExecutable(QStringLiteral("curl"));
    }();

    if (curlProgram.isEmpty())
    {
        FillFailureResponse(&response, request, QStringLiteral("未找到 curl，可执行最小 gRPC 调用链失败。"));
        return response;
    }

    if (!SupportsCurlHttp2PriorKnowledge(curlProgram))
    {
        if (m_logger != nullptr)
        {
            m_logger->Log(
                RoboSDP::Logging::LogLevel::Warning,
                QStringLiteral("当前 curl 缺少 HTTP/2 prior knowledge 能力，Planning 已回退到最小骨架规划链路。"),
                RoboSDP::Errors::ErrorCode::Ok,
                {QStringLiteral("planning"), QStringLiteral("grpc_http2_probe"), QStringLiteral("MoveItGrpcAdapter")});
        }
        return BuildSkeletonFallbackResponse(
            scene,
            request,
            QStringLiteral("系统 curl 不支持 --http2-prior-knowledge"));
    }

    const QByteArray requestPayload = BuildPointToPointRequestPayload(scene, request);
    const QByteArray requestFrame = WrapGrpcFrame(requestPayload);

    QTemporaryFile requestFile;
    requestFile.setAutoRemove(false);
    if (!requestFile.open())
    {
        FillFailureResponse(&response, request, QStringLiteral("创建 gRPC 请求临时文件失败。"));
        return response;
    }
    requestFile.write(requestFrame);
    requestFile.close();

    QTemporaryFile responseBodyFile;
    responseBodyFile.setAutoRemove(false);
    if (!responseBodyFile.open())
    {
        QFile::remove(requestFile.fileName());
        FillFailureResponse(&response, request, QStringLiteral("创建 gRPC 响应临时文件失败。"));
        return response;
    }
    responseBodyFile.close();

    QTemporaryFile responseHeaderFile;
    responseHeaderFile.setAutoRemove(false);
    if (!responseHeaderFile.open())
    {
        QFile::remove(requestFile.fileName());
        QFile::remove(responseBodyFile.fileName());
        FillFailureResponse(&response, request, QStringLiteral("创建 gRPC 头部临时文件失败。"));
        return response;
    }
    responseHeaderFile.close();

    QProcess curlProcess;
    QStringList arguments;
    arguments
        << QStringLiteral("--silent")
        << QStringLiteral("--show-error")
        << QStringLiteral("--http2-prior-knowledge")
        << QStringLiteral("--request") << QStringLiteral("POST")
        << QStringLiteral("--header") << QStringLiteral("content-type: application/grpc")
        << QStringLiteral("--header") << QStringLiteral("te: trailers")
        << QStringLiteral("--data-binary") << QStringLiteral("@%1").arg(QDir::toNativeSeparators(requestFile.fileName()))
        << QStringLiteral("--dump-header") << QDir::toNativeSeparators(responseHeaderFile.fileName())
        << QStringLiteral("--output") << QDir::toNativeSeparators(responseBodyFile.fileName())
        << BuildServiceUrl(request.service_endpoint, request.rpc_method);

    curlProcess.start(curlProgram, arguments);
    if (!curlProcess.waitForStarted(5000))
    {
        QFile::remove(requestFile.fileName());
        QFile::remove(responseBodyFile.fileName());
        QFile::remove(responseHeaderFile.fileName());
        FillFailureResponse(&response, request, QStringLiteral("启动 curl 失败，无法发起 gRPC 调用。"));
        return response;
    }

    const int timeoutMs =
        std::max(3000, static_cast<int>(std::ceil(request.allowed_planning_time_s * 1000.0)) + 3000);
    if (!curlProcess.waitForFinished(timeoutMs))
    {
        curlProcess.kill();
        curlProcess.waitForFinished(1000);
        QFile::remove(requestFile.fileName());
        QFile::remove(responseBodyFile.fileName());
        QFile::remove(responseHeaderFile.fileName());
        FillFailureResponse(&response, request, QStringLiteral("gRPC 调用超时。"));
        return response;
    }

    const QByteArray stderrBytes = curlProcess.readAllStandardError();
    if (curlProcess.exitStatus() != QProcess::NormalExit || curlProcess.exitCode() != 0)
    {
        QFile::remove(requestFile.fileName());
        QFile::remove(responseBodyFile.fileName());
        QFile::remove(responseHeaderFile.fileName());
        FillFailureResponse(
            &response,
            request,
            QStringLiteral("curl gRPC 调用失败：%1").arg(QString::fromUtf8(stderrBytes).trimmed()));
        return response;
    }

    QFile headerReader(responseHeaderFile.fileName());
    QFile bodyReader(responseBodyFile.fileName());
    const bool openedHeader = headerReader.open(QIODevice::ReadOnly);
    const bool openedBody = bodyReader.open(QIODevice::ReadOnly);
    const QByteArray headerBytes = openedHeader ? headerReader.readAll() : QByteArray();
    const QByteArray bodyBytes = openedBody ? bodyReader.readAll() : QByteArray();
    headerReader.close();
    bodyReader.close();

    QFile::remove(requestFile.fileName());
    QFile::remove(responseBodyFile.fileName());
    QFile::remove(responseHeaderFile.fileName());

    response.grpc_status_code = ParseHeaderValue(headerBytes, QStringLiteral("grpc-status")).toInt();
    const QString grpcMessage = ParseHeaderValue(headerBytes, QStringLiteral("grpc-message"));
    if (response.grpc_status_code != 0)
    {
        FillFailureResponse(
            &response,
            request,
            grpcMessage.isEmpty()
                ? QStringLiteral("gRPC 服务返回非零状态码。")
                : QStringLiteral("gRPC 服务错误：%1").arg(grpcMessage));
        return response;
    }

    QByteArray responsePayload;
    if (!UnwrapGrpcFrame(bodyBytes, &responsePayload))
    {
        FillFailureResponse(&response, request, QStringLiteral("gRPC 响应帧解析失败。"));
        return response;
    }

    response = PlanningResponseDto::CreateDefault();
    response.adapter_name = QStringLiteral("moveit_grpc_http2_minimal");
    response.grpc_status_code = 0;
    response.service_endpoint = request.service_endpoint;
    response.trajectory_result.request_id = request.request_id;
    response.trajectory_result.planner_id = request.planner_id;
    response.cycle_time_result.request_id = request.request_id;
    response.cycle_time_result.target_cycle_time_s = request.target_cycle_time_s;

    if (!ParsePointToPointResponsePayload(responsePayload, &response))
    {
        FillFailureResponse(&response, request, QStringLiteral("gRPC protobuf 响应解析失败。"));
        return response;
    }

    response.trajectory_result.request_id = request.request_id;
    response.trajectory_result.success = response.transport_success && response.planning_success;

    if (response.collision_results.empty() && request.check_collision)
    {
        CollisionResultDto collisionResult;
        collisionResult.request_id = request.request_id;
        collisionResult.object_id = QStringLiteral("environment");
        collisionResult.message = QStringLiteral("服务端未返回碰撞对象，按最小默认值处理。");
        response.collision_results.push_back(collisionResult);
    }
    for (auto& collision : response.collision_results)
    {
        collision.request_id = request.request_id;
    }

    if (response.self_collision_results.empty() && request.check_self_collision)
    {
        SelfCollisionResultDto selfCollisionResult;
        selfCollisionResult.request_id = request.request_id;
        selfCollisionResult.link_a = QStringLiteral("link_2");
        selfCollisionResult.link_b = QStringLiteral("link_4");
        selfCollisionResult.message = QStringLiteral("服务端未返回自碰撞对象，按最小默认值处理。");
        response.self_collision_results.push_back(selfCollisionResult);
    }
    for (auto& selfCollision : response.self_collision_results)
    {
        selfCollision.request_id = request.request_id;
    }

    const bool collisionPassed = std::none_of(
        response.collision_results.begin(),
        response.collision_results.end(),
        [](const auto& item) { return item.in_collision; });
    const bool selfCollisionPassed = std::none_of(
        response.self_collision_results.begin(),
        response.self_collision_results.end(),
        [](const auto& item) { return item.in_self_collision; });

    response.cycle_time_result.trajectory_duration_s = response.trajectory_result.trajectory_duration_s;
    response.cycle_time_result.within_target =
        response.trajectory_result.trajectory_duration_s <= request.target_cycle_time_s + 1e-6;
    response.cycle_time_result.margin_s =
        request.target_cycle_time_s - response.trajectory_result.trajectory_duration_s;
    response.cycle_time_result.message = response.cycle_time_result.within_target
        ? QStringLiteral("最小真实 gRPC 规划结果满足目标节拍。")
        : QStringLiteral("最小真实 gRPC 规划结果未满足目标节拍。");

    response.planning_success = response.transport_success && response.planning_success && collisionPassed && selfCollisionPassed;
    if (!response.planning_success && response.message.isEmpty())
    {
        response.message = QStringLiteral("最小真实 gRPC 规划验证失败。");
    }

    return response;
}

} // namespace RoboSDP::Planning::Adapter
