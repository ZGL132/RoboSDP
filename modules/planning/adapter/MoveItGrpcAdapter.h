#pragma once

#include "core/logging/ILogger.h"
#include "modules/planning/dto/PlanningRequestDto.h"
#include "modules/planning/dto/PlanningResponseDto.h"
#include "modules/planning/dto/PlanningSceneDto.h"

namespace RoboSDP::Planning::Adapter
{

/**
 * @brief MoveIt gRPC Adapter 骨架。
 *
 * 本轮保持技术路线为 Python MoveIt 服务 + gRPC，
 * 但 C++ 侧只实现接口骨架与调用链，内部先返回最小 skeleton 响应，
 * 为后续接入真实 gRPC 客户端保留稳定边界。
 */
class MoveItGrpcAdapter
{
public:
    explicit MoveItGrpcAdapter(RoboSDP::Logging::ILogger* logger = nullptr);

    RoboSDP::Planning::Dto::PlanningResponseDto VerifyPointToPoint(
        const RoboSDP::Planning::Dto::PlanningSceneDto& scene,
        const RoboSDP::Planning::Dto::PlanningRequestDto& request) const;

private:
    RoboSDP::Logging::ILogger* m_logger = nullptr;
};

} // namespace RoboSDP::Planning::Adapter
