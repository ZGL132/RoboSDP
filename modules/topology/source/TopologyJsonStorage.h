#pragma once

#include "core/errors/ErrorCode.h"
#include "core/repository/IJsonRepository.h"
#include "modules/topology/dto/TopologyRecommendationDto.h"

#include <QJsonObject>

namespace RoboSDP::Topology::Persistence
{

/**
 * @brief Topology JSON 持久化组件。
 *
 * 该组件负责在 `topology/topology-model.json` 与 Topology 工作态 DTO 之间
 * 做序列化与反序列化，不承担规则校验和候选推荐逻辑。
 */
class TopologyJsonStorage
{
public:
    explicit TopologyJsonStorage(RoboSDP::Repository::IJsonRepository& repository);

    RoboSDP::Errors::ErrorCode Save(
        const QString& projectRootPath,
        const RoboSDP::Topology::Dto::TopologyWorkspaceStateDto& state) const;

    RoboSDP::Errors::ErrorCode Load(
        const QString& projectRootPath,
        RoboSDP::Topology::Dto::TopologyWorkspaceStateDto& state) const;

    /// 返回 Topology JSON 的相对路径。
    QString RelativeFilePath() const;

    /// 根据项目目录构造 Topology JSON 的绝对路径。
    QString BuildAbsoluteFilePath(const QString& projectRootPath) const;

private:
    QJsonObject ToJsonObject(const RoboSDP::Topology::Dto::TopologyWorkspaceStateDto& state) const;
    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto FromJsonObject(const QJsonObject& jsonObject) const;

    QJsonObject ToModelObject(const RoboSDP::Topology::Dto::RobotTopologyModelDto& model) const;
    RoboSDP::Topology::Dto::RobotTopologyModelDto FromModelObject(const QJsonObject& jsonObject) const;

private:
    RoboSDP::Repository::IJsonRepository& m_repository;
};

} // namespace RoboSDP::Topology::Persistence
