#pragma once

#include "core/errors/ErrorCode.h"
#include "core/repository/IJsonRepository.h"
#include "modules/kinematics/dto/KinematicSolverResultDto.h"

#include <QJsonObject>

namespace RoboSDP::Kinematics::Persistence
{

/**
 * @brief Kinematics JSON 持久化组件。运动学草稿数据与分析结果的读写管理器。
 * 将内存状态持久化为项目目录下的两个 JSON 文件：
 * 1. kinematics/kinematic-model.json：保存模型参数、坐标系、限位、IK 配置以及最近一次 FK/IK 的结果摘要。
 * 2. kinematics/workspace-cache.json：由于工作空间点云数据量大，单独写入缓存文件。
 * 当前实现把模型与最近一次 FK/IK 摘要写入 `kinematics/kinematic-model.json`，
 * 把基础工作空间采样结果写入 `kinematics/workspace-cache.json`。
 * 支持向前兼容读取：在加载旧版 JSON 时，若缺失“关节签名（joint_order_signature）”或“统一快照”等新版同步字段，会自动执行动态补齐与兼容推断。
 */
class KinematicJsonStorage
{
public:
    explicit KinematicJsonStorage(RoboSDP::Repository::IJsonRepository& repository);

    RoboSDP::Errors::ErrorCode SaveModel(
        const QString& projectRootPath,
        const RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& state) const;

    RoboSDP::Errors::ErrorCode LoadModel(
        const QString& projectRootPath,
        RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& state) const;

    RoboSDP::Errors::ErrorCode SaveWorkspaceCache(
        const QString& projectRootPath,
        const RoboSDP::Kinematics::Dto::WorkspaceResultDto& result) const;

    RoboSDP::Errors::ErrorCode LoadWorkspaceCache(
        const QString& projectRootPath,
        RoboSDP::Kinematics::Dto::WorkspaceResultDto& result) const;

    QString RelativeModelFilePath() const;
    QString RelativeWorkspaceFilePath() const;
    QString BuildAbsoluteModelFilePath(const QString& projectRootPath) const;
    QString BuildAbsoluteWorkspaceFilePath(const QString& projectRootPath) const;

private:
    QJsonObject ToModelJsonObject(const RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& state) const;
    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto FromModelJsonObject(const QJsonObject& jsonObject) const;

    QJsonObject ToWorkspaceJsonObject(const RoboSDP::Kinematics::Dto::WorkspaceResultDto& result) const;
    RoboSDP::Kinematics::Dto::WorkspaceResultDto FromWorkspaceJsonObject(const QJsonObject& jsonObject) const;

private:
    RoboSDP::Repository::IJsonRepository& m_repository;
};

} // namespace RoboSDP::Kinematics::Persistence
