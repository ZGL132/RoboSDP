#include "core/repository/ProjectService.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QList>

namespace RoboSDP::Repository
{

namespace
{

QList<QString> BuildProjectDirectories()
{
    return {
        QStringLiteral("requirements"),
        QStringLiteral("topology"),
        QStringLiteral("kinematics"),
        QStringLiteral("dynamics"),
        QStringLiteral("selection"),
        QStringLiteral("planning"),
        QStringLiteral("snapshots"),
        QStringLiteral("exports")};
}

QJsonObject BuildFileIndex()
{
    QJsonObject files;
    files.insert(QStringLiteral("requirement_model"), QStringLiteral("requirements/requirement-model.json"));
    files.insert(QStringLiteral("topology_model"), QStringLiteral("topology/topology-model.json"));
    files.insert(QStringLiteral("kinematic_model"), QStringLiteral("kinematics/kinematic-model.json"));
    files.insert(QStringLiteral("dynamic_model"), QStringLiteral("dynamics/dynamic-model.json"));
    files.insert(QStringLiteral("motor_selection"), QStringLiteral("selection/motor-selection.json"));
    files.insert(QStringLiteral("reducer_selection"), QStringLiteral("selection/reducer-selection.json"));
    files.insert(QStringLiteral("drivetrain_selection"), QStringLiteral("selection/drivetrain-selection.json"));
    files.insert(QStringLiteral("planning_scene"), QStringLiteral("planning/planning-scene.json"));
    files.insert(QStringLiteral("scheme_snapshot"), QStringLiteral("snapshots/scheme-snapshot.json"));
    files.insert(QStringLiteral("scheme_export"), QStringLiteral("exports/scheme-export.json"));
    return files;
}

} // namespace

ProjectCreateResult ProjectService::CreateProject(const QString& projectRootPath) const
{
    ProjectCreateResult result;

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空。");
        return result;
    }

    const QString absoluteRootPath = QDir(projectRootPath).absolutePath();
    QDir rootDir(absoluteRootPath);
    if (!rootDir.exists() && !QDir().mkpath(absoluteRootPath))
    {
        result.error_code = RoboSDP::Errors::ErrorCode::RepositoryRootNotFound;
        result.message = QStringLiteral("无法创建项目目录：%1").arg(absoluteRootPath);
        return result;
    }

    const QFileInfo projectFileInfo(rootDir.filePath(QStringLiteral("project.json")));
    if (projectFileInfo.exists())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.project_root_path = absoluteRootPath;
        result.project_name = QFileInfo(absoluteRootPath).fileName();
        result.message = QStringLiteral("目录中已存在 project.json，本轮新建项目不会覆盖已有项目。");
        return result;
    }

    for (const QString& relativeDirectory : BuildProjectDirectories())
    {
        if (!rootDir.mkpath(relativeDirectory))
        {
            result.error_code = RoboSDP::Errors::ErrorCode::RepositoryWriteFailed;
            result.project_root_path = absoluteRootPath;
            result.message = QStringLiteral("创建项目子目录失败：%1").arg(relativeDirectory);
            return result;
        }
    }

    const QString projectName = QFileInfo(absoluteRootPath).fileName().trimmed().isEmpty()
        ? QStringLiteral("未命名项目")
        : QFileInfo(absoluteRootPath).fileName();
    const QString now = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);

    QJsonObject projectDocument;
    projectDocument.insert(QStringLiteral("schema_version"), QStringLiteral("1.0"));
    projectDocument.insert(QStringLiteral("project_name"), projectName);
    projectDocument.insert(QStringLiteral("created_at"), now);
    projectDocument.insert(QStringLiteral("updated_at"), now);
    projectDocument.insert(QStringLiteral("current_scheme_id"), QStringLiteral("default"));
    projectDocument.insert(QStringLiteral("files"), BuildFileIndex());

    QFile projectFile(projectFileInfo.absoluteFilePath());
    if (!projectFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    {
        result.error_code = RoboSDP::Errors::ErrorCode::RepositoryWriteFailed;
        result.project_root_path = absoluteRootPath;
        result.message = QStringLiteral("无法写入 project.json。");
        return result;
    }

    const QJsonDocument jsonDocument(projectDocument);
    const qint64 writtenBytes = projectFile.write(jsonDocument.toJson(QJsonDocument::Indented));
    projectFile.close();
    if (writtenBytes <= 0)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::RepositoryWriteFailed;
        result.project_root_path = absoluteRootPath;
        result.message = QStringLiteral("project.json 写入为空，请检查目录权限。");
        return result;
    }

    result.success = true;
    result.error_code = RoboSDP::Errors::ErrorCode::Ok;
    result.project_root_path = absoluteRootPath;
    result.project_name = projectName;
    result.message = QStringLiteral("项目骨架创建完成：%1").arg(absoluteRootPath);
    return result;
}

} // namespace RoboSDP::Repository
