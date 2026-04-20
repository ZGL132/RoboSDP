#include "core/repository/LocalJsonRepository.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>

namespace RoboSDP::Repository
{

namespace
{

QString NormalizeRelativePath(const QString& relativePath)
{
    return QDir::fromNativeSeparators(relativePath.trimmed());
}

} // namespace

RoboSDP::Errors::ErrorCode LocalJsonRepository::OpenProject(const QString& projectRootPath)
{
    if (projectRootPath.trimmed().isEmpty())
    {
        return RoboSDP::Errors::ErrorCode::InvalidArgument;
    }

    const QString absoluteRootPath = QDir(projectRootPath).absolutePath();
    if (!QDir().mkpath(absoluteRootPath))
    {
        return RoboSDP::Errors::ErrorCode::RepositoryRootNotFound;
    }

    m_project_root_path = absoluteRootPath;
    return RoboSDP::Errors::ErrorCode::Ok;
}

RoboSDP::Errors::ErrorCode LocalJsonRepository::ReadDocument(
    const QString& relativePath,
    QJsonObject& document) const
{
    if (m_project_root_path.isEmpty())
    {
        return RoboSDP::Errors::ErrorCode::RepositoryNotInitialized;
    }

    const QString absoluteFilePath = QDir(m_project_root_path).filePath(NormalizeRelativePath(relativePath));
    QFile file(absoluteFilePath);
    if (!file.exists())
    {
        return RoboSDP::Errors::ErrorCode::RepositoryDocumentNotFound;
    }

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return RoboSDP::Errors::ErrorCode::RepositoryReadFailed;
    }

    const QByteArray rawContent = file.readAll();
    file.close();

    QJsonParseError parseError;
    const QJsonDocument jsonDocument = QJsonDocument::fromJson(rawContent, &parseError);
    if (parseError.error != QJsonParseError::NoError || !jsonDocument.isObject())
    {
        return RoboSDP::Errors::ErrorCode::JsonFormatInvalid;
    }

    document = jsonDocument.object();
    return RoboSDP::Errors::ErrorCode::Ok;
}

RoboSDP::Errors::ErrorCode LocalJsonRepository::WriteDocument(
    const QString& relativePath,
    const QJsonObject& document)
{
    if (m_project_root_path.isEmpty())
    {
        return RoboSDP::Errors::ErrorCode::RepositoryNotInitialized;
    }

    const QString normalizedRelativePath = NormalizeRelativePath(relativePath);
    const QString absoluteFilePath = QDir(m_project_root_path).filePath(normalizedRelativePath);
    const QFileInfo fileInfo(absoluteFilePath);

    if (!QDir().mkpath(fileInfo.absolutePath()))
    {
        return RoboSDP::Errors::ErrorCode::RepositoryWriteFailed;
    }

    QFile file(absoluteFilePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    {
        return RoboSDP::Errors::ErrorCode::RepositoryWriteFailed;
    }

    const QJsonDocument jsonDocument(document);
    const qint64 writtenBytes = file.write(jsonDocument.toJson(QJsonDocument::Indented));
    file.close();

    if (writtenBytes <= 0)
    {
        return RoboSDP::Errors::ErrorCode::RepositoryWriteFailed;
    }

    return RoboSDP::Errors::ErrorCode::Ok;
}

QString LocalJsonRepository::ProjectRootPath() const
{
    return m_project_root_path;
}

} // namespace RoboSDP::Repository
