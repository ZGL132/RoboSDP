#include "modules/selection/catalog/JsonComponentCatalog.h"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace RoboSDP::Selection::Catalog
{

namespace
{

QString BuildMotorsFilePath(const QString& catalogRootPath)
{
    return QDir(catalogRootPath).filePath(QStringLiteral("motors-stage1.json"));
}

QString BuildReducersFilePath(const QString& catalogRootPath)
{
    return QDir(catalogRootPath).filePath(QStringLiteral("reducers-stage1.json"));
}

RoboSDP::Errors::ErrorCode ReadJsonObjectFile(const QString& filePath, QJsonObject& jsonObject)
{
    QFile file(filePath);
    if (!file.exists())
    {
        return RoboSDP::Errors::ErrorCode::RepositoryDocumentNotFound;
    }

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return RoboSDP::Errors::ErrorCode::RepositoryReadFailed;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    file.close();
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        return RoboSDP::Errors::ErrorCode::JsonFormatInvalid;
    }

    jsonObject = document.object();
    return RoboSDP::Errors::ErrorCode::Ok;
}

std::vector<RoboSDP::Selection::Dto::MotorCatalogItemDto> ParseMotors(const QJsonArray& array)
{
    std::vector<RoboSDP::Selection::Dto::MotorCatalogItemDto> motors;
    motors.reserve(static_cast<std::size_t>(array.size()));
    for (const auto& value : array)
    {
        const QJsonObject object = value.toObject();
        motors.push_back({
            object.value(QStringLiteral("motor_id")).toString(),
            object.value(QStringLiteral("name")).toString(),
            object.value(QStringLiteral("vendor")).toString(),
            object.value(QStringLiteral("rated_torque_nm")).toDouble(),
            object.value(QStringLiteral("peak_torque_nm")).toDouble(),
            object.value(QStringLiteral("rated_speed_rpm")).toDouble(),
            object.value(QStringLiteral("max_speed_rpm")).toDouble(),
            object.value(QStringLiteral("rated_power_w")).toDouble(),
            object.value(QStringLiteral("rotor_inertia_kg_m2")).toDouble(),
            object.value(QStringLiteral("efficiency")).toDouble(),
            object.value(QStringLiteral("has_brake")).toBool(),
            object.value(QStringLiteral("brake_holding_torque_nm")).toDouble()});
    }
    return motors;
}

std::vector<RoboSDP::Selection::Dto::ReducerCatalogItemDto> ParseReducers(const QJsonArray& array)
{
    std::vector<RoboSDP::Selection::Dto::ReducerCatalogItemDto> reducers;
    reducers.reserve(static_cast<std::size_t>(array.size()));
    for (const auto& value : array)
    {
        const QJsonObject object = value.toObject();
        reducers.push_back({
            object.value(QStringLiteral("reducer_id")).toString(),
            object.value(QStringLiteral("name")).toString(),
            object.value(QStringLiteral("vendor")).toString(),
            object.value(QStringLiteral("ratio")).toDouble(),
            object.value(QStringLiteral("rated_output_torque_nm")).toDouble(),
            object.value(QStringLiteral("peak_output_torque_nm")).toDouble(),
            object.value(QStringLiteral("max_input_speed_rpm")).toDouble(),
            object.value(QStringLiteral("efficiency")).toDouble(),
            object.value(QStringLiteral("backlash_arcmin")).toDouble()});
    }
    return reducers;
}

} // namespace

CatalogLoadResult JsonComponentCatalog::LoadFromDirectory(const QString& catalogRootPath)
{
    CatalogLoadResult result;
    result.directory_path = QDir(catalogRootPath).absolutePath();

    if (catalogRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("元件目录不能为空。");
        return result;
    }

    QJsonObject motorsObject;
    result.error_code = ReadJsonObjectFile(BuildMotorsFilePath(result.directory_path), motorsObject);
    if (result.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.message = QStringLiteral("读取电机样例目录失败：%1")
                             .arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        return result;
    }

    QJsonObject reducersObject;
    result.error_code = ReadJsonObjectFile(BuildReducersFilePath(result.directory_path), reducersObject);
    if (result.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.message = QStringLiteral("读取减速器样例目录失败：%1")
                             .arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        return result;
    }

    m_motors = ParseMotors(motorsObject.value(QStringLiteral("motors")).toArray());
    m_reducers = ParseReducers(reducersObject.value(QStringLiteral("reducers")).toArray());
    m_loaded_directory_path = result.directory_path;

    if (m_motors.empty() || m_reducers.empty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::JsonFormatInvalid;
        result.message = QStringLiteral("外部元件目录为空或缺少有效样例。");
        return result;
    }

    result.message = QStringLiteral("已加载外部样例目录。");
    return result;
}

const std::vector<RoboSDP::Selection::Dto::MotorCatalogItemDto>& JsonComponentCatalog::Motors() const
{
    return m_motors;
}

const std::vector<RoboSDP::Selection::Dto::ReducerCatalogItemDto>& JsonComponentCatalog::Reducers() const
{
    return m_reducers;
}

QString JsonComponentCatalog::LoadedDirectoryPath() const
{
    return m_loaded_directory_path;
}

} // namespace RoboSDP::Selection::Catalog
