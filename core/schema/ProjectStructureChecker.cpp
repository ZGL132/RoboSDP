#include "core/schema/ProjectStructureChecker.h"

#include <QDir>
#include <QFileInfo>

namespace RoboSDP::Schema
{

namespace
{

struct DirectorySpec
{
    QString field;
    QString relative_path;
};

struct FileSpec
{
    QString field;
    QString relative_path;
};

QList<DirectorySpec> BuildDirectorySpecs()
{
    return {
        {QStringLiteral("project.requirements_dir"), QStringLiteral("requirements")},
        {QStringLiteral("project.topology_dir"), QStringLiteral("topology")},
        {QStringLiteral("project.kinematics_dir"), QStringLiteral("kinematics")},
        {QStringLiteral("project.dynamics_dir"), QStringLiteral("dynamics")},
        {QStringLiteral("project.selection_dir"), QStringLiteral("selection")},
        {QStringLiteral("project.planning_dir"), QStringLiteral("planning")},
        {QStringLiteral("project.snapshots_dir"), QStringLiteral("snapshots")},
        {QStringLiteral("project.exports_dir"), QStringLiteral("exports")}};
}

QList<FileSpec> BuildFileSpecs()
{
    return {
        {QStringLiteral("project.requirement_file"), QStringLiteral("requirements/requirement-model.json")},
        {QStringLiteral("project.topology_file"), QStringLiteral("topology/topology-model.json")},
        {QStringLiteral("project.kinematics_model_file"), QStringLiteral("kinematics/kinematic-model.json")},
        {QStringLiteral("project.kinematics_workspace_file"), QStringLiteral("kinematics/workspace-cache.json")},
        {QStringLiteral("project.dynamics_model_file"), QStringLiteral("dynamics/dynamic-model.json")},
        {QStringLiteral("project.dynamics_mass_file"), QStringLiteral("dynamics/mass-properties.json")},
        {QStringLiteral("project.dynamics_envelope_file"), QStringLiteral("dynamics/load-envelopes.json")},
        {QStringLiteral("project.selection_motor_file"), QStringLiteral("selection/motor-selection.json")},
        {QStringLiteral("project.selection_reducer_file"), QStringLiteral("selection/reducer-selection.json")},
        {QStringLiteral("project.selection_drive_file"), QStringLiteral("selection/drivetrain-selection.json")},
        {QStringLiteral("project.planning_scene_file"), QStringLiteral("planning/planning-scene.json")},
        {QStringLiteral("project.planning_request_file"), QStringLiteral("planning/planning-requests.json")},
        {QStringLiteral("project.planning_result_file"), QStringLiteral("planning/planning-results.json")},
        {QStringLiteral("project.scheme_snapshot_file"), QStringLiteral("snapshots/scheme-snapshot.json")},
        {QStringLiteral("project.scheme_export_file"), QStringLiteral("exports/scheme-export.json")}};
}

} // namespace

ValidationResult ProjectStructureChecker::CheckProject(const QString& projectRootPath) const
{
    ValidationResult result;

    if (projectRootPath.trimmed().isEmpty())
    {
        result.Add(
            QStringLiteral("project.root"),
            QStringLiteral("PROJECT_ROOT_REQUIRED"),
            QStringLiteral("项目目录不能为空，无法检查目录结构。"));
        return result;
    }

    const QDir projectDir(projectRootPath);
    if (!projectDir.exists())
    {
        result.Add(
            QStringLiteral("project.root"),
            QStringLiteral("PROJECT_ROOT_NOT_FOUND"),
            QStringLiteral("项目目录不存在：%1").arg(projectRootPath));
        return result;
    }

    // 目录检查只验证当前第一阶段约定的最小目录是否存在。
    for (const auto& spec : BuildDirectorySpecs())
    {
        if (!projectDir.exists(spec.relative_path))
        {
            result.Add(
                spec.field,
                QStringLiteral("PROJECT_DIRECTORY_MISSING"),
                QStringLiteral("缺少目录：%1").arg(spec.relative_path),
                ValidationSeverity::Warning);
        }
    }

    // 文件检查只看当前主链保存路径，不检查历史兼容文件或二期目录。
    for (const auto& spec : BuildFileSpecs())
    {
        const QFileInfo fileInfo(projectDir.filePath(spec.relative_path));
        if (!fileInfo.exists() || !fileInfo.isFile())
        {
            result.Add(
                spec.field,
                QStringLiteral("PROJECT_FILE_MISSING"),
                QStringLiteral("缺少主文件：%1").arg(spec.relative_path),
                ValidationSeverity::Warning);
        }
    }

    return result;
}

} // namespace RoboSDP::Schema
