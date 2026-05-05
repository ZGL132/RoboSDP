#include "modules/kinematics/service/KinematicsService.h"
#include "modules/kinematics/service/KinematicsServiceInternal.h"

#include <QDir>

namespace RoboSDP::Kinematics::Service
{

using namespace Internal;

KinematicSaveResult KinematicsService::SaveDraft(
    const QString& projectRootPath,
    const RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto& state) const
{
    using namespace RoboSDP::Kinematics::Dto;

    KinematicSaveResult result;
    result.model_file_path = m_storage.BuildAbsoluteModelFilePath(projectRootPath);
    result.workspace_file_path = m_storage.BuildAbsoluteWorkspaceFilePath(projectRootPath);

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法保存 Kinematics 草稿。");
        return result;
    }

    const auto validation = ValidateModel(state.current_model);
    if (!validation.success)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = validation.message;
        return result;
    }

    KinematicsWorkspaceStateDto persistedState = state;
    if (persistedState.current_model.master_model_type != QStringLiteral("urdf"))
    {
        QString artifactDiagnosticMessage;
        auto snapshot = persistedState.current_model.unified_robot_snapshot;
        const RoboSDP::Errors::ErrorCode artifactError = WriteDerivedUrdfArtifact(
            projectRootPath,
            persistedState.current_model,
            snapshot,
            artifactDiagnosticMessage);
        persistedState.current_model.unified_robot_snapshot = snapshot;
        persistedState.current_model.conversion_diagnostics = artifactDiagnosticMessage;

        if (artifactError != RoboSDP::Errors::ErrorCode::Ok)
        {
            result.error_code = artifactError;
            result.message = artifactDiagnosticMessage;
            return result;
        }
    }

    result.error_code = m_storage.SaveModel(projectRootPath, persistedState);
    if (result.error_code == RoboSDP::Errors::ErrorCode::Ok)
        result.error_code = m_storage.SaveWorkspaceCache(projectRootPath, persistedState.last_workspace_result);

    result.message = result.IsSuccess()
        ? QStringLiteral("Kinematics 草稿已保存到：%1").arg(result.model_file_path)
        : QStringLiteral("Kinematics 草稿保存失败：%1").arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
    return result;
}

KinematicLoadResult KinematicsService::LoadDraft(const QString& projectRootPath) const
{
    KinematicLoadResult result;
    result.model_file_path = m_storage.BuildAbsoluteModelFilePath(projectRootPath);
    result.workspace_file_path = m_storage.BuildAbsoluteWorkspaceFilePath(projectRootPath);
    result.state = CreateDefaultState();

    if (projectRootPath.trimmed().isEmpty())
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = QStringLiteral("项目目录不能为空，无法加载 Kinematics 草稿。");
        return result;
    }

    result.error_code = m_storage.LoadModel(projectRootPath, result.state);
    if (result.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.message = QStringLiteral("Kinematics 加载失败：%1")
            .arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        return result;
    }

    const auto validation = ValidateModel(result.state.current_model);
    if (!validation.success)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = validation.message;
        return result;
    }

    RoboSDP::Kinematics::Dto::WorkspaceResultDto workspaceResult;
    const RoboSDP::Errors::ErrorCode workspaceLoadError =
        m_storage.LoadWorkspaceCache(projectRootPath, workspaceResult);
    if (workspaceLoadError == RoboSDP::Errors::ErrorCode::Ok)
    {
        result.state.last_workspace_result = workspaceResult;
        result.message = QStringLiteral("Kinematics 已从 JSON 重新加载。");
    }
    else
    {
        result.message = QStringLiteral("Kinematics 模型已加载，工作空间缓存未找到，已保留默认空结果。");
    }

    return result;
}

} // namespace RoboSDP::Kinematics::Service
