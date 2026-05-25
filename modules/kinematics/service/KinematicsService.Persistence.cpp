#include "modules/kinematics/service/KinematicsService.h"
#include "modules/kinematics/service/KinematicsServiceInternal.h"

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

    // 保存前极其重要的拦截：拒绝将错误的、非法的脏数据写入磁盘污染项目
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

    // 先保存核心的模型信息
    result.error_code = m_storage.SaveModel(projectRootPath, persistedState);
    if (result.error_code == RoboSDP::Errors::ErrorCode::Ok)
    {
        // 只有核心模型确保安全落盘后，才去保存庞大的工作区点云采样缓存
        result.error_code = m_storage.SaveWorkspaceCache(projectRootPath, persistedState.last_workspace_result);
    }

    // 格式化友好的反馈信息
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

    // 调用持久化层读取文件内容反序列化进 state
    result.error_code = m_storage.LoadModel(projectRootPath, result.state);
    if (result.error_code != RoboSDP::Errors::ErrorCode::Ok)
    {
        result.message = QStringLiteral("Kinematics 加载失败：%1").arg(RoboSDP::Errors::ToChineseMessage(result.error_code));
        return result;
    }

    // 安全防御点：即使从本地读上来的模型也必须校验，
    // 因为外部可能存在手动使用文本编辑器篡改 JSON 注入恶意/非法参数的行为。
    const auto validation = ValidateModel(result.state.current_model);
    if (!validation.success)
    {
        result.error_code = RoboSDP::Errors::ErrorCode::InvalidArgument;
        result.message = validation.message;
        return result;
    }

    // 尝试加载可能存在的采样点云缓存
    RoboSDP::Kinematics::Dto::WorkspaceResultDto workspaceResult;
    const RoboSDP::Errors::ErrorCode workspaceLoadError = m_storage.LoadWorkspaceCache(projectRootPath, workspaceResult);
    if (workspaceLoadError == RoboSDP::Errors::ErrorCode::Ok)
    {
        result.state.last_workspace_result = workspaceResult;
        result.message = QStringLiteral("Kinematics 已从 JSON 重新加载。");
    }
    else
    {
        // 容错处理：采样缓存较大或可丢弃，它的缺失不应该阻断核心模型的正常加载
        result.message = QStringLiteral("Kinematics 模型已加载，工作空间缓存未找到，已保留默认空结果。");
    }

    return result;
}

} // namespace RoboSDP::Kinematics::Service