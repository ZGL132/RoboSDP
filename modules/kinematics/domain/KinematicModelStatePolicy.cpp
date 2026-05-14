#include "modules/kinematics/domain/KinematicModelStatePolicy.h"

namespace RoboSDP::Kinematics::Domain
{

namespace
{

constexpr auto kDhMaster = "dh_mdh";
constexpr auto kUrdfMaster = "urdf";
constexpr auto kDh = "DH";
constexpr auto kMdh = "MDH";
constexpr auto kUrdf = "URDF";
constexpr auto kBackend = "pinocchio_kinematic_backend";
constexpr auto kNone = "none";
constexpr auto kDiagnosticOnly = "diagnostic_only";
constexpr auto kFull = "full";
constexpr auto kPartial = "partial";

QString NormalizedSourceMode(const QString& sourceMode)
{
    return sourceMode.trimmed().isEmpty()
        ? QStringLiteral("manual_seed")
        : sourceMode.trimmed();
}

QString DerivedArtifactPathFor(const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    const QString id = model.meta.kinematic_id.trimmed().isEmpty()
        ? QStringLiteral("default")
        : model.meta.kinematic_id.trimmed();
    return QStringLiteral("kinematics/derived/%1.urdf").arg(id);
}

QString Token(const char* value)
{
    return QString::fromLatin1(value);
}

} // namespace

bool KinematicModelStatePolicy::IsDhConvention(const QString& convention)
{
    return convention == Token(kDh) || convention == Token(kMdh);
}

bool KinematicModelStatePolicy::IsDhParametric(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    return model.master_model_type == Token(kDhMaster) && model.dh_editable;
}

bool KinematicModelStatePolicy::IsImportedUrdfReference(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    return model.master_model_type == Token(kUrdfMaster) &&
           !model.dh_editable &&
           !model.urdf_source_path.trimmed().isEmpty();
}

bool KinematicModelStatePolicy::IsOriginalImportedUrdfReference(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    return IsImportedUrdfReference(model) &&
           model.urdf_master_source_type == QStringLiteral("original_imported");
}

bool KinematicModelStatePolicy::CanCopyUrdfDraftToDh(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    const QString level = model.dh_draft_extraction_level.trimmed();
    return IsOriginalImportedUrdfReference(model) &&
           !model.links.empty() &&
           !model.joint_limits.empty() &&
           model.links.size() == model.joint_limits.size() &&
           (level == Token(kFull) || level == Token(kPartial));
}

void KinematicModelStatePolicy::ApplyDhParametricState(
    RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const QString& sourceMode)
{
    model.master_model_type = Token(kDhMaster);
    model.dh_editable = true;
    model.urdf_editable = false;
    if (!IsDhConvention(model.parameter_convention))
    {
        model.parameter_convention = Token(kDh);
    }
    model.modeling_mode = model.parameter_convention;
    model.model_source_mode = NormalizedSourceMode(sourceMode);
    model.backend_type = Token(kBackend);
    model.urdf_source_path.clear();
    model.urdf_master_source_type = Token(kNone);
    model.dh_draft_extraction_level.clear();
    model.dh_draft_readonly_reason.clear();
    model.joint_count = static_cast<int>(model.links.size());
    model.conversion_diagnostics =
        model.model_source_mode == QStringLiteral("topology_derived")
            ? QStringLiteral("当前设计真源为 Topology 生成的 DH/MDH 参数化模型。")
            : QStringLiteral("当前设计真源为 DH/MDH 参数化模型。");
    if (model.unified_robot_snapshot.derived_artifact_relative_path.trimmed().isEmpty())
    {
        model.unified_robot_snapshot.derived_artifact_relative_path = DerivedArtifactPathFor(model);
    }
    SyncSnapshot(model);
}

void KinematicModelStatePolicy::ApplyImportedUrdfReferenceState(
    RoboSDP::Kinematics::Dto::KinematicModelDto& model,
    const QString& urdfPath,
    const QString& sourceType)
{
    model.master_model_type = Token(kUrdfMaster);
    model.modeling_mode = Token(kUrdf);
    // 保留 DH/MDH 草案的参数约定，便于只读诊断表显示；无草案时使用 DH 兜底以兼容校验链路。
    if (!IsDhConvention(model.parameter_convention))
    {
        model.parameter_convention = Token(kDh);
    }
    model.dh_editable = false;
    model.urdf_editable = true;
    model.model_source_mode = QStringLiteral("urdf_imported");
    model.backend_type = Token(kBackend);
    model.urdf_source_path = urdfPath;
    const QString normalizedSourceType =
        sourceType.trimmed().isEmpty() || sourceType == Token(kNone)
            ? QStringLiteral("original_imported")
            : sourceType.trimmed();
    if (normalizedSourceType == QStringLiteral("original_imported"))
    {
        model.original_imported_urdf_path = urdfPath;
    }
    model.urdf_master_source_type = normalizedSourceType;
    if (model.dh_draft_extraction_level.trimmed().isEmpty())
    {
        model.dh_draft_extraction_level = Token(kDiagnosticOnly);
    }
    model.dh_draft_readonly_reason = normalizedSourceType == QStringLiteral("project_derived")
        ? QStringLiteral("当前文件是 DH/MDH 派生的最小 URDF，仅用于交换/预览，不允许再复制回 DH 模型。请继续使用 Topology + DH/MDH 参数化设计。")
        : QStringLiteral("当前 DH/MDH 参数表是由工程 URDF 提取的只读诊断草案；修改参数化设计请复制为 DH 模型或从 Topology 生成。");
    const QString referenceDiagnostics =
        normalizedSourceType == QStringLiteral("project_derived")
            ? QStringLiteral("当前视图来源为 DH 派生最小 URDF 预览；该文件不是设计真源，不能再复制回 DH。")
            : QStringLiteral("当前视图来源为工程 URDF 参考模型；DH/MDH 参数表仅作只读诊断草案。");
    const QString previousDiagnostics = model.conversion_diagnostics.trimmed();
    if (previousDiagnostics.isEmpty())
    {
        model.conversion_diagnostics = referenceDiagnostics;
    }
    else if (previousDiagnostics.contains(referenceDiagnostics))
    {
        model.conversion_diagnostics = previousDiagnostics;
    }
    else
    {
        model.conversion_diagnostics =
            QStringLiteral("%1\n%2").arg(referenceDiagnostics, previousDiagnostics);
    }
    model.joint_count = model.links.empty()
        ? model.joint_count
        : static_cast<int>(model.links.size());
    SyncSnapshot(model);
}

RoboSDP::Kinematics::Dto::KinematicModelDto
KinematicModelStatePolicy::CreateDhCopyFromUrdfDraft(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& urdfDraftModel)
{
    auto model = urdfDraftModel;
    ApplyDhParametricState(model, QStringLiteral("urdf_draft_copied"));
    model.derived_model_state = QStringLiteral("stale");
    model.pinocchio_model_ready = false;
    model.unified_robot_model_ref.clear();
    model.joint_order_signature.clear();
    model.unified_robot_snapshot = {};
    model.unified_robot_snapshot.derived_artifact_relative_path = DerivedArtifactPathFor(model);
    model.conversion_diagnostics =
        QStringLiteral("当前设计真源为从工程 URDF 诊断草案复制出的 DH/MDH 参数化模型；原始 URDF 不会被反写。");
    SyncSnapshot(model);
    return model;
}

void KinematicModelStatePolicy::NormalizeAfterLoad(
    RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    if (model.master_model_type == Token(kUrdfMaster) ||
        (!model.urdf_source_path.trimmed().isEmpty() && !model.dh_editable))
    {
        const QString urdfPath = model.urdf_source_path.trimmed().isEmpty()
            ? model.original_imported_urdf_path
            : model.urdf_source_path;
        ApplyImportedUrdfReferenceState(
            model,
            urdfPath,
            model.urdf_master_source_type.trimmed().isEmpty() ||
                model.urdf_master_source_type == Token(kNone)
                ? QStringLiteral("original_imported")
                : model.urdf_master_source_type);
        return;
    }

    ApplyDhParametricState(
        model,
        model.model_source_mode.trimmed().isEmpty()
            ? QStringLiteral("manual_seed")
            : model.model_source_mode);
}

QString KinematicModelStatePolicy::BuildUserFacingStateText(
    const RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    if (IsDhParametric(model))
    {
        if (model.model_source_mode == QStringLiteral("topology_derived"))
        {
            return QStringLiteral("当前设计真源：Topology + DH/MDH 参数化模型");
        }
        if (model.model_source_mode == QStringLiteral("urdf_draft_copied"))
        {
            return QStringLiteral("当前设计真源：由 URDF 诊断草案复制出的 DH/MDH 参数化模型");
        }
        return QStringLiteral("当前设计真源：DH/MDH 参数化模型");
    }

    if (IsImportedUrdfReference(model))
    {
        if (model.urdf_master_source_type == QStringLiteral("project_derived"))
        {
            return QStringLiteral("当前视图来源：DH 派生最小 URDF 预览；不可复制回 DH");
        }
        return QStringLiteral("当前视图来源：原始工程 URDF 参考；DH 表为只读诊断草案");
    }

    return QStringLiteral("当前模型状态：待确认");
}

void KinematicModelStatePolicy::SyncSnapshot(
    RoboSDP::Kinematics::Dto::KinematicModelDto& model)
{
    auto& snapshot = model.unified_robot_snapshot;
    snapshot.unified_robot_model_ref = model.unified_robot_model_ref;
    snapshot.source_kinematic_id = model.meta.kinematic_id;
    snapshot.master_model_type = model.master_model_type;
    snapshot.modeling_mode = model.modeling_mode;
    snapshot.parameter_convention = model.parameter_convention;
    snapshot.backend_type = model.backend_type;
    snapshot.joint_order_signature = model.joint_order_signature;
    snapshot.pinocchio_model_ready = model.pinocchio_model_ready;
    snapshot.frame_semantics_version = model.frame_semantics_version;
    snapshot.model_source_mode = model.model_source_mode;
    snapshot.conversion_diagnostics = model.conversion_diagnostics;
    if (snapshot.derived_artifact_relative_path.trimmed().isEmpty() &&
        model.master_model_type == Token(kDhMaster))
    {
        snapshot.derived_artifact_relative_path = DerivedArtifactPathFor(model);
    }
}

} // namespace RoboSDP::Kinematics::Domain
