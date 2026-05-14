#include "modules/kinematics/domain/KinematicModelStatePolicy.h"

#include <iostream>

namespace
{

using RoboSDP::Kinematics::Domain::KinematicModelStatePolicy;
using RoboSDP::Kinematics::Dto::KinematicJointLimitDto;
using RoboSDP::Kinematics::Dto::KinematicLinkParameterDto;
using RoboSDP::Kinematics::Dto::KinematicModelDto;

bool Expect(bool condition, const char* message)
{
    if (condition)
    {
        return true;
    }
    std::cerr << message << '\n';
    return false;
}

KinematicModelDto MakeDraft(const QString& extractionLevel)
{
    KinematicModelDto model;
    model.links = {KinematicLinkParameterDto{}, KinematicLinkParameterDto{}};
    model.joint_limits = {KinematicJointLimitDto{}, KinematicJointLimitDto{}};
    model.parameter_convention = QStringLiteral("DH");
    model.dh_draft_extraction_level = extractionLevel;
    KinematicModelStatePolicy::ApplyImportedUrdfReferenceState(
        model,
        QStringLiteral("D:/robots/arm.urdf"),
        QStringLiteral("original_imported"));
    return model;
}

} // namespace

int main()
{
    bool ok = true;

    auto fullDraft = MakeDraft(QStringLiteral("full"));
    ok &= Expect(KinematicModelStatePolicy::IsImportedUrdfReference(fullDraft),
                 "full URDF draft should be an imported URDF reference");
    ok &= Expect(KinematicModelStatePolicy::IsOriginalImportedUrdfReference(fullDraft),
                 "full URDF draft should be an original imported URDF reference");
    ok &= Expect(KinematicModelStatePolicy::CanCopyUrdfDraftToDh(fullDraft),
                 "full URDF draft should be copyable to DH");
    ok &= Expect(!fullDraft.dh_editable, "imported URDF reference must keep DH table readonly");

    auto derivedDraft = MakeDraft(QStringLiteral("full"));
    KinematicModelStatePolicy::ApplyImportedUrdfReferenceState(
        derivedDraft,
        QStringLiteral("D:/robots/derived/arm.urdf"),
        QStringLiteral("project_derived"));
    ok &= Expect(!KinematicModelStatePolicy::CanCopyUrdfDraftToDh(derivedDraft),
                 "project-derived URDF draft must not be copyable back to DH");
    ok &= Expect(!KinematicModelStatePolicy::IsOriginalImportedUrdfReference(derivedDraft),
                 "project-derived URDF should not be treated as original imported reference");

    auto diagnosticDraft = MakeDraft(QStringLiteral("diagnostic_only"));
    ok &= Expect(!KinematicModelStatePolicy::CanCopyUrdfDraftToDh(diagnosticDraft),
                 "diagnostic_only draft must not be copyable to DH");

    auto copiedDh = KinematicModelStatePolicy::CreateDhCopyFromUrdfDraft(fullDraft);
    ok &= Expect(KinematicModelStatePolicy::IsDhParametric(copiedDh),
                 "copied draft should become a DH/MDH parametric model");
    ok &= Expect(copiedDh.dh_editable, "copied DH model should be editable");
    ok &= Expect(copiedDh.urdf_source_path.isEmpty(), "copied DH model should not keep URDF as active source");
    ok &= Expect(copiedDh.model_source_mode == QStringLiteral("urdf_draft_copied"),
                 "copied DH model should preserve source mode semantics");

    KinematicModelDto legacyUrdf;
    legacyUrdf.master_model_type = QStringLiteral("urdf");
    legacyUrdf.dh_editable = false;
    legacyUrdf.original_imported_urdf_path = QStringLiteral("D:/legacy/original.urdf");
    KinematicModelStatePolicy::NormalizeAfterLoad(legacyUrdf);
    ok &= Expect(KinematicModelStatePolicy::IsImportedUrdfReference(legacyUrdf),
                 "legacy URDF project should normalize to imported URDF reference");
    ok &= Expect(legacyUrdf.urdf_source_path == legacyUrdf.original_imported_urdf_path,
                 "legacy URDF source path should fall back to original imported path");
    ok &= Expect(legacyUrdf.urdf_master_source_type == QStringLiteral("original_imported"),
                 "legacy URDF source type should normalize none to original_imported");

    KinematicModelDto dhModel;
    dhModel.master_model_type.clear();
    dhModel.dh_editable = true;
    KinematicModelStatePolicy::NormalizeAfterLoad(dhModel);
    ok &= Expect(KinematicModelStatePolicy::IsDhParametric(dhModel),
                 "legacy DH project should normalize to editable DH/MDH parametric state");

    return ok ? 0 : 1;
}
