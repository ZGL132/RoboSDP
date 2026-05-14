#pragma once

#include "modules/kinematics/dto/KinematicModelDto.h"

#include <QString>

namespace RoboSDP::Kinematics::Domain
{

class KinematicModelStatePolicy
{
public:
    static bool IsDhParametric(const RoboSDP::Kinematics::Dto::KinematicModelDto& model);
    static bool IsImportedUrdfReference(const RoboSDP::Kinematics::Dto::KinematicModelDto& model);
    static bool IsOriginalImportedUrdfReference(const RoboSDP::Kinematics::Dto::KinematicModelDto& model);
    static bool CanCopyUrdfDraftToDh(const RoboSDP::Kinematics::Dto::KinematicModelDto& model);

    static void ApplyDhParametricState(
        RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const QString& sourceMode);

    static void ApplyImportedUrdfReferenceState(
        RoboSDP::Kinematics::Dto::KinematicModelDto& model,
        const QString& urdfPath,
        const QString& sourceType);

    static RoboSDP::Kinematics::Dto::KinematicModelDto CreateDhCopyFromUrdfDraft(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& urdfDraftModel);

    static void NormalizeAfterLoad(RoboSDP::Kinematics::Dto::KinematicModelDto& model);
    static QString BuildUserFacingStateText(
        const RoboSDP::Kinematics::Dto::KinematicModelDto& model);

private:
    static bool IsDhConvention(const QString& convention);
    static void SyncSnapshot(RoboSDP::Kinematics::Dto::KinematicModelDto& model);
};

} // namespace RoboSDP::Kinematics::Domain
