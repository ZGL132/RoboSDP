#pragma once

#include "modules/kinematics/dto/KinematicModelDto.h"

#include <QString>

namespace RoboSDP::Kinematics::Domain
{
/**
 * @brief 运动学模型的设计真源（State Machine）与切换策略规约。
 *
 * @details
 * 管理两种主链状态之间的流转：
 * 1.参数化设计主链（DH/MDH Master）：可编辑参数，并派生最小 URDF。
 * 2.工程参考参考（URDF Master）：导入外部 URDF 骨架，DH 表变为只读诊断草案。
 * 提供 ApplyDhParametricState 和 ApplyImportedUrdfReferenceState 状态机切换函数，实现 “URDF 草案转换为可编辑 DH（Promote）” 与 “回到 URDF 参考（Return）” 的一致性校验。
 */
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
