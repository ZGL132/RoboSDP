#include "core/repository/LocalJsonRepository.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/dynamics/service/DynamicsService.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/kinematics/service/KinematicsService.h"
#include "modules/requirement/persistence/RequirementJsonStorage.h"
#include "modules/topology/persistence/TopologyJsonStorage.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include <cmath>

namespace
{

struct LoadedProjectState
{
    RoboSDP::Requirement::Dto::RequirementModelDto requirement;
    RoboSDP::Topology::Dto::TopologyWorkspaceStateDto topology;
    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto kinematics;
    RoboSDP::Dynamics::Dto::DynamicsWorkspaceStateDto dynamics;
};

QString ReadTextFile(const QString& filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return {};
    }

    return QString::fromUtf8(file.readAll());
}

bool WriteFixtureBundleToProject(
    const QString& fixturePath,
    const QString& projectRootPath,
    RoboSDP::Repository::LocalJsonRepository& repository,
    const QHash<QString, QString>& replacements = {})
{
    QString rawContent = ReadTextFile(fixturePath);
    if (rawContent.trimmed().isEmpty())
    {
        return false;
    }

    for (auto it = replacements.constBegin(); it != replacements.constEnd(); ++it)
    {
        rawContent.replace(it.key(), it.value());
    }

    QJsonParseError parseError;
    const QJsonDocument fixtureDocument = QJsonDocument::fromJson(rawContent.toUtf8(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !fixtureDocument.isObject())
    {
        return false;
    }

    if (repository.OpenProject(projectRootPath) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return false;
    }

    const QJsonObject filesObject = fixtureDocument.object().value(QStringLiteral("files")).toObject();
    for (auto it = filesObject.constBegin(); it != filesObject.constEnd(); ++it)
    {
        if (!it.value().isObject())
        {
            return false;
        }

        if (repository.WriteDocument(it.key(), it.value().toObject()) != RoboSDP::Errors::ErrorCode::Ok)
        {
            return false;
        }
    }

    return true;
}

bool LoadAllStorages(
    const QString& projectRootPath,
    RoboSDP::Requirement::Persistence::RequirementJsonStorage& requirementStorage,
    RoboSDP::Topology::Persistence::TopologyJsonStorage& topologyStorage,
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage& kinematicStorage,
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage& dynamicStorage,
    LoadedProjectState& loadedState)
{
    using namespace RoboSDP::Errors;

    return requirementStorage.Load(projectRootPath, loadedState.requirement) == ErrorCode::Ok &&
           topologyStorage.Load(projectRootPath, loadedState.topology) == ErrorCode::Ok &&
           kinematicStorage.LoadModel(projectRootPath, loadedState.kinematics) == ErrorCode::Ok &&
           dynamicStorage.Load(projectRootPath, loadedState.dynamics) == ErrorCode::Ok;
}

bool HasAnyNonZeroTorque(const std::vector<double>& torques)
{
    for (double torque : torques)
    {
        if (std::abs(torque) > 1.0e-6)
        {
            return true;
        }
    }
    return false;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    using namespace RoboSDP::Errors;

    const QDir sourceRoot(QStringLiteral(ROBOSDP_TEST_PROJECT_SOURCE_DIR));
    const QString legacyFixturePath = sourceRoot.absoluteFilePath(
        QStringLiteral("tests/integration/project-migration/fixtures/legacy_v1_project.json"));
    const QString modernFixturePath = sourceRoot.absoluteFilePath(
        QStringLiteral("tests/integration/project-migration/fixtures/modern_v2_project.json"));
    const QString sampleUrdfPath = QDir::fromNativeSeparators(sourceRoot.absoluteFilePath(
        QStringLiteral("resources/sample-projects/stage1-demo-project/kinematics/sample-6r-preview.urdf")));

    if (!QFile::exists(legacyFixturePath) || !QFile::exists(modernFixturePath) || !QFile::exists(sampleUrdfPath))
    {
        return 1;
    }

    // 用例 1：Legacy 涅槃。模拟用户打开一个缺少统一语义字段的老项目。
    QTemporaryDir legacyProjectDir;
    if (!legacyProjectDir.isValid())
    {
        return 2;
    }

    RoboSDP::Repository::LocalJsonRepository legacyRepository;
    if (!WriteFixtureBundleToProject(legacyFixturePath, legacyProjectDir.path(), legacyRepository))
    {
        return 3;
    }

    RoboSDP::Requirement::Persistence::RequirementJsonStorage legacyRequirementStorage(legacyRepository);
    RoboSDP::Topology::Persistence::TopologyJsonStorage legacyTopologyStorage(legacyRepository);
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage legacyKinematicStorage(legacyRepository);
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage legacyDynamicStorage(legacyRepository);
    LoadedProjectState legacyLoadedState;
    if (!LoadAllStorages(
            legacyProjectDir.path(),
            legacyRequirementStorage,
            legacyTopologyStorage,
            legacyKinematicStorage,
            legacyDynamicStorage,
            legacyLoadedState))
    {
        return 4;
    }

    RoboSDP::Kinematics::Service::KinematicsService legacyKinematicsService(
        legacyKinematicStorage,
        legacyTopologyStorage,
        nullptr);
    const auto legacyBuildContext =
        legacyKinematicsService.InspectBackendBuildContext(legacyLoadedState.kinematics.current_model);
    if (!legacyBuildContext.IsSuccess() ||
        !legacyBuildContext.status.shared_robot_kernel_ready ||
        legacyBuildContext.context.normalized_model.modeling_mode != QStringLiteral("DH") ||
        legacyBuildContext.context.normalized_model.parameter_convention != QStringLiteral("DH") ||
        legacyLoadedState.kinematics.current_model.joint_order_signature.trimmed().isEmpty() ||
        legacyBuildContext.status.status_message.trimmed().isEmpty())
    {
        return 5;
    }

    RoboSDP::Dynamics::Service::DynamicsService legacyDynamicsService(
        legacyDynamicStorage,
        legacyKinematicStorage,
        nullptr);
    const std::vector<double> zeroPositions(
        static_cast<std::size_t>(legacyLoadedState.kinematics.current_model.joint_count),
        0.0);
    const auto legacyRneaResult = legacyDynamicsService.InspectNativeRneaDryRun(
        legacyLoadedState.kinematics.current_model,
        legacyLoadedState.dynamics.current_model,
        zeroPositions,
        zeroPositions,
        zeroPositions);
    if (!legacyRneaResult.success ||
        legacyRneaResult.joint_torques_nm.size() != zeroPositions.size() ||
        !HasAnyNonZeroTorque(legacyRneaResult.joint_torques_nm))
    {
        return 6;
    }

    // 用例 2：Modern 稳定。现代项目应直接 ready，且不出现兼容迁移预警。
    QTemporaryDir modernProjectDir;
    if (!modernProjectDir.isValid())
    {
        return 7;
    }

    RoboSDP::Repository::LocalJsonRepository modernRepository;
    if (!WriteFixtureBundleToProject(
            modernFixturePath,
            modernProjectDir.path(),
            modernRepository,
            {{QStringLiteral("__SAMPLE_URDF_PATH__"), sampleUrdfPath}}))
    {
        return 8;
    }

    RoboSDP::Requirement::Persistence::RequirementJsonStorage modernRequirementStorage(modernRepository);
    RoboSDP::Topology::Persistence::TopologyJsonStorage modernTopologyStorage(modernRepository);
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage modernKinematicStorage(modernRepository);
    RoboSDP::Dynamics::Persistence::DynamicJsonStorage modernDynamicStorage(modernRepository);
    LoadedProjectState modernLoadedState;
    if (!LoadAllStorages(
            modernProjectDir.path(),
            modernRequirementStorage,
            modernTopologyStorage,
            modernKinematicStorage,
            modernDynamicStorage,
            modernLoadedState))
    {
        return 9;
    }

    RoboSDP::Kinematics::Service::KinematicsService modernKinematicsService(
        modernKinematicStorage,
        modernTopologyStorage,
        nullptr);
    const auto modernBuildContext =
        modernKinematicsService.InspectBackendBuildContext(modernLoadedState.kinematics.current_model);
    if (!modernBuildContext.IsSuccess() ||
        !modernBuildContext.status.shared_robot_kernel_ready ||
        modernBuildContext.context.normalized_model.modeling_mode != QStringLiteral("URDF") ||
        modernBuildContext.context.normalized_model.model_source_mode != QStringLiteral("urdf_imported") ||
        modernBuildContext.status.status_message.contains(QStringLiteral("预警")) ||
        modernBuildContext.status.status_message.contains(QStringLiteral("Mesh")) ||
        modernBuildContext.status.status_message.contains(QStringLiteral("兼容")))
    {
        return 10;
    }

    // 用例 3：Save-Load 往返。老项目一旦重新保存，新时代显式字段必须写回磁盘。
    if (legacyKinematicStorage.SaveModel(legacyProjectDir.path(), legacyLoadedState.kinematics) != ErrorCode::Ok)
    {
        return 11;
    }

    QJsonObject upgradedLegacyModelDocument;
    if (legacyRepository.ReadDocument(
            legacyKinematicStorage.RelativeModelFilePath(),
            upgradedLegacyModelDocument) != ErrorCode::Ok)
    {
        return 12;
    }
    if (!upgradedLegacyModelDocument.contains(QStringLiteral("modeling_mode")) ||
        !upgradedLegacyModelDocument.contains(QStringLiteral("model_source_mode")) ||
        !upgradedLegacyModelDocument.contains(QStringLiteral("backend_type")) ||
        !upgradedLegacyModelDocument.contains(QStringLiteral("frame_semantics_version")) ||
        !upgradedLegacyModelDocument.contains(QStringLiteral("joint_order_signature")))
    {
        return 13;
    }

    RoboSDP::Repository::LocalJsonRepository reopenedLegacyRepository;
    if (reopenedLegacyRepository.OpenProject(legacyProjectDir.path()) != ErrorCode::Ok)
    {
        return 14;
    }
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage reopenedLegacyKinematicStorage(reopenedLegacyRepository);
    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto reopenedLegacyState;
    if (reopenedLegacyKinematicStorage.LoadModel(legacyProjectDir.path(), reopenedLegacyState) != ErrorCode::Ok ||
        reopenedLegacyState.current_model.modeling_mode != QStringLiteral("DH") ||
        reopenedLegacyState.current_model.model_source_mode != QStringLiteral("manual_seed") ||
        reopenedLegacyState.current_model.backend_type != QStringLiteral("pinocchio_kinematic_backend") ||
        reopenedLegacyState.current_model.frame_semantics_version != 1 ||
        reopenedLegacyState.current_model.joint_order_signature.trimmed().isEmpty())
    {
        return 15;
    }

    return 0;
}
