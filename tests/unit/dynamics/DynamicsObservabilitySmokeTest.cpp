#include "core/logging/ILogger.h"
#include "core/repository/LocalJsonRepository.h"
#include "modules/dynamics/adapter/PinocchioDynamicsBackendAdapter.h"
#include "modules/dynamics/persistence/DynamicJsonStorage.h"
#include "modules/dynamics/ui/DynamicsWidget.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"

#include <QApplication>
#include <QDir>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

namespace
{

class CapturingLogger final : public RoboSDP::Logging::ILogger
{
public:
    void Log(
        RoboSDP::Logging::LogLevel level,
        const QString& message,
        RoboSDP::Errors::ErrorCode errorCode = RoboSDP::Errors::ErrorCode::Ok,
        const RoboSDP::Logging::LogContext& context = {}) override
    {
        Q_UNUSED(errorCode);
        Q_UNUSED(context);
        entries.push_back({level, message});
    }

    int WarningCount() const
    {
        int count = 0;
        for (const auto& entry : entries)
        {
            if (entry.level == RoboSDP::Logging::LogLevel::Warning)
            {
                ++count;
            }
        }
        return count;
    }

    QString LastWarningMessage() const
    {
        for (auto iterator = entries.rbegin(); iterator != entries.rend(); ++iterator)
        {
            if (iterator->level == RoboSDP::Logging::LogLevel::Warning)
            {
                return iterator->message;
            }
        }
        return {};
    }

private:
    struct Entry
    {
        RoboSDP::Logging::LogLevel level;
        QString message;
    };

    std::vector<Entry> entries;
};

} // namespace

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    using namespace RoboSDP::Dynamics::Adapter;
    using namespace RoboSDP::Dynamics::Dto;
    using namespace RoboSDP::Dynamics::Persistence;
    using namespace RoboSDP::Dynamics::Ui;
    using namespace RoboSDP::Kinematics::Dto;
    using namespace RoboSDP::Kinematics::Persistence;
    using namespace RoboSDP::Repository;

    LocalJsonRepository repository;
    KinematicJsonStorage kinematicStorage(repository);
    DynamicJsonStorage dynamicStorage(repository);

    const QString projectRoot =
        QDir::current().absoluteFilePath(QStringLiteral("dynamics-observability-project"));

    KinematicsWorkspaceStateDto kinematicState = KinematicsWorkspaceStateDto::CreateDefault();
    kinematicState.current_model.meta.kinematic_id = QStringLiteral("kinematic_observability");
    kinematicState.current_model.meta.topology_ref = QStringLiteral("topology_observability");
    kinematicState.current_model.meta.requirement_ref = QStringLiteral("requirement_observability");
    kinematicState.current_model.modeling_mode = QStringLiteral("UNSUPPORTED_MODE");
    if (kinematicStorage.SaveModel(projectRoot, kinematicState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 1;
    }

    DynamicsWidget widget;
    bool telemetryWarning = false;
    QString telemetryMessage;
    QObject::connect(
        &widget,
        &DynamicsWidget::TelemetryStatusGenerated,
        [&telemetryWarning, &telemetryMessage](const QString& message, bool warning) {
            telemetryWarning = warning;
            telemetryMessage = message;
        });

    auto* projectRootEdit = widget.findChild<QLineEdit*>(QStringLiteral("dynamics_project_root_edit"));
    auto* buildButton = widget.findChild<QPushButton*>(QStringLiteral("dynamics_build_from_kinematics_button"));
    auto* runButton = widget.findChild<QPushButton*>(QStringLiteral("dynamics_run_analysis_button"));
    auto* operationLabel = widget.findChild<QLabel*>(QStringLiteral("dynamics_operation_label"));
    if (projectRootEdit == nullptr || buildButton == nullptr || runButton == nullptr || operationLabel == nullptr)
    {
        return 2;
    }

    projectRootEdit->setText(projectRoot);
    buildButton->click();
    runButton->click();

    if (!telemetryWarning ||
        !telemetryMessage.contains(QStringLiteral("Pinocchio")) ||
        !operationLabel->styleSheet().contains(QStringLiteral("#b54708")))
    {
        return 3;
    }

    CapturingLogger logger;
    PinocchioDynamicsBackendAdapter adapter(&logger);
    DynamicModelDto dynamicModel = DynamicModelDto::CreateDefault();
    ParameterizedTrajectoryDto trajectory;
    trajectory.trajectory_id = QStringLiteral("observability_trajectory");
    trajectory.name = QStringLiteral("observability");
    trajectory.samples.push_back({
        0.0,
        std::vector<double>(6, 0.0),
        std::vector<double>(6, 0.0),
        std::vector<double>(6, 0.0)});

    const auto result = adapter.Analyze(kinematicState.current_model, dynamicModel, trajectory);
    if (result.success ||
        result.used_fallback ||
        logger.WarningCount() <= 0 ||
        logger.LastWarningMessage().trimmed().isEmpty() ||
        logger.LastWarningMessage().contains(QStringLiteral("Legacy")))
    {
        return 4;
    }

    return 0;
}
