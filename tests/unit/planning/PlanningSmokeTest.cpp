#include "core/repository/LocalJsonRepository.h"
#include "modules/kinematics/persistence/KinematicJsonStorage.h"
#include "modules/planning/persistence/PlanningJsonStorage.h"
#include "modules/planning/service/PlanningVerificationService.h"
#include "modules/selection/persistence/SelectionJsonStorage.h"

#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QHostAddress>
#include <QProcess>
#include <QProcessEnvironment>
#include <QTextStream>
#include <QStandardPaths>
#include <QTcpServer>

namespace
{

QString FindPythonExecutable()
{
    const QStringList candidates {
        QStringLiteral("python.exe"),
        QStringLiteral("python"),
        QStringLiteral("py.exe"),
        QStringLiteral("py"),
    };

    for (const auto& candidate : candidates)
    {
        const QString executable = QStandardPaths::findExecutable(candidate);
        if (!executable.isEmpty())
        {
            return executable;
        }
    }

    return {};
}

bool WaitForServerReady(QProcess& process, QString* output)
{
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < 8000)
    {
        process.waitForReadyRead(200);
        *output += QString::fromUtf8(process.readAll());
        if (output->contains(QStringLiteral("Planning gRPC server started")))
        {
            return true;
        }

        if (process.state() == QProcess::NotRunning)
        {
            return false;
        }
    }

    *output += QString::fromUtf8(process.readAll());
    return output->contains(QStringLiteral("Planning gRPC server started"));
}

void StopServer(QProcess& process)
{
    if (process.state() == QProcess::NotRunning)
    {
        return;
    }

    process.terminate();
    if (!process.waitForFinished(3000))
    {
        process.kill();
        process.waitForFinished(1000);
    }
}

int AcquireFreeLocalPort()
{
    QTcpServer probeServer;
    if (!probeServer.listen(QHostAddress::LocalHost, 0))
    {
        return -1;
    }

    const int port = static_cast<int>(probeServer.serverPort());
    probeServer.close();
    return port;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    const QString projectRoot =
        QDir::current().absoluteFilePath(QStringLiteral("planning-smoke-project"));

    RoboSDP::Repository::LocalJsonRepository repository;
    RoboSDP::Kinematics::Persistence::KinematicJsonStorage kinematicStorage(repository);
    RoboSDP::Selection::Persistence::SelectionJsonStorage selectionStorage(repository);
    RoboSDP::Planning::Persistence::PlanningJsonStorage planningStorage(repository);

    RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto kinematicState =
        RoboSDP::Kinematics::Dto::KinematicsWorkspaceStateDto::CreateDefault();
    kinematicState.current_model.meta.kinematic_id = QStringLiteral("kinematic_planning_smoke");
    kinematicState.current_model.meta.topology_ref = QStringLiteral("topology_planning_smoke");
    kinematicState.current_model.meta.requirement_ref = QStringLiteral("requirement_planning_smoke");
    if (kinematicStorage.SaveModel(projectRoot, kinematicState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 1;
    }

    RoboSDP::Selection::Dto::SelectionWorkspaceStateDto selectionState =
        RoboSDP::Selection::Dto::SelectionWorkspaceStateDto::CreateDefault();
    selectionState.drive_train_result.dynamic_ref = QStringLiteral("dynamic_planning_smoke");
    selectionState.drive_train_result.kinematic_ref = kinematicState.current_model.meta.kinematic_id;
    selectionState.drive_train_result.topology_ref = kinematicState.current_model.meta.topology_ref;
    selectionState.drive_train_result.requirement_ref = kinematicState.current_model.meta.requirement_ref;
    selectionState.drive_train_result.success = true;
    selectionState.drive_train_result.message = QStringLiteral("selection smoke");
    if (selectionStorage.Save(projectRoot, selectionState) != RoboSDP::Errors::ErrorCode::Ok)
    {
        return 2;
    }

    RoboSDP::Planning::Service::PlanningVerificationService planningService(
        planningStorage,
        kinematicStorage,
        selectionStorage,
        nullptr);

    const QString pythonExecutable = FindPythonExecutable();
    if (pythonExecutable.isEmpty())
    {
        return 7;
    }

    // 中文说明：planning smoke 以前固定绑定 50071 端口，容易被本机残留进程占用而产生随机失败。
    // 本轮改为动态申请空闲本地端口，避免测试环境抖动影响规划主链回归。
    const int grpcPort = AcquireFreeLocalPort();
    if (grpcPort <= 0)
    {
        return 9;
    }

    const QString serverScript = QDir::current().absoluteFilePath(
        QStringLiteral("services/planning-grpc/server/planning_server.py"));
    QProcess serverProcess;
    serverProcess.setProcessChannelMode(QProcess::MergedChannels);
    QProcessEnvironment environment = QProcessEnvironment::systemEnvironment();
    environment.insert(QStringLiteral("PYTHONUNBUFFERED"), QStringLiteral("1"));
    serverProcess.setProcessEnvironment(environment);
    serverProcess.setProgram(pythonExecutable);
    serverProcess.setArguments({
        serverScript,
        QStringLiteral("--host"),
        QStringLiteral("127.0.0.1"),
        QStringLiteral("--port"),
        QString::number(grpcPort),
    });
    serverProcess.start();

    QString serverOutput;
    if (!WaitForServerReady(serverProcess, &serverOutput))
    {
        StopServer(serverProcess);
        return 8;
    }

    auto buildResult = planningService.BuildPlanningScene(projectRoot);
    if (!buildResult.IsSuccess())
    {
        StopServer(serverProcess);
        return 3;
    }
    buildResult.state.current_scene.planning_config.service_endpoint =
        QStringLiteral("127.0.0.1:%1").arg(grpcPort);

    const auto runResult = planningService.RunPointToPointVerification(projectRoot, buildResult.state);
    if (!runResult.IsSuccess() ||
        runResult.state.requests.empty() ||
        runResult.state.results.empty() ||
        !runResult.state.results.back().success)
    {
        QTextStream(stderr)
            << "[planning_smoke] run failed\n"
            << "service_message=" << runResult.message << "\n"
            << "build_message=" << buildResult.message << "\n"
            << "server_output=" << serverOutput << "\n";
        if (!runResult.state.results.empty())
        {
            const auto& latestResult = runResult.state.results.back();
            QTextStream(stderr)
                << "verification_message=" << latestResult.message << "\n"
                << "verification_success=" << latestResult.success << "\n";
            if (!latestResult.trajectory_results.empty())
            {
                const auto& latestTrajectory = latestResult.trajectory_results.back();
                QTextStream(stderr)
                    << "trajectory_success=" << latestTrajectory.success << "\n"
                    << "trajectory_message=" << latestTrajectory.message << "\n";
            }
        }
        StopServer(serverProcess);
        return 4;
    }

    const auto saveResult = planningService.SaveDraft(projectRoot, runResult.state);
    if (!saveResult.IsSuccess())
    {
        StopServer(serverProcess);
        return 5;
    }

    const auto loadResult = planningService.LoadDraft(projectRoot);
    if (!loadResult.IsSuccess() ||
        loadResult.state.requests.empty() ||
        loadResult.state.results.empty())
    {
        StopServer(serverProcess);
        return 6;
    }

    StopServer(serverProcess);
    return 0;
}
