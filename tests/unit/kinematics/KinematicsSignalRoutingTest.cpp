#include "modules/kinematics/ui/KinematicsWidget.h"
#include "modules/topology/ui/TopologyWidget.h"
#include "modules/dynamics/ui/DynamicsWidget.h"

#include <QApplication>

// ============================================================
// 测试套件 1: KinematicsWidget::StatusChanged 信号 + CanXxx 查询
// ============================================================
static int TestKinematicsStatusSignal()
{
    // 验证最简构造下，StatusChanged 信号在空 widget 上依然可以被连接和发射
    RoboSDP::Kinematics::Ui::KinematicsWidget widget;

    // 使用计数器模拟 QSignalSpy，验证 StatusChanged 被发射
    int signalCount = 0;
    QObject::connect(&widget,
        &RoboSDP::Kinematics::Ui::KinematicsWidget::StatusChanged,
        [&signalCount]() { ++signalCount; });

    // 初始状态下 CanBuildFromTopology 应为 false（无拓扑引用）
    if (widget.CanBuildFromTopology())
    {
        return 1; // 期望 false，但得到 true
    }

    // CanImportUrdf 应始终为 true
    if (!widget.CanImportUrdf())
    {
        return 2; // 期望 true，但得到 false
    }

    // 初始状态下 CanPromoteToDhMaster 应为 false
    if (widget.CanPromoteToDhMaster())
    {
        return 3;
    }

    // 初始状态下 CanSwitchToUrdfMaster 应为 false
    if (widget.CanSwitchToUrdfMaster())
    {
        return 4;
    }

    // 初始状态下 CanRunFk / CanRunIk / CanSampleWorkspace 应为 false（无 links 数据）
    if (widget.CanRunFk() || widget.CanRunIk() || widget.CanSampleWorkspace())
    {
        return 5;
    }

    // 初始状态下 CanSaveDraft 应为 false（无 unsaved changes）
    if (widget.CanSaveDraft())
    {
        return 6;
    }

    // 显式触发 StatusChanged（验证信号连接基础设施正常，可多次发射）
    constexpr int kExpectedSignalCount = 5;
    for (int i = 0; i < kExpectedSignalCount; ++i)
    {
        emit widget.StatusChanged();
    }

    if (signalCount != kExpectedSignalCount)
    {
        return 7; // StatusChanged 信号计数不匹配
    }

    return 0;
}

// ============================================================
// 测试套件 2: TopologyWidget::StatusChanged 信号 + CanXxx 查询
// ============================================================
static int TestTopologyStatusSignal()
{
    RoboSDP::Topology::Ui::TopologyWidget widget;

    int signalCount = 0;
    QObject::connect(&widget,
        &RoboSDP::Topology::Ui::TopologyWidget::StatusChanged,
        [&signalCount]() { ++signalCount; });

    // CanRefreshTemplates 应始终为 true
    if (!widget.CanRefreshTemplates())
    {
        return 1;
    }

    // 初始状态下 CanGenerate 应为 false（无选中模板）
    if (widget.CanGenerate())
    {
        return 2;
    }

    // 初始状态下 CanValidate 应为 false（无构型名称）
    if (widget.CanValidate())
    {
        return 3;
    }

    // 初始状态下 CanSaveDraft 应为 false
    if (widget.CanSaveDraft())
    {
        return 4;
    }

    // 验证信号可多次发射
    constexpr int kExpectedSignalCount = 3;
    for (int i = 0; i < kExpectedSignalCount; ++i)
    {
        emit widget.StatusChanged();
    }

    if (signalCount != kExpectedSignalCount)
    {
        return 5;
    }

    return 0;
}

// ============================================================
// 测试套件 3: DynamicsWidget::StatusChanged 信号 + CanXxx 查询
// ============================================================
static int TestDynamicsStatusSignal()
{
    RoboSDP::Dynamics::Ui::DynamicsWidget widget;

    int signalCount = 0;
    QObject::connect(&widget,
        &RoboSDP::Dynamics::Ui::DynamicsWidget::StatusChanged,
        [&signalCount]() { ++signalCount; });

    // 初始状态下 CanBuildFromKinematics 应为 false（无运动学引用）
    if (widget.CanBuildFromKinematics())
    {
        return 1;
    }

    // 初始状态下 CanRunAnalysis 应为 false（无连杆表数据）
    if (widget.CanRunAnalysis())
    {
        return 2;
    }

    // 初始状态下 CanSaveDraft 应为 false
    if (widget.CanSaveDraft())
    {
        return 3;
    }

    // 验证信号可多次发射
    constexpr int kExpectedSignalCount = 3;
    for (int i = 0; i < kExpectedSignalCount; ++i)
    {
        emit widget.StatusChanged();
    }

    if (signalCount != kExpectedSignalCount)
    {
        return 4;
    }

    return 0;
}

// ============================================================
// 主入口
// ============================================================
int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    int result;

    result = TestKinematicsStatusSignal();
    if (result != 0)
    {
        return 100 + result; // 套件 1 错误码 101~107
    }

    result = TestTopologyStatusSignal();
    if (result != 0)
    {
        return 200 + result; // 套件 2 错误码 201~205
    }

    result = TestDynamicsStatusSignal();
    if (result != 0)
    {
        return 300 + result; // 套件 3 错误码 301~304
    }

    return 0; // 全部通过
}
