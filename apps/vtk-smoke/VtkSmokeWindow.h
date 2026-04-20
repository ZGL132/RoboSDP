#pragma once

#include <QMainWindow>

class QLabel;
class QVBoxLayout;
class QWidget;

namespace RoboSDP::VtkSmoke
{

/**
 * @brief 最小 Qt + VTK 验证窗口。
 *
 * 该窗口只负责承载一个独立的 QVTKOpenGLNativeWidget 验证入口，
 * 用于确认当前机器和当前 Qt 工具链是否已经具备最小三维显示能力。
 */
class VtkSmokeWindow : public QMainWindow
{
public:
    explicit VtkSmokeWindow(QWidget* parent = nullptr);
    ~VtkSmokeWindow() override = default;

private:
    /// 创建窗口中的最小布局骨架，不接入任何业务模块。
    void BuildLayout();

    /// 根据当前构建环境创建真实 VTK 视图或 fallback 提示。
    void BuildScene();

    /// 当未检测到可用 VTK 依赖时，显示明确的依赖说明。
    void BuildFallbackView();

private:
    QWidget* m_centralWidget = nullptr;
    QVBoxLayout* m_layout = nullptr;
    QLabel* m_statusLabel = nullptr;
};

} // namespace RoboSDP::VtkSmoke
