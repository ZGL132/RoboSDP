#pragma once

#include <QWidget>

class QLabel;
class QPushButton;
class QVBoxLayout;

namespace RoboSDP::Desktop
{

/**
 * @brief 最小 VTK 场景承载控件。
 *
 * 本轮只负责在 Qt 主窗口中央区域提供最小三维承载容器，
 * 若检测到 VTK 依赖则显示基础示例对象，否则显示明确的缺依赖提示。
 */
class VtkSceneWidget : public QWidget
{
public:
    explicit VtkSceneWidget(QWidget* parent = nullptr);
    ~VtkSceneWidget() override = default;

private:
    /// 构建中央三维视图的最小工具栏区域，并挂接刷新动作。
    void BuildToolbar();

    /// 根据当前构建环境重建最小三维场景或 fallback 占位说明。
    void RefreshScene();

    /// 清理旧的场景承载控件，保证刷新时不会残留旧对象。
    void ClearSceneContent();

    /// 在检测到 VTK 依赖时构建最小示例场景。
    void BuildScene();

    /// 在未接入 VTK 依赖时提供稳定的中央占位说明。
    void BuildFallbackView();

private:
    QVBoxLayout* m_layout = nullptr;
    QVBoxLayout* m_contentLayout = nullptr;
    QPushButton* m_refreshButton = nullptr;
    QLabel* m_statusLabel = nullptr;
};

} // namespace RoboSDP::Desktop
