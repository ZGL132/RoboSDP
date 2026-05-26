#pragma once

#include <QWidget>

class QLabel;
class QToolButton;
class QVBoxLayout;

/**
 * @brief 可折叠面板组件（Accordion Section）
 *
 * 用于右侧属性面板中对参数进行分组折叠，使页面更紧凑。
 * 包含一个点击切换的标题栏和一个内容区域。
 * 内容区域可以是任意 QWidget（例如 QFormLayout 的容器）。
 */
class CollapsibleSectionWidget : public QWidget
{
    Q_OBJECT

public:
    /// @brief 构造折叠面板
    /// @param title 折叠面板标题
    /// @param parent 父控件
    explicit CollapsibleSectionWidget(const QString& title, QWidget* parent = nullptr);

    /// @brief 设置内容控件
    /// @param contentWidget 任意 QWidget 作为折叠区域的内容
    void SetContent(QWidget* contentWidget);

    /// @brief 展开/折叠
    void SetCollapsed(bool collapsed);
    bool IsCollapsed() const { return m_collapsed; }

    /// @brief 切换展开/折叠
    void Toggle();

signals:
    /// @brief 展开/折叠状态变化时发出
    void CollapsedChanged(bool collapsed);

private:
    void OnToggleClicked();

    QToolButton* m_toggle_button = nullptr;   ///< 标题栏按钮（点击切换展开/折叠）
    QWidget* m_content_widget = nullptr;       ///< 内容区域
    QVBoxLayout* m_layout = nullptr;           ///< 主布局
    bool m_collapsed = false;                  ///< 当前是否折叠
};
