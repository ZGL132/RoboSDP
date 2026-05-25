#include "apps/desktop-qt/widgets/vtk/RobotVtkView.h"
#include "apps/desktop-qt/widgets/vtk/VtkSceneBuilder.h"

#include <QCheckBox>
#include <QCursor>
#include <QFrame>
#include <QGraphicsOpacityEffect>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QPixmap>
#include <QResizeEvent>
#include <QSignalBlocker>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#if defined(ROBOSDP_HAVE_VTK)
#include <QVTKOpenGLNativeWidget.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkBoxWidget2.h>
#include <vtkBoxRepresentation.h>
#include <vtkCamera.h>
#include <vtkCaptionActor2D.h>
#include <vtkGlyph3D.h>
#include <vtkLineSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkSphereSource.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkCoordinate.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNew.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkProperty.h>
#include <vtkPropPicker.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTextProperty.h>
#include <vtkTextActor.h>
#include <vtkTransform.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>

namespace
{

constexpr auto kLayerRequirementWorkspace = "requirement.workspace";
constexpr auto kLayerRequirementKeyPoses = "requirement.key_poses";
constexpr auto kLayerKinematicsWorkspace = "kinematics.workspace";
constexpr auto kLayerKinematicsSingularity = "kinematics.singularity";
constexpr auto kLayerKinematicsIkCompare = "kinematics.ik_compare";
constexpr auto kLayerRequirementWorkspaceName = u8"需求工作空间";
constexpr auto kLayerRequirementKeyPosesName = u8"关键工位";
constexpr auto kLayerKinematicsWorkspaceName = u8"运动学工作空间";
constexpr auto kLayerKinematicsSingularityName = u8"奇异性空间";
constexpr auto kLayerKinematicsIkCompareName = u8"IK 对比";

struct AnalysisLayerUiSpec
{
    const char* id;
    const char* name;
    const char* color;
    bool defaultVisible;
};

const std::array<AnalysisLayerUiSpec, 5> kAnalysisLayerSpecs {{
    {kLayerRequirementWorkspace, kLayerRequirementWorkspaceName, "#3b82f6", false},
    {kLayerRequirementKeyPoses, kLayerRequirementKeyPosesName, "#f59e0b", false},
    {kLayerKinematicsWorkspace, kLayerKinematicsWorkspaceName, "#22c55e", false},
    {kLayerKinematicsSingularity, kLayerKinematicsSingularityName, "#ef4444", false},
    {kLayerKinematicsIkCompare, kLayerKinematicsIkCompareName, "#06b6d4", true}
}};

constexpr double kAnalysisLayerOverlayOpacity = 0.6;

const AnalysisLayerUiSpec* FindLayerSpec(const QString& layerId)
{
    for (const auto& spec : kAnalysisLayerSpecs)
    {
        if (layerId == QString::fromLatin1(spec.id))
        {
            return &spec;
        }
    }
    return nullptr;
}

QIcon MakeCameraToolbarIcon(const QString& kind)
{
    QPixmap pixmap(48, 48);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    QPen pen(QColor(248, 250, 252, 220), 3.6);
    pen.setCapStyle(Qt::RoundCap);
    pen.setJoinStyle(Qt::RoundJoin);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);

    if (kind == QStringLiteral("zoom_in") || kind == QStringLiteral("zoom_out"))
    {
        painter.drawEllipse(QPointF(19.0, 19.0), 10.0, 10.0);
        painter.drawLine(QPointF(26.4, 26.4), QPointF(36.0, 36.0));
        painter.drawLine(QPointF(14.0, 19.0), QPointF(24.0, 19.0));
        if (kind == QStringLiteral("zoom_in"))
        {
            painter.drawLine(QPointF(19.0, 14.0), QPointF(19.0, 24.0));
        }
    }
    else if (kind == QStringLiteral("reset"))
    {
        painter.drawArc(QRectF(10.0, 10.0, 28.0, 28.0), 35 * 16, 280 * 16);
        painter.drawLine(QPointF(12.4, 12.0), QPointF(10.0, 22.0));
        painter.drawLine(QPointF(12.4, 12.0), QPointF(22.0, 12.4));
    }
    else if (kind == QStringLiteral("front"))
    {
        painter.drawRect(QRectF(12.0, 12.0, 24.0, 24.0));
        painter.drawLine(QPointF(18.0, 12.0), QPointF(18.0, 36.0));
        painter.drawLine(QPointF(30.0, 12.0), QPointF(30.0, 36.0));
    }
    else if (kind == QStringLiteral("side"))
    {
        painter.drawRect(QRectF(16.0, 10.0, 16.0, 28.0));
        painter.drawLine(QPointF(16.0, 16.0), QPointF(32.0, 16.0));
        painter.drawLine(QPointF(16.0, 32.0), QPointF(32.0, 32.0));
    }
    else if (kind == QStringLiteral("top"))
    {
        painter.drawEllipse(QPointF(24.0, 24.0), 13.0, 13.0);
        painter.drawLine(QPointF(24.0, 11.0), QPointF(24.0, 37.0));
        painter.drawLine(QPointF(11.0, 24.0), QPointF(37.0, 24.0));
    }
    else if (kind == QStringLiteral("iso"))
    {
        QPolygonF top;
        top << QPointF(24.0, 9.0) << QPointF(36.0, 16.0) << QPointF(24.0, 23.0) << QPointF(12.0, 16.0);
        QPolygonF left;
        left << QPointF(12.0, 16.0) << QPointF(24.0, 23.0) << QPointF(24.0, 38.0) << QPointF(12.0, 31.0);
        QPolygonF right;
        right << QPointF(36.0, 16.0) << QPointF(24.0, 23.0) << QPointF(24.0, 38.0) << QPointF(36.0, 31.0);
        painter.drawPolygon(top);
        painter.drawPolygon(left);
        painter.drawPolygon(right);
    }

    return QIcon(pixmap);
}

#if defined(ROBOSDP_HAVE_VTK)
vtkSmartPointer<vtkTransform> BuildPoseTransform(
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    auto transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate(pose.position_m[0], pose.position_m[1], pose.position_m[2]);
    transform->RotateZ(pose.rpy_deg[2]);
    transform->RotateY(pose.rpy_deg[1]);
    transform->RotateX(pose.rpy_deg[0]);
    return transform;
}

RoboSDP::Kinematics::Dto::CartesianPoseDto ToCartesianPose(
    const RoboSDP::Requirement::Dto::RequirementKeyPoseDto& keyPose)
{
    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = {keyPose.pose[0], keyPose.pose[1], keyPose.pose[2]};
    pose.rpy_deg = {keyPose.pose[3], keyPose.pose[4], keyPose.pose[5]};
    return pose;
}

void TuneAxesCaption(vtkCaptionActor2D* captionActor, double red, double green, double blue)
{
    if (captionActor == nullptr || captionActor->GetCaptionTextProperty() == nullptr)
    {
        return;
    }
    if (captionActor->GetTextActor() != nullptr)
    {
        captionActor->GetTextActor()->SetTextScaleModeToNone();
    }
    captionActor->BorderOff();
    captionActor->LeaderOff();
    captionActor->GetCaptionTextProperty()->SetFontSize(12);
    captionActor->GetCaptionTextProperty()->SetBold(true);
    captionActor->GetCaptionTextProperty()->SetColor(red, green, blue);
}

vtkSmartPointer<vtkAxesActor> CreateTcpComparisonAxes(double length)
{
    auto axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetTotalLength(length, length, length);
    axes->SetNormalizedShaftLength(0.8, 0.8, 0.8);
    axes->SetNormalizedTipLength(0.2, 0.2, 0.2);
    axes->GetXAxisShaftProperty()->SetColor(1.0, 0.0, 0.0);
    axes->GetXAxisTipProperty()->SetColor(1.0, 0.0, 0.0);
    axes->GetYAxisShaftProperty()->SetColor(1.0, 1.0, 0.0);
    axes->GetYAxisTipProperty()->SetColor(1.0, 1.0, 0.0);
    axes->GetZAxisShaftProperty()->SetColor(0.0, 0.8, 0.0);
    axes->GetZAxisTipProperty()->SetColor(0.0, 0.8, 0.0);
    TuneAxesCaption(axes->GetXAxisCaptionActor2D(), 1.0, 0.0, 0.0);
    TuneAxesCaption(axes->GetYAxisCaptionActor2D(), 1.0, 1.0, 0.0);
    TuneAxesCaption(axes->GetZAxisCaptionActor2D(), 0.0, 0.8, 0.0);
    return axes;
}

vtkSmartPointer<vtkBillboardTextActor3D> CreateBillboardLabel(
    const QString& text,
    const std::array<double, 3>& position,
    double red,
    double green,
    double blue,
    int offsetX,
    int offsetY)
{
    auto label = vtkSmartPointer<vtkBillboardTextActor3D>::New();
    label->SetInput(text.toStdString().c_str());
    label->SetPosition(position[0], position[1], position[2]);
    label->SetDisplayOffset(offsetX, offsetY);
    vtkTextProperty* textProp = label->GetTextProperty();
    if (textProp != nullptr)
    {
        textProp->SetFontSize(13);
        textProp->SetColor(red, green, blue);
        textProp->SetBold(true);
        textProp->SetShadow(true);
        textProp->SetShadowOffset(1, -1);
    }
    return label;
}

vtkSmartPointer<vtkActor> CreatePointCloudActor(
    const std::vector<std::array<double, 3>>& positions,
    double red,
    double green,
    double blue,
    double opacity,
    double pointSize)
{
    vtkNew<vtkPoints> points;
    points->SetNumberOfPoints(static_cast<vtkIdType>(positions.size()));
    for (vtkIdType i = 0; i < static_cast<vtkIdType>(positions.size()); ++i)
    {
        const auto& point = positions[static_cast<std::size_t>(i)];
        points->SetPoint(i, point[0], point[1], point[2]);
    }

    vtkNew<vtkPolyData> polyData;
    polyData->SetPoints(points);

    vtkNew<vtkVertexGlyphFilter> glyphFilter;
    glyphFilter->SetInputData(polyData);
    glyphFilter->Update();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(glyphFilter->GetOutputPort());

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(red, green, blue);
    actor->GetProperty()->SetOpacity(opacity);
    actor->GetProperty()->SetPointSize(pointSize);
    actor->GetProperty()->SetLighting(false);
    return actor;
}

vtkSmartPointer<vtkActor> CreateClassifiedPointCloudActor(
    const std::vector<std::array<double, 3>>& positions,
    const std::vector<bool>& isSingular)
{
    vtkNew<vtkPoints> points;
    points->SetNumberOfPoints(static_cast<vtkIdType>(positions.size()));
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetNumberOfComponents(3);
    colors->SetName("AnalysisLayerColors");

    for (vtkIdType i = 0; i < static_cast<vtkIdType>(positions.size()); ++i)
    {
        const auto& point = positions[static_cast<std::size_t>(i)];
        points->SetPoint(i, point[0], point[1], point[2]);
        if (isSingular[static_cast<std::size_t>(i)])
        {
            colors->InsertNextTuple3(239, 68, 68);
        }
        else
        {
            colors->InsertNextTuple3(34, 197, 94);
        }
    }

    vtkNew<vtkPolyData> polyData;
    polyData->SetPoints(points);
    polyData->GetPointData()->SetScalars(colors);

    vtkNew<vtkVertexGlyphFilter> glyphFilter;
    glyphFilter->SetInputData(polyData);
    glyphFilter->Update();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(glyphFilter->GetOutputPort());
    mapper->ScalarVisibilityOn();
    mapper->SetScalarModeToUsePointData();

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetOpacity(0.78);
    actor->GetProperty()->SetPointSize(3.5);
    actor->GetProperty()->SetLighting(false);
    return actor;
}
#endif

// 中文说明：QPainter 绘制的地图风格比例尺覆盖层，白色刻度线与标签，置于三维视图底部。
class ScaleBarOverlay : public QWidget
{
public:
    explicit ScaleBarOverlay(QWidget* parent)
        : QWidget(parent)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_TranslucentBackground);
        setAttribute(Qt::WA_NoSystemBackground);
        setStyleSheet(QStringLiteral("background:transparent;"));
        setFixedHeight(40);
    }

    void setScale(double niceDistMeters, const QString& unit, double barRatio)
    {
        m_niceDist = niceDistMeters;
        m_unit = unit;
        m_barRatio = barRatio;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        if (width() < 60 || height() < 8)
            return;

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        const int w = width();
        const int h = height();
        const int barY = h - 8;        // 主刻度线 y 坐标（底部）
        const int tickTop = barY - 8;  // 短刻度顶部
        const int tickMid = barY - 6;  // 中刻度顶部
        const int textY = 1;           // 文字顶部 y（靠上绘制，避免与刻度线重叠）

        const int margin = 4;
        const int barWidth = w - 2 * margin;
        const int barLeft = margin;
        const int barRight = margin + barWidth;

        // 根据 barRatio 决定分段数（1~5 段）
        int segments = 1;
        if (m_barRatio >= 2.5) segments = 5;
        else if (m_barRatio >= 2.0) segments = 4;
        else if (m_barRatio >= 1.5) segments = 3;
        else if (m_barRatio >= 0.8) segments = 2;

        const double segWidth = static_cast<double>(barWidth) / segments;
        const double segDist = m_niceDist / segments;

        // ── 白色画笔 ──
        QPen linePen(QColor(230, 230, 235), 1.5);
        painter.setPen(linePen);

        // 主水平线
        painter.drawLine(QPointF(barLeft, barY), QPointF(barRight, barY));

        // 左端竖线
        painter.drawLine(QPointF(barLeft, tickTop), QPointF(barLeft, barY));
        // 右端竖线
        painter.drawLine(QPointF(barRight, tickTop), QPointF(barRight, barY));

        // 分段刻度和填充
        QColor fillA(255, 255, 255, 140);
        QColor fillB(255, 255, 255, 45);
        for (int i = 0; i < segments; ++i)
        {
            double segLeft = barLeft + i * segWidth;
            double segRight = segLeft + segWidth;

            // 交替填充
            if (segments > 1)
            {
                painter.fillRect(QRectF(segLeft, barY - 4, segWidth, 5.0),
                                 (i % 2 == 0) ? fillA : fillB);
            }

            // 中间刻度线（在每段起始处）
            if (i > 0)
            {
                painter.drawLine(QPointF(segLeft, tickMid), QPointF(segLeft, barY));
            }

            // 细分小刻度（每段内再分 2 或 5 小格）
            int subTicks = (segments <= 3) ? 5 : 2;
            for (int j = 1; j < subTicks; ++j)
            {
                double subX = segLeft + j * segWidth / subTicks;
                painter.drawLine(QPointF(subX, barY - 4), QPointF(subX, barY));
            }

            // 最后一段末尾的小刻度（在 barRight 处）
            if (i == segments - 1 && segments > 1)
            {
                for (int j = 1; j < subTicks; ++j)
                {
                    double subX = segRight - (subTicks - j) * segWidth / subTicks;
                    // 已通过 i>0 处理了 segLeft（即上一段的末尾）
                    // 这里不需要重复
                }
            }
        }

        // ── 文字标签 ──
        painter.setPen(QPen(QColor(220, 222, 228), 1.0));
        QFont font(QStringLiteral("Consolas"), 5);
        font.setStyleHint(QFont::Monospace);
        painter.setFont(font);

        const QFontMetrics fm(font);
        // 只在分段端点绘制标签
        for (int i = 0; i <= segments; ++i)
        {
            double val = i * segDist;
            double labelX = barLeft + i * segWidth;

            QString label;
            if (m_unit == QStringLiteral("m"))
            {
                if (val >= 1.0)
                    label = QStringLiteral("%1").arg(val, 0, 'g', 3);
                else
                    label = QStringLiteral("%1").arg(val, 0, 'f', 2);
            }
            else
            {
                label = QStringLiteral("%1").arg(val, 0, 'g', 3);
            }

            // 最右端标签附加单位
            if (i == segments)
            {
                label = label + QStringLiteral(" ") + m_unit;
            }

            int textWidth = fm.horizontalAdvance(label);
            double textX = labelX - textWidth / 2.0;
            // 避免文字溢出左边界
            if (i == 0)
                textX = std::max(0.0, labelX - 2.0);
            // 避免文字溢出右边界
            if (i == segments)
                textX = std::min(static_cast<double>(w - textWidth - 2), labelX - textWidth / 2.0);

            painter.drawText(QPointF(textX, textY + fm.ascent()), label);
        }

    }

private:
    double m_niceDist = 0.5;
    QString m_unit = QStringLiteral("m");
    double m_barRatio = 1.0;
};

} // namespace

/// @brief 自定义 VTK 鼠标交互器，支持左键点击拾取 3D Actor、滚轮关节驱动、虚线框选等。
class VtkClickInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    /// @brief 工厂方法（手动实现而非 vtkStandardNewMacro，兼容 MinGW）。
    static VtkClickInteractorStyle* New()
    {
        auto* ret = new VtkClickInteractorStyle;
        ret->InitializeObjectBase();
        return ret;
    }

    vtkTypeMacro(VtkClickInteractorStyle, vtkInteractorStyleTrackballCamera);

    /// @brief 指向所属的 RobotVtkView，用于回调 HandleActorClicked。
    RoboSDP::Desktop::Vtk::RobotVtkView* parentView = nullptr;
    /// @brief 当前渲染器，用于 vtkPropPicker 拾取。
    vtkRenderer* renderer = nullptr;

    void OnLeftButtonDown() override
    {
        // 中文说明：左键按下时先执行拾取；若命中 Actor 则只高亮不旋转视角；若未命中则保留默认相机操作。
        bool pickedActor = false;
        if (renderer != nullptr && parentView != nullptr && this->GetInteractor() != nullptr)
        {
            const int* clickPos = this->GetInteractor()->GetEventPosition();
            vtkNew<vtkPropPicker> picker;
            picker->Pick(clickPos[0], clickPos[1], 0, renderer);

            vtkActor* clickedActor = picker->GetActor();
            // 中文说明：传递拾取位置（世界坐标），供 HandleActorClicked 在骨架节点查找时使用。
            const double* pickPos = picker->GetPickPosition();
            parentView->HandleActorClicked(clickedActor, pickPos);
            pickedActor = (clickedActor != nullptr);
        }
        // 中文说明：仅未命中 Actor 时才保留默认的视角旋转/平移交互，避免点击标签导致相机偏移。
        if (!pickedActor)
        {
            vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
        }
    }

    // ── 【逆向驱动】鼠标滚轮事件重写 ──────────────────────────────
    /// @brief 滚轮向前滚动：如果已选中连杆，则发射关节正转信号，否则保留默认视角缩放。
    ///        步长由 parentView->GetScrollStep() 控制，默认 1.0°。
    void OnMouseWheelForward() override
    {
        if (parentView != nullptr && !parentView->GetPickedLinkName().isEmpty())
        {
            parentView->EmitJointScroll(parentView->GetScrollStep()); // 正转 +step 度
            return; // 不传递给基类，避免视角缩放干扰
        }
        vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
    }

    /// @brief 滚轮向后滚动：如果已选中连杆，则发射关节反转信号。
    ///        步长由 parentView->GetScrollStep() 控制，默认 -1.0°。
    void OnMouseWheelBackward() override
    {
        if (parentView != nullptr && !parentView->GetPickedLinkName().isEmpty())
        {
            parentView->EmitJointScroll(-parentView->GetScrollStep()); // 反转 -step 度
            return;
        }
        vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
    }
};

#endif

namespace RoboSDP::Desktop::Vtk
{

RobotVtkView::RobotVtkView(QWidget* parent)
    : QWidget(parent)
{
    BuildLayout();
}

RobotVtkView::~RobotVtkView()
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_corner_axes_widget != nullptr)
    {
        // 中文说明：角落坐标轴绑定了 VTK interactor，销毁视图前先禁用，避免关闭窗口时访问失效对象。
        m_corner_axes_widget->SetEnabled(0);
    }
#endif
}

// RobotVtkView.cpp

void RobotVtkView::ShowPreviewScene(const PreviewSceneDto& scene, bool resetCamera)
{
    // 清理旧的 Actor 缓存（如 Mesh），但不一定会重置相机
    ClearCache();
    ClearIkPoseComparison();
    m_currentScene = scene;

    // 【修复】场景数据更新后，重建 link_name → joint_index 映射表，
    // 供 EmitJointScroll 在滚轮驱动时快速查找关节索引。
    RebuildLinkToJointMap();

    // 将 resetCamera 参数透传给实际的刷新函数
    RefreshScene(resetCamera);
}

void RobotVtkView::ShowIkPoseComparison(
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& targetPose,
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& actualPose,
    double positionErrorMm,
    double orientationErrorDeg,
    bool withinTolerance,
    bool hasActualPose)
{
    m_ikTargetPose = targetPose;
    m_ikActualPose = actualPose;
    m_ikPositionErrorMm = positionErrorMm;
    m_ikOrientationErrorDeg = orientationErrorDeg;
    m_ikWithinTolerance = withinTolerance;
    m_hasIkPoseComparison = true;
    m_hasIkActualPose = hasActualPose;
    SetAnalysisLayerVisible(QString::fromLatin1(kLayerKinematicsIkCompare), true);
    RenderIkPoseComparisonLayer();
}

void RobotVtkView::ClearIkPoseComparison()
{
    m_hasIkPoseComparison = false;
    m_hasIkActualPose = false;
    m_ikWithinTolerance = false;
    m_ikPositionErrorMm = 0.0;
    m_ikOrientationErrorDeg = 0.0;
    ClearIkPoseComparisonActors();
    // IK 对比清除后恢复 FK 实时 TCP 坐标指示器
    if (m_tcp_axes_actor != nullptr && !m_currentScene.IsEmpty())
    {
        m_tcp_axes_actor->SetVisibility(true);
    }
    SetAnalysisLayerVisible(QString::fromLatin1(kLayerKinematicsIkCompare), false);
#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderWindow != nullptr)
    {
        m_renderWindow->Render();
    }
#endif
}

void RobotVtkView::UpdatePreviewPoses(
    const std::map<QString, RoboSDP::Kinematics::Dto::CartesianPoseDto>& linkWorldPoses)
{
    // 1. 数据非空校验
    // 如果传入的姿态字典为空，说明没有新的运动学解算结果，直接返回，避免无意义的计算。
    if (linkWorldPoses.empty())
    {
        return;
    }

    // =========================================================================
    // 第一阶段：更新内存中的轻量级数据 (DTO 同步)
    // =========================================================================

    // 2. 更新节点 (Nodes) 数据
    // 中文说明：同步更新内存中的轻量场景 DTO，避免后续标签开关触发完整重绘时回到旧姿态。
    // 解释：m_currentScene 缓存了当前 3D 视图显示的所有数据。
    // 当拖动滑块时，机械臂姿态改变，我们必须把新的姿态覆盖回 m_currentScene，
    // 这样如果用户稍后点击了“显示/隐藏网格”按钮触发了重绘，画面不会错误地“跳回”到拖动前的样子。
    for (auto& node : m_currentScene.nodes)
    {
        // 去字典里找这个节点（比如 "J2" 或 "Link_1"）的新姿态
        const auto poseIt = linkWorldPoses.find(node.link_name);
        if (poseIt == linkWorldPoses.end())
        {
            continue; // 如果没找到，保持原样
        }

        // 覆盖为最新姿态（包含位置和欧拉角）
        node.world_pose = poseIt->second;
        // 单独把平移位置再存一份，方便后续画图使用
        node.position_m = poseIt->second.position_m;
    }

    // 3. 定义一个辅助读取函数 (Lambda)
    // 解释：这个函数用于安全地从 linkWorldPoses 字典里提取指定节点的三维坐标 (XYZ)。
    // 如果节点不在字典里（比如静态基座），就原样返回旧坐标 (fallbackPosition)，保证程序不崩溃。
    auto readPosition = [&linkWorldPoses](
                            const QString& linkName,
                            const std::array<double, 3>& fallbackPosition) {
        const auto poseIt = linkWorldPoses.find(linkName);
        return poseIt != linkWorldPoses.end() ? poseIt->second.position_m : fallbackPosition;
    };

    // 4. 更新连杆线段 (Segments) 数据
    for (auto& segment : m_currentScene.segments)
    {
        // 中文说明：骨架线段必须和 link world pose 同步更新，否则 FK 预览时 mesh 会动而骨架停在零位。
        // 解释：连杆在三维视图里是一根线（或圆柱）。线的两端分别是父节点和子节点。
        // 既然节点的位置已经在上面第 2 步更新了，这根线的两端坐标也必须跟着更新。
        // 否则会出现极其诡异的画面：三维模型（Mesh）在跟着滑块转动，但里面的骨架线条却停在原地。
        segment.start_position_m = readPosition(segment.parent_link_name, segment.start_position_m);
        segment.end_position_m = readPosition(segment.child_link_name, segment.end_position_m);
    }

    // =========================================================================
    // 第二阶段：触发 VTK 引擎重新渲染 (画面刷新)
    // =========================================================================

#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer != nullptr)
    {
        // 动态路径重建轻量骨架/标签层，但复用 Mesh Actor 缓存，并且不重置相机。
        RefreshScene(false);

        // 更新 TCP 坐标系指示器位姿（如果 tcp_frame 在 poseMap 中）
        const auto tcpPoseIt = linkWorldPoses.find(QStringLiteral("tcp_frame"));
        if (tcpPoseIt != linkWorldPoses.end() && m_tcp_axes_actor != nullptr)
        {
            const auto& tcpPose = tcpPoseIt->second;
            vtkNew<vtkTransform> tcpTransform;
            tcpTransform->Translate(tcpPose.position_m[0],
                                    tcpPose.position_m[1],
                                    tcpPose.position_m[2]);
            // RPY → VTK 旋转：先绕 Z 轴转 RZ，再绕 Y 轴转 RY，再绕 X 轴转 RX
            tcpTransform->RotateZ(tcpPose.rpy_deg[2]);
            tcpTransform->RotateY(tcpPose.rpy_deg[1]);
            tcpTransform->RotateX(tcpPose.rpy_deg[0]);
            m_tcp_axes_actor->SetUserTransform(tcpTransform);
            m_tcp_axes_actor->SetVisibility(true);
        }
        // TCP 坐标轴更新在 RefreshScene(false) 的 Render() 之后，
        // 必须额外触发一次渲染，否则新位置不会立即显示。
        if (m_renderWindow != nullptr)
        {
            m_renderWindow->Render();
        }
    }
#else
    Q_UNUSED(linkWorldPoses);
#endif
}

void RobotVtkView::ShowWorkspacePointCloud(
    const std::vector<std::array<double, 3>>& tcpPositions)
{
    ShowKinematicsWorkspace(tcpPositions);
}

void RobotVtkView::ShowRequirementWorkspace(const std::vector<std::array<double, 3>>& workspacePoints)
{
    m_requirementWorkspacePositions = workspacePoints;
    RenderAnalysisLayers(true);
}

void RobotVtkView::ShowKinematicsWorkspace(const std::vector<std::array<double, 3>>& tcpPositions)
{
    m_kinematicsWorkspacePositions = tcpPositions;
    if (!tcpPositions.empty())
    {
        AutoEnableAnalysisLayerIfDefault(QString::fromLatin1(kLayerKinematicsWorkspace));
    }
    RenderAnalysisLayers(true);
}

void RobotVtkView::RenderAnalysisLayers(bool renderNow)
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer == nullptr) return;

    auto removeActor = [this](vtkSmartPointer<vtkActor>& actor) {
        if (actor != nullptr)
        {
            m_renderer->RemoveActor(actor);
            actor = nullptr;
        }
    };

    removeActor(m_requirement_workspace_actor);
    removeActor(m_kinematics_workspace_actor);
    removeActor(m_singularity_workspace_actor);

    if (IsAnalysisLayerVisible(QString::fromLatin1(kLayerRequirementWorkspace)) &&
        !m_requirementWorkspacePositions.empty())
    {
        m_requirement_workspace_actor = CreatePointCloudActor(
            m_requirementWorkspacePositions,
            0.23,
            0.51,
            0.96,
            0.34,
            2.4);
        m_renderer->AddActor(m_requirement_workspace_actor);
    }

    if (IsAnalysisLayerVisible(QString::fromLatin1(kLayerKinematicsWorkspace)) &&
        !m_kinematicsWorkspacePositions.empty())
    {
        m_kinematics_workspace_actor = CreatePointCloudActor(
            m_kinematicsWorkspacePositions,
            0.13,
            0.77,
            0.37,
            IsAnalysisLayerVisible(QString::fromLatin1(kLayerKinematicsSingularity)) ? 0.28 : 0.68,
            3.0);
        m_renderer->AddActor(m_kinematics_workspace_actor);
    }

    if (IsAnalysisLayerVisible(QString::fromLatin1(kLayerKinematicsSingularity)) &&
        !m_singularityWorkspacePositions.empty() &&
        m_singularityWorkspacePositions.size() == m_singularityFlags.size())
    {
        m_singularity_workspace_actor =
            CreateClassifiedPointCloudActor(m_singularityWorkspacePositions, m_singularityFlags);
        m_renderer->AddActor(m_singularity_workspace_actor);
    }

    if (renderNow && m_renderWindow != nullptr)
    {
        m_renderWindow->Render();
    }
#else
    Q_UNUSED(renderNow);
#endif
}

void RobotVtkView::ShowColoredWorkspacePointCloud(
    const std::vector<std::array<double, 3>>& tcpPositions,
    const std::vector<bool>& isSingular)
{
    ShowSingularityWorkspace(tcpPositions, isSingular);
}

void RobotVtkView::ShowSingularityWorkspace(
    const std::vector<std::array<double, 3>>& tcpPositions,
    const std::vector<bool>& isSingular)
{
    m_singularityWorkspacePositions = tcpPositions;
    m_singularityFlags = isSingular;
    if (!tcpPositions.empty())
    {
        AutoEnableAnalysisLayerIfDefault(QString::fromLatin1(kLayerKinematicsSingularity));
    }
    RenderAnalysisLayers(true);
}

void RobotVtkView::ShowRequirementKeyPoses(
    const std::vector<RoboSDP::Requirement::Dto::RequirementKeyPoseDto>& keyPoses,
    int selectedIndex)
{
    m_requirementKeyPoses = keyPoses;
    m_requirementSelectedKeyPoseIndex = selectedIndex;
    RenderRequirementKeyPoseLayer();
}

void RobotVtkView::SetRequirementWorkspaceLayerVisible(bool visible)
{
    SetAnalysisLayerVisible(QString::fromLatin1(kLayerRequirementWorkspace), visible);
}

void RobotVtkView::SetRequirementKeyPoseLayerVisible(bool visible)
{
    SetAnalysisLayerVisible(QString::fromLatin1(kLayerRequirementKeyPoses), visible);
}

bool RobotVtkView::IsRequirementWorkspaceLayerVisible() const
{
    return IsAnalysisLayerVisible(QString::fromLatin1(kLayerRequirementWorkspace));
}

bool RobotVtkView::IsRequirementKeyPoseLayerVisible() const
{
    return IsAnalysisLayerVisible(QString::fromLatin1(kLayerRequirementKeyPoses));
}

void RobotVtkView::ClearCache()
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer != nullptr)
    {
        for (const auto& [linkName, actor] : m_link_actors)
        {
            Q_UNUSED(linkName);
            if (actor != nullptr)
            {
                m_renderer->RemoveActor(actor);
            }
        }
    }

    // 【修复】清除 m_link_actors 后，m_last_picked_actor 指向的 Actor 可能已被销毁，
    // 必须置空以避免后续点击时野指针崩溃。
    m_last_picked_actor = nullptr;
    m_link_actors.clear();
    m_link_mesh_geometries.clear();
    m_link_to_joint_index.clear();
    ClearRequirementKeyPoseActors();
    m_requirement_workspace_actor = nullptr;
    m_kinematics_workspace_actor = nullptr;
    m_singularity_workspace_actor = nullptr;
#endif

    // 【修复】模型场景切换时，清除当前选中的连杆名称，避免旧引用残留。
    m_current_picked_link.clear();
}

void RobotVtkView::ClearIkPoseComparisonActors()
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer == nullptr)
    {
        return;
    }

    if (m_ik_target_axes_actor != nullptr)
    {
        m_renderer->RemoveActor(m_ik_target_axes_actor);
        m_ik_target_axes_actor = nullptr;
    }
    if (m_ik_actual_axes_actor != nullptr)
    {
        m_renderer->RemoveActor(m_ik_actual_axes_actor);
        m_ik_actual_axes_actor = nullptr;
    }
    if (m_ik_error_line_actor != nullptr)
    {
        m_renderer->RemoveActor(m_ik_error_line_actor);
        m_ik_error_line_actor = nullptr;
    }
    if (m_ik_target_marker_actor != nullptr)
    {
        m_renderer->RemoveActor(m_ik_target_marker_actor);
        m_ik_target_marker_actor = nullptr;
    }
    if (m_ik_target_label_actor != nullptr)
    {
        m_renderer->RemoveActor(m_ik_target_label_actor);
        if (m_label_renderer != nullptr)
        {
            m_label_renderer->RemoveActor(m_ik_target_label_actor);
        }
        m_ik_target_label_actor = nullptr;
    }
    if (m_ik_actual_label_actor != nullptr)
    {
        m_renderer->RemoveActor(m_ik_actual_label_actor);
        if (m_label_renderer != nullptr)
        {
            m_label_renderer->RemoveActor(m_ik_actual_label_actor);
        }
        m_ik_actual_label_actor = nullptr;
    }
    if (m_ik_error_label_actor != nullptr)
    {
        m_renderer->RemoveActor(m_ik_error_label_actor);
        if (m_label_renderer != nullptr)
        {
            m_label_renderer->RemoveActor(m_ik_error_label_actor);
        }
        m_ik_error_label_actor = nullptr;
    }
#endif
}

void RobotVtkView::ClearRequirementKeyPoseActors()
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer == nullptr)
    {
        return;
    }

    for (const auto& actor : m_requirement_key_pose_marker_actors)
    {
        if (actor != nullptr)
        {
            m_renderer->RemoveActor(actor);
        }
    }
    for (const auto& actor : m_requirement_key_pose_axes_actors)
    {
        if (actor != nullptr)
        {
            m_renderer->RemoveActor(actor);
        }
    }
    for (const auto& actor : m_requirement_key_pose_label_actors)
    {
        if (actor != nullptr)
        {
            m_renderer->RemoveActor(actor);
            if (m_label_renderer != nullptr)
            {
                m_label_renderer->RemoveActor(actor);
            }
        }
    }
    m_requirement_key_pose_marker_actors.clear();
    m_requirement_key_pose_axes_actors.clear();
    m_requirement_key_pose_label_actors.clear();
#endif
}

void RobotVtkView::RenderRequirementKeyPoseLayer()
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer == nullptr)
    {
        return;
    }

    ClearRequirementKeyPoseActors();
    if (!IsAnalysisLayerVisible(QString::fromLatin1(kLayerRequirementKeyPoses)))
    {
        if (m_renderWindow != nullptr)
        {
            m_renderWindow->Render();
        }
        return;
    }
    for (std::size_t index = 0; index < m_requirementKeyPoses.size(); ++index)
    {
        const auto& keyPose = m_requirementKeyPoses[index];
        const bool selected = static_cast<int>(index) == m_requirementSelectedKeyPoseIndex;
        const double red = selected ? 1.0 : 0.18;
        const double green = selected ? 0.62 : 0.72;
        const double blue = selected ? 0.12 : 1.0;

        vtkNew<vtkSphereSource> markerSource;
        markerSource->SetRadius(selected ? 0.032 : 0.022);
        markerSource->SetThetaResolution(24);
        markerSource->SetPhiResolution(16);
        vtkNew<vtkPolyDataMapper> markerMapper;
        markerMapper->SetInputConnection(markerSource->GetOutputPort());
        auto markerActor = vtkSmartPointer<vtkActor>::New();
        markerActor->SetMapper(markerMapper);
        markerActor->SetPosition(keyPose.pose[0], keyPose.pose[1], keyPose.pose[2]);
        markerActor->GetProperty()->SetColor(red, green, blue);
        markerActor->GetProperty()->SetOpacity(selected ? 0.92 : 0.72);
        markerActor->GetProperty()->SetAmbient(0.35);
        m_renderer->AddActor(markerActor);
        m_requirement_key_pose_marker_actors.push_back(markerActor);

        auto axesActor = CreateTcpComparisonAxes(selected ? 0.18 : 0.13);
        axesActor->SetUserTransform(BuildPoseTransform(ToCartesianPose(keyPose)));
        m_renderer->AddActor(axesActor);
        m_requirement_key_pose_axes_actors.push_back(axesActor);

        const QString labelText = keyPose.name.trimmed().isEmpty()
                                      ? QStringLiteral("工位%1").arg(static_cast<int>(index) + 1)
                                      : keyPose.name.trimmed();
        std::array<double, 3> labelPosition {
            keyPose.pose[0],
            keyPose.pose[1],
            keyPose.pose[2] + (selected ? 0.08 : 0.06)};
        auto labelActor = CreateBillboardLabel(labelText, labelPosition, red, green, blue, 16, selected ? 18 : 12);
        if (m_label_renderer != nullptr)
        {
            m_label_renderer->AddActor(labelActor);
        }
        else
        {
            m_renderer->AddActor(labelActor);
        }
        m_requirement_key_pose_label_actors.push_back(labelActor);
    }

    if (m_renderWindow != nullptr)
    {
        m_renderWindow->Render();
    }
#endif
}

void RobotVtkView::RenderIkPoseComparisonLayer()
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer == nullptr || !m_hasIkPoseComparison ||
        !IsAnalysisLayerVisible(QString::fromLatin1(kLayerKinematicsIkCompare)))
    {
        return;
    }

    ClearIkPoseComparisonActors();

    // IK 对比层激活时隐藏 FK 实时 TCP 坐标指示器，避免与 IK Target/Actual 坐标轴重叠
    if (m_tcp_axes_actor != nullptr)
    {
        m_tcp_axes_actor->SetVisibility(false);
    }

    m_ik_target_axes_actor = CreateTcpComparisonAxes(0.18);
    m_ik_target_axes_actor->SetUserTransform(BuildPoseTransform(m_ikTargetPose));
    m_renderer->AddActor(m_ik_target_axes_actor);

    vtkNew<vtkSphereSource> targetMarkerSource;
    targetMarkerSource->SetRadius(0.018);
    targetMarkerSource->SetThetaResolution(24);
    targetMarkerSource->SetPhiResolution(16);
    vtkNew<vtkPolyDataMapper> targetMarkerMapper;
    targetMarkerMapper->SetInputConnection(targetMarkerSource->GetOutputPort());
    m_ik_target_marker_actor = vtkSmartPointer<vtkActor>::New();
    m_ik_target_marker_actor->SetMapper(targetMarkerMapper);
    m_ik_target_marker_actor->SetPosition(
        m_ikTargetPose.position_m[0],
        m_ikTargetPose.position_m[1],
        m_ikTargetPose.position_m[2]);
    m_ik_target_marker_actor->GetProperty()->SetColor(0.0, 0.78, 1.0);
    m_ik_target_marker_actor->GetProperty()->SetOpacity(0.45);
    m_ik_target_marker_actor->GetProperty()->SetAmbient(0.55);
    m_renderer->AddActor(m_ik_target_marker_actor);

    m_ik_target_label_actor = CreateBillboardLabel(
        QStringLiteral("Target TCP"),
        m_ikTargetPose.position_m,
        0.32,
        0.90,
        1.0,
        18,
        18);
    if (m_label_renderer != nullptr)
    {
        m_label_renderer->AddActor(m_ik_target_label_actor);
    }
    else
    {
        m_renderer->AddActor(m_ik_target_label_actor);
    }

    if (m_hasIkActualPose)
    {
        m_ik_actual_axes_actor = CreateTcpComparisonAxes(0.16);
        m_ik_actual_axes_actor->SetUserTransform(BuildPoseTransform(m_ikActualPose));
        m_renderer->AddActor(m_ik_actual_axes_actor);

        const double okRed = 0.22;
        const double okGreen = 0.86;
        const double okBlue = 0.42;
        const double warnRed = 1.0;
        const double warnGreen = 0.34;
        const double warnBlue = 0.18;
        const double red = m_ikWithinTolerance ? okRed : warnRed;
        const double green = m_ikWithinTolerance ? okGreen : warnGreen;
        const double blue = m_ikWithinTolerance ? okBlue : warnBlue;

        m_ik_actual_label_actor = CreateBillboardLabel(
            QStringLiteral("Actual TCP"),
            m_ikActualPose.position_m,
            red,
            green,
            blue,
            18,
            -18);
        if (m_label_renderer != nullptr)
        {
            m_label_renderer->AddActor(m_ik_actual_label_actor);
        }
        else
        {
            m_renderer->AddActor(m_ik_actual_label_actor);
        }

        vtkNew<vtkLineSource> lineSource;
        lineSource->SetPoint1(
            m_ikTargetPose.position_m[0],
            m_ikTargetPose.position_m[1],
            m_ikTargetPose.position_m[2]);
        lineSource->SetPoint2(
            m_ikActualPose.position_m[0],
            m_ikActualPose.position_m[1],
            m_ikActualPose.position_m[2]);
        vtkNew<vtkPolyDataMapper> lineMapper;
        lineMapper->SetInputConnection(lineSource->GetOutputPort());
        m_ik_error_line_actor = vtkSmartPointer<vtkActor>::New();
        m_ik_error_line_actor->SetMapper(lineMapper);
        m_ik_error_line_actor->GetProperty()->SetColor(red, green, blue);
        m_ik_error_line_actor->GetProperty()->SetLineWidth(3.0);
        m_ik_error_line_actor->GetProperty()->SetOpacity(0.85);
        m_renderer->AddActor(m_ik_error_line_actor);

        const std::array<double, 3> midPoint {
            (m_ikTargetPose.position_m[0] + m_ikActualPose.position_m[0]) * 0.5,
            (m_ikTargetPose.position_m[1] + m_ikActualPose.position_m[1]) * 0.5,
            (m_ikTargetPose.position_m[2] + m_ikActualPose.position_m[2]) * 0.5};
        const QString stateText = m_ikWithinTolerance ? QStringLiteral("OK") : QStringLiteral("OUT OF TOL");
        const QString errorText = QStringLiteral("%1 | pos: %2 mm / ori: %3 deg")
                                      .arg(stateText)
                                      .arg(m_ikPositionErrorMm, 0, 'f', 2)
                                      .arg(m_ikOrientationErrorDeg, 0, 'f', 2);
        m_ik_error_label_actor = CreateBillboardLabel(errorText, midPoint, red, green, blue, 18, 0);
        if (m_label_renderer != nullptr)
        {
            m_label_renderer->AddActor(m_ik_error_label_actor);
        }
        else
        {
            m_renderer->AddActor(m_ik_error_label_actor);
        }
    }

    if (m_renderWindow != nullptr)
    {
        m_renderWindow->Render();
    }
#endif
}

void RobotVtkView::ResetCameraToCurrentScene()
{
#if defined(ROBOSDP_HAVE_VTK)
    // 中文说明：显式重置时先让 VTK 包围场景自适应焦距，再覆写等轴测方向。
    if (m_renderer != nullptr)
    {
        m_renderer->ResetCamera();
    }
    ApplyCameraPreset(1.0, -1.0, 1.0, 0.0, 0.0, 1.0);
#endif
}

void RobotVtkView::SetSkeletonVisible(bool visible)
{
    if (m_showSkeleton == visible)
    {
        return;
    }

    m_showSkeleton = visible;
    // 中文说明：顶部视图开关只影响显示层，不改变预览场景 DTO 或 FK 计算结果。
    RefreshScene(false);
}

void RobotVtkView::SetVisualMeshVisible(bool visible)
{
    if (m_showVisualMesh == visible)
    {
        return;
    }

    m_showVisualMesh = visible;
    RefreshScene(false);
}

void RobotVtkView::SetCollisionMeshVisible(bool visible)
{
    if (m_showCollisionMesh == visible)
    {
        return;
    }

    m_showCollisionMesh = visible;
    RefreshScene(false);
}

void RobotVtkView::SetJointAxesVisible(bool visible)
{
    if (m_showJointAxes == visible)
    {
        return;
    }

    m_showJointAxes = visible;
    RefreshScene(false);
}

void RobotVtkView::SetAxesVisible(bool visible)
{
    if (m_showAxes == visible)
    {
        return;
    }

    m_showAxes = visible;
    RefreshScene(false);
}

void RobotVtkView::SetGroundGridVisible(bool visible)
{
    if (m_showGroundGrid == visible)
    {
        return;
    }

    m_showGroundGrid = visible;
    RefreshScene(false);
}

void RobotVtkView::SetCornerAxesVisible(bool visible)
{
    if (m_showCornerAxes == visible)
    {
        return;
    }

    m_showCornerAxes = visible;
    if (m_statusLabel != nullptr)
    {
        m_statusLabel->setText(BuildStatusText());
    }
    RefreshCornerAxesVisibility();
}

void RobotVtkView::SetLinkLabelsVisible(bool visible)
{
    if (m_showLinkLabels == visible)
    {
        return;
    }

    m_showLinkLabels = visible;
    RefreshScene(false);
}

void RobotVtkView::SetJointLabelsVisible(bool visible)
{
    if (m_showJointLabels == visible)
    {
        return;
    }

    m_showJointLabels = visible;
    RefreshScene(false);
}

void RobotVtkView::SetTcpGizmoVisible(bool visible)
{
    if (m_showTcpGizmo == visible)
    {
        return;
    }

    m_showTcpGizmo = visible;
    RefreshScene(false);
}

void RobotVtkView::ApplyDesignViewPreset()
{
    m_showSkeleton = true;
    m_showVisualMesh = false;
    m_showCollisionMesh = false;
    m_showJointAxes = true;
    m_showAxes = true;
    m_showGroundGrid = true;
    m_showCornerAxes = true;
    m_showLinkLabels = false;
    m_showJointLabels = true;
    m_showTcpGizmo = false;
    RefreshCornerAxesVisibility();
    RefreshScene(false);
}

void RobotVtkView::ApplyEngineeringViewPreset()
{
    m_showSkeleton = true;
    m_showVisualMesh = true;
    m_showCollisionMesh = false;
    m_showJointAxes = false;
    m_showAxes = true;
    m_showGroundGrid = true;
    m_showCornerAxes = true;
    m_showLinkLabels = false;
    m_showJointLabels = false;
    m_showTcpGizmo = false;
    RefreshCornerAxesVisibility();
    RefreshScene(false);
}

void RobotVtkView::ApplyDiagnosticViewPreset()
{
    m_showSkeleton = true;
    m_showVisualMesh = false;
    m_showCollisionMesh = false;
    m_showJointAxes = true;
    m_showAxes = true;
    m_showGroundGrid = true;
    m_showCornerAxes = true;
    m_showLinkLabels = true;
    m_showJointLabels = true;
    m_showTcpGizmo = false;
    RefreshCornerAxesVisibility();
    RefreshScene(false);
}

void RobotVtkView::SetFrontCameraView()
{
    ApplyCameraPreset(0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
}

void RobotVtkView::SetSideCameraView()
{
    ApplyCameraPreset(1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
}

void RobotVtkView::SetTopCameraView()
{
    ApplyCameraPreset(0.0, 0.0, 1.0, 0.0, 1.0, 0.0);
}

void RobotVtkView::SetIsometricCameraView()
{
    ApplyCameraPreset(1.0, -1.0, 1.0, 0.0, 0.0, 1.0);
}

void RobotVtkView::ZoomInCamera()
{
    ZoomCamera(0.82);
}

void RobotVtkView::ZoomOutCamera()
{
    ZoomCamera(1.22);
}

void RobotVtkView::BuildAnalysisLayerPanel(QWidget* viewportFrame)
{
    if (viewportFrame == nullptr)
    {
        return;
    }

    m_analysisLayerPanel = new QFrame(viewportFrame);
    auto* panel = m_analysisLayerPanel;
    panel->setObjectName(QStringLiteral("analysisLayerPanel"));
    panel->setAttribute(Qt::WA_StyledBackground, true);
    panel->setStyleSheet(QStringLiteral(
        "QFrame#analysisLayerPanel{"
        "background:transparent;"
        "border:none;"
        "}"
        "QLabel#analysisLayerTitle{color:#ffffff;font-size:28px;font-weight:700;}"
        "QCheckBox{color:#f8fafc;font-size:26px;font-weight:600;spacing:14px;background:transparent;}"
        "QCheckBox::indicator{width:26px;height:26px;}"
        "QLabel#analysisLayerSwatch{min-width:24px;max-width:24px;min-height:24px;max-height:24px;}"));

    auto* opacityEffect = new QGraphicsOpacityEffect(panel);
    opacityEffect->setOpacity(kAnalysisLayerOverlayOpacity);
    panel->setGraphicsEffect(opacityEffect);

    auto* panelLayout = new QVBoxLayout(panel);
    panelLayout->setContentsMargins(0, 0, 0, 0);
    panelLayout->setSpacing(12);

    for (const auto& spec : kAnalysisLayerSpecs)
    {
        const QString layerId = QString::fromLatin1(spec.id);
        m_analysisLayerVisibility[layerId] = spec.defaultVisible;

        auto* row = new QWidget(panel);
        auto* rowLayout = new QHBoxLayout(row);
        rowLayout->setContentsMargins(0, 0, 0, 0);
        rowLayout->setSpacing(10);

        auto* swatch = new QLabel(row);
        swatch->setObjectName(QStringLiteral("analysisLayerSwatch"));
        swatch->setStyleSheet(QStringLiteral("QLabel#analysisLayerSwatch{background:%1;}").arg(QString::fromLatin1(spec.color)));

        auto* check = new QCheckBox(QString::fromUtf8(spec.name), row);
        check->setChecked(spec.defaultVisible);
        m_analysisLayerChecks[layerId] = check;
        connect(check, &QCheckBox::toggled, this, [this, layerId](bool visible) {
            SetAnalysisLayerVisible(layerId, visible);
        });

        rowLayout->addWidget(swatch);
        rowLayout->addWidget(check, 1);
        panelLayout->addWidget(row);
    }

    PositionAnalysisLayerPanel();
    panel->show();
    RaiseViewOverlays();
}

void RobotVtkView::RefreshAnalysisLayerPanel()
{
    for (const auto& [layerId, check] : m_analysisLayerChecks)
    {
        if (check == nullptr)
        {
            continue;
        }
        const QSignalBlocker blocker(check);
        check->setChecked(IsAnalysisLayerVisible(layerId));
    }
}

void RobotVtkView::SetAnalysisLayerVisible(const QString& layerId, bool visible)
{
    SetAnalysisLayerVisibleInternal(layerId, visible, true);
}

void RobotVtkView::SetAnalysisLayerVisibleInternal(const QString& layerId, bool visible, bool userInitiated)
{
    if (FindLayerSpec(layerId) == nullptr)
    {
        return;
    }

    const bool changed = IsAnalysisLayerVisible(layerId) != visible;
    m_analysisLayerVisibility[layerId] = visible;
    if (userInitiated)
    {
        m_analysisLayerUserOverrides[layerId] = true;
    }
    RefreshAnalysisLayerPanel();

#if defined(ROBOSDP_HAVE_VTK)
    if (layerId == QString::fromLatin1(kLayerRequirementKeyPoses))
    {
        RenderRequirementKeyPoseLayer();
    }
    else if (layerId == QString::fromLatin1(kLayerKinematicsIkCompare))
    {
        if (visible)
        {
            RenderIkPoseComparisonLayer();
        }
        else
        {
            ClearIkPoseComparisonActors();
            // 图层关闭时恢复 FK 实时 TCP 坐标指示器
            if (m_tcp_axes_actor != nullptr && !m_currentScene.IsEmpty())
            {
                m_tcp_axes_actor->SetVisibility(true);
            }
            if (m_renderWindow != nullptr)
            {
                m_renderWindow->Render();
            }
        }
    }
    else
    {
        RenderAnalysisLayers(true);
    }
#endif

    if (changed)
    {
        if (layerId == QString::fromLatin1(kLayerRequirementWorkspace))
        {
            emit signalRequirementWorkspaceLayerVisibilityChanged(visible);
        }
        else if (layerId == QString::fromLatin1(kLayerRequirementKeyPoses))
        {
            emit signalRequirementKeyPoseLayerVisibilityChanged(visible);
        }
    }
}

void RobotVtkView::AutoEnableAnalysisLayerIfDefault(const QString& layerId)
{
    const auto overrideIt = m_analysisLayerUserOverrides.find(layerId);
    if (overrideIt != m_analysisLayerUserOverrides.end() && overrideIt->second)
    {
        return;
    }

    if (!IsAnalysisLayerVisible(layerId))
    {
        SetAnalysisLayerVisibleInternal(layerId, true, false);
    }
}

bool RobotVtkView::IsAnalysisLayerVisible(const QString& layerId) const
{
    const auto it = m_analysisLayerVisibility.find(layerId);
    if (it != m_analysisLayerVisibility.end())
    {
        return it->second;
    }

    if (const AnalysisLayerUiSpec* spec = FindLayerSpec(layerId))
    {
        return spec->defaultVisible;
    }
    return false;
}

void RobotVtkView::RaiseViewOverlays()
{
    if (m_scaleBarOverlay != nullptr)
    {
        m_scaleBarOverlay->raise();
    }
    if (m_cameraToolbar != nullptr)
    {
        m_cameraToolbar->raise();
    }
    if (m_analysisLayerPanel != nullptr)
    {
        m_analysisLayerPanel->raise();
    }
}

void RobotVtkView::PositionCameraToolbar()
{
    if (m_cameraToolbar == nullptr || m_cameraToolbar->parentWidget() == nullptr)
    {
        return;
    }

    auto* parent = m_cameraToolbar->parentWidget();
    m_cameraToolbar->adjustSize();
    const int topMargin = 20;
    const int x = std::max(8, (parent->width() - m_cameraToolbar->width()) / 2);
    m_cameraToolbar->move(x, topMargin);
}

void RobotVtkView::PositionAnalysisLayerPanel()
{
    if (m_analysisLayerPanel == nullptr || m_analysisLayerPanel->parentWidget() == nullptr)
    {
        return;
    }

    auto* parent = m_analysisLayerPanel->parentWidget();
    m_analysisLayerPanel->adjustSize();
    const int margin = 28;
    const int x = std::max(margin, parent->width() - m_analysisLayerPanel->width() - margin);
    m_analysisLayerPanel->move(x, margin);
}

void RobotVtkView::BuildLayout()
{
    m_layout = new QVBoxLayout(this);
    m_layout->setContentsMargins(0, 0, 0, 0);
    m_layout->setSpacing(0);

    setObjectName(QStringLiteral("robotVtkView"));
    setStyleSheet(QStringLiteral(
        "QWidget#robotVtkView{background:#dfe3ea;}"
        "QFrame#renderViewportFrame{background:#000000;border:2px solid #003dff;}"
        "QLabel#vtkStatusLabel{color:#475569;font-size:11px;padding-left:6px;}"));

#if defined(ROBOSDP_HAVE_VTK)
    BuildVtkView();
#else
    BuildFallbackView();
#endif
}

void RobotVtkView::BuildControlBar()
{
}

void RobotVtkView::BuildCameraToolbar(QWidget* viewportFrame)
{
    if (viewportFrame == nullptr)
    {
        return;
    }

    auto* toolbar = new QWidget(viewportFrame);
    m_cameraToolbar = toolbar;
    toolbar->setObjectName(QStringLiteral("cameraToolbar"));
    toolbar->setAttribute(Qt::WA_StyledBackground, true);
    toolbar->setMouseTracking(true);
    toolbar->setStyleSheet(QStringLiteral(
        "QWidget#cameraToolbar{background:transparent;border:none;}"
        "QToolButton{"
        "background:transparent;"
        "border:2px solid rgba(248,250,252,145);"
        "border-radius:8px;"
        "color:rgba(248,250,252,210);"
        "min-width:56px;"
        "max-width:56px;"
        "min-height:52px;"
        "max-height:52px;"
        "padding:0;"
        "}"
        "QToolButton:hover{background:rgba(255,255,255,45);border-color:rgba(255,255,255,210);}"
        "QToolButton:pressed{background:rgba(255,255,255,70);}"));

    auto* layout = new QHBoxLayout(toolbar);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(12);

    auto addButton = [this, toolbar, layout](const QString& iconKind, const QString& tooltip, const auto& handler) {
        auto* button = new QToolButton(toolbar);
        button->setIcon(MakeCameraToolbarIcon(iconKind));
        button->setIconSize(QSize(44, 44));
        button->setToolTip(tooltip);
        button->setCursor(Qt::PointingHandCursor);
        button->setFocusPolicy(Qt::NoFocus);
        button->setMouseTracking(true);
        button->installEventFilter(this);
        layout->addWidget(button);
        connect(button, &QToolButton::clicked, this, handler);
    };

    addButton(QStringLiteral("zoom_in"), QStringLiteral("放大视图"), [this]() { ZoomCamera(0.82); });
    addButton(QStringLiteral("zoom_out"), QStringLiteral("缩小视图"), [this]() { ZoomCamera(1.22); });
    addButton(QStringLiteral("reset"), QStringLiteral("重置相机"), [this]() { ResetCameraToCurrentScene(); });
    addButton(QStringLiteral("front"), QStringLiteral("正视图"), [this]() { SetFrontCameraView(); });
    addButton(QStringLiteral("side"), QStringLiteral("侧视图"), [this]() { SetSideCameraView(); });
    addButton(QStringLiteral("top"), QStringLiteral("俯视图"), [this]() { SetTopCameraView(); });
    addButton(QStringLiteral("iso"), QStringLiteral("轴测图"), [this]() { SetIsometricCameraView(); });

    PositionCameraToolbar();
    toolbar->installEventFilter(this);
    toolbar->hide();
}

void RobotVtkView::ZoomCamera(double factor)
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer == nullptr || factor <= 0.0)
    {
        return;
    }

    vtkCamera* camera = m_renderer->GetActiveCamera();
    if (camera == nullptr)
    {
        return;
    }

    camera->Dolly(1.0 / factor);
    m_renderer->ResetCameraClippingRange();
    UpdateScaleBar();
    if (m_renderWindow != nullptr)
    {
        m_renderWindow->Render();
    }
#else
    Q_UNUSED(factor);
#endif
}

void RobotVtkView::UpdateCameraToolbarVisibility(const QPoint& viewportPos)
{
    if (m_cameraToolbar == nullptr || m_cameraToolbar->parentWidget() == nullptr)
    {
        return;
    }

    auto* parent = m_cameraToolbar->parentWidget();
    const QRect toolbarRect = m_cameraToolbar->geometry().adjusted(-12, -12, 12, 16);
    const int hotZoneWidth = std::max(m_cameraToolbar->width() + 96, parent->width() / 3);
    const int hotZoneHeight = std::max(m_cameraToolbar->height() + 34, 88);
    const QRect hotZone(
        std::max(0, (parent->width() - hotZoneWidth) / 2),
        0,
        std::min(hotZoneWidth, parent->width()),
        hotZoneHeight);

    const bool shouldShow = hotZone.contains(viewportPos) || toolbarRect.contains(viewportPos);
    if (m_cameraToolbar->isVisible() != shouldShow)
    {
        m_cameraToolbar->setVisible(shouldShow);
        if (shouldShow)
        {
            m_cameraToolbar->raise();
        }
    }
}

void RobotVtkView::BuildVtkView()
{
#if defined(ROBOSDP_HAVE_VTK)
    // 中文说明：中央三维区使用原生 QVTK 控件承载骨架预览，而不是直接在 QWidget 里写渲染逻辑。
    auto* viewportFrame = new QFrame(this);
    viewportFrame->setObjectName(QStringLiteral("renderViewportFrame"));
    viewportFrame->setMouseTracking(true);
    auto* viewportLayout = new QVBoxLayout(viewportFrame);
    viewportLayout->setContentsMargins(0, 0, 0, 0);
    viewportLayout->setSpacing(0);

    m_vtkWidget = new QVTKOpenGLNativeWidget(viewportFrame);
    m_vtkWidget->setMinimumSize(0, 0);
    m_vtkWidget->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    m_vtkWidget->setMouseTracking(true);

    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    // 中文说明：开启 8x MSAA 多重采样抗锯齿，平滑 3D 骨架边缘棱角，提升渲染画质。
    renderWindow->SetMultiSamples(8);
    m_vtkWidget->setRenderWindow(renderWindow);
    m_renderWindow = renderWindow;

    vtkNew<vtkRenderer> renderer;
    renderer->SetLayer(0);
    renderWindow->AddRenderer(renderer);
    m_renderer = renderer;

    // 创建标签层 Renderer（Layer 1），确保 link/joint 标签永远渲染在最前面不被遮挡
    vtkNew<vtkRenderer> labelRenderer;
    labelRenderer->SetLayer(1);
    labelRenderer->EraseOff(); // 透明背景，让 Layer 0 的几何体透过来
    labelRenderer->SetActiveCamera(renderer->GetActiveCamera());
    renderWindow->SetNumberOfLayers(2);
    renderWindow->AddRenderer(labelRenderer);
    m_label_renderer = labelRenderer;

    BuildCornerAxesWidget();

    // TCP 坐标系指示器：红绿蓝三轴，默认隐藏，FK 成功后显示在末端
    m_tcp_axes_actor = vtkSmartPointer<vtkAxesActor>::New();
    m_tcp_axes_actor->SetTotalLength(0.15, 0.15, 0.15);
    m_tcp_axes_actor->SetNormalizedShaftLength(0.8, 0.8, 0.8);
    m_tcp_axes_actor->SetNormalizedTipLength(0.2, 0.2, 0.2);
    m_tcp_axes_actor->SetVisibility(false);
    // XYZ 标签样式：与角落极坐标一致，禁用自动缩放，小字号 + 亮色
    {
        auto tuneCaption = [](vtkCaptionActor2D* captionActor) {
            if (captionActor == nullptr || captionActor->GetCaptionTextProperty() == nullptr)
                return;
            if (captionActor->GetTextActor())
                captionActor->GetTextActor()->SetTextScaleModeToNone();
            captionActor->BorderOff();
            captionActor->LeaderOff();
            captionActor->GetCaptionTextProperty()->SetFontSize(12);
            captionActor->GetCaptionTextProperty()->SetBold(false);
            captionActor->GetCaptionTextProperty()->SetColor(0.90, 0.92, 0.95);
        };
        tuneCaption(m_tcp_axes_actor->GetXAxisCaptionActor2D());
        tuneCaption(m_tcp_axes_actor->GetYAxisCaptionActor2D());
        tuneCaption(m_tcp_axes_actor->GetZAxisCaptionActor2D());
    }
    m_renderer->AddActor(m_tcp_axes_actor);

    // 🔽🔽🔽 【新增水印初始化代码】 🔽🔽🔽
    m_watermark_actor = vtkSmartPointer<vtkTextActor>::New();
    m_watermark_actor->SetInput("No Preview Model"); // 默认文本
    // 设置文字属性
    vtkTextProperty* textProp = m_watermark_actor->GetTextProperty();
    textProp->SetFontSize(16);          // 字体大小
    textProp->SetColor(0.8, 0.8, 0.8);  // 浅灰色
    textProp->SetOpacity(0.6);          // 半透明效果，不喧宾夺主
    textProp->SetBold(true);
    textProp->SetShadow(true);          // 开启阴影让文字在任何背景下都清晰
    textProp->SetShadowOffset(1, -1);
    
    // 设置水印位置：屏幕右上角 (以屏幕像素系为基准)
    // VTK 的 SetPosition2 是用 Normalized Viewport (0~1) 坐标的，这里我们直接给具体像素位置或者比例
    m_watermark_actor->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
    m_watermark_actor->SetPosition(0.02, 0.95); // 0.02(左) 0.95(上)，即左上角

    m_renderer->AddActor2D(m_watermark_actor); // 加入渲染器 (2D 覆盖层)
    // 🔼🔼🔼 【新增结束】 🔼🔼🔼

    // ── 比例尺初始化（QWidget 覆盖层，QPainter 绘制白色刻度线）────────
    m_scaleBarOverlay = new ScaleBarOverlay(viewportFrame);
    m_scaleBarOverlay->show();
    viewportFrame->installEventFilter(this);
    // ── 比例尺初始化结束 ──────────────────────────────────────────

    BuildCameraToolbar(viewportFrame);
    BuildAnalysisLayerPanel(viewportFrame);

    viewportLayout->addWidget(m_vtkWidget, 1);
    RaiseViewOverlays();
    m_layout->addWidget(viewportFrame, 1);

    // 中文说明：绑定自定义拾取交互器，替换默认相机操作风格，支持左键点击高亮。
    vtkNew<VtkClickInteractorStyle> clickStyle;
    clickStyle->parentView = this;
    clickStyle->renderer = m_renderer;
    clickStyle->SetDefaultRenderer(m_renderer);
    m_vtkWidget->interactor()->SetInteractorStyle(clickStyle);

    // 中文说明：安装 Qt 事件过滤器，监听鼠标释放与滚轮事件，自动刷新比例尺。
    m_vtkWidget->installEventFilter(this);

    RefreshScene();
#endif
}

void RobotVtkView::HandleActorClicked(vtkActor* clickedActor, const double pickPosition[3])
{
#if defined(ROBOSDP_HAVE_VTK)
    // 中文说明：恢复上一个被选中对象的原始颜色和环境光系数。
    if (m_last_picked_actor != nullptr && m_last_picked_actor->GetProperty() != nullptr)
    {
        m_last_picked_actor->GetProperty()->SetColor(m_last_picked_color);
        m_last_picked_actor->GetProperty()->SetAmbient(m_last_picked_ambient);
    }

    if (clickedActor == nullptr)
    {
        // 【修复】点击空白区域，同时清除选中状态和当前选中连杆名称
        m_last_picked_actor = nullptr;
        m_current_picked_link.clear();    // <--- 【新增】清理选中连杆
        emit signalLinkPicked(QString());
        if (m_statusLabel != nullptr)
        {
            m_statusLabel->setText(BuildStatusText());
        }
        return;
    }

    // 记录新选中对象的原始属性
    m_last_picked_actor = clickedActor;
    clickedActor->GetProperty()->GetColor(m_last_picked_color);
    m_last_picked_ambient = clickedActor->GetProperty()->GetAmbient();

    // 设置高亮效果：青色 + 高环境光，模拟"发光发亮"的选中状态
    clickedActor->GetProperty()->SetColor(0.0, 1.0, 1.0); // Cyan
    clickedActor->GetProperty()->SetAmbient(0.8);

    // ── 在 m_link_actors 字典中反向查找被点击 Actor 的实体名称 ──────────
    // 【修复】从复合键 "visual:link_1:0" 中提取纯 link_name（中间段），
    // 格式为 "layerName:linkName:geometryIndex"，使用 section(':', 1, 1) 取 linkName。
    QString pickedLinkName;
    for (auto it = m_link_actors.begin(); it != m_link_actors.end(); ++it)
    {
        if (it->second.Get() == clickedActor)
        {
            // 提取复合键的第二段：layer:link_name:index → link_name
            pickedLinkName = it->first.section(QLatin1Char(':'), 1, 1);
            break;
        }
    }

    // ── 如果在 Mesh Actor 字典中未找到，尝试按拾取位置查找最近骨架节点 ──
    if (pickedLinkName.isEmpty() && pickPosition != nullptr)
    {
        // 中文说明：遍历所有骨架节点，找距离拾取位置最近的节点，
        // 如果距离小于合理阈值（2cm），认为点击了该骨架球。
        double minDistSq = std::numeric_limits<double>::max();
        int nearestNodeIndex = -1;
        for (int i = 0; i < static_cast<int>(m_currentScene.nodes.size()); ++i)
        {
            const auto& node = m_currentScene.nodes[i];
            const double dx = node.position_m[0] - pickPosition[0];
            const double dy = node.position_m[1] - pickPosition[1];
            const double dz = node.position_m[2] - pickPosition[2];
            const double distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < minDistSq)
            {
                minDistSq = distSq;
                nearestNodeIndex = i;
            }
        }
        if (nearestNodeIndex >= 0)
        {
            const double threshold = 0.02; // 2cm 阈值，覆盖骨架球半径
            if (std::sqrt(minDistSq) < threshold)
            {
                pickedLinkName = m_currentScene.nodes[nearestNodeIndex].link_name;
            }
        }
    }

    // 【逆向驱动】保存当前选中连杆名称（纯 link_name），供滚轮关节驱动使用。
    m_current_picked_link = pickedLinkName;
    emit signalLinkPicked(pickedLinkName);

    // 中文说明：状态栏显示，便于用户确认当前选中了哪个连杆/骨架。
    const QString displayName = pickedLinkName.isEmpty()
        ? QStringLiteral("骨架段/关节")
        : pickedLinkName;
    if (m_statusLabel != nullptr)
    {
        m_statusLabel->setText(
            QStringLiteral("中央三维主视图区：已选中 [ %1 ]").arg(displayName));
    }

    if (m_renderWindow != nullptr)
    {
        m_renderWindow->Render();
    }
#else
    Q_UNUSED(clickedActor);
    Q_UNUSED(pickPosition);
#endif
}

void RobotVtkView::BuildFallbackView()
{
    auto* viewportFrame = new QFrame(this);
    viewportFrame->setObjectName(QStringLiteral("renderViewportFrame"));
    auto* viewportLayout = new QVBoxLayout(viewportFrame);
    viewportLayout->setContentsMargins(18, 18, 18, 18);

    auto* fallbackLabel = new QLabel(QStringLiteral("VTK 渲染后端不可用"), viewportFrame);
    fallbackLabel->setAlignment(Qt::AlignCenter);
    fallbackLabel->setStyleSheet(QStringLiteral("color:#cbd5e1;font-size:16px;font-weight:700;"));
    viewportLayout->addWidget(fallbackLabel, 1);

    m_layout->addWidget(viewportFrame, 1);
    RefreshScene();
}

void RobotVtkView::BuildCornerAxesWidget()
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_vtkWidget == nullptr || m_corner_axes_widget != nullptr)
    {
        return;
    }

    vtkNew<vtkAxesActor> axesActor;
    axesActor->SetTotalLength(0.85, 0.85, 0.85);
    axesActor->SetShaftTypeToCylinder();
    axesActor->SetCylinderRadius(0.018);
    axesActor->SetConeRadius(0.075);
    axesActor->SetSphereRadius(0.045);
    axesActor->GetXAxisShaftProperty()->SetColor(1.0, 0.0, 0.0);
    axesActor->GetXAxisTipProperty()->SetColor(1.0, 0.0, 0.0);
    axesActor->GetYAxisShaftProperty()->SetColor(1.0, 1.0, 0.0);
    axesActor->GetYAxisTipProperty()->SetColor(1.0, 1.0, 0.0);
    axesActor->GetZAxisShaftProperty()->SetColor(0.0, 0.80, 0.0);
    axesActor->GetZAxisTipProperty()->SetColor(0.0, 0.80, 0.0);

    auto tuneCaption = [](vtkCaptionActor2D* captionActor, double red, double green, double blue) {
        if (captionActor == nullptr || captionActor->GetCaptionTextProperty() == nullptr)
        {
            return;
        }
        // 🔽🔽🔽 【核心修复：彻底禁用自动缩放】 🔽🔽🔽
        if (captionActor->GetTextActor())
        {
            captionActor->GetTextActor()->SetTextScaleModeToNone();
        }
        captionActor->BorderOff();
        captionActor->LeaderOff();
        // 🔼🔼🔼 【修复结束】 🔼🔼🔼
        captionActor->GetCaptionTextProperty()->SetFontSize(12);
        captionActor->GetCaptionTextProperty()->SetBold(true);
        captionActor->GetCaptionTextProperty()->SetColor(red, green, blue);
    };
    tuneCaption(axesActor->GetXAxisCaptionActor2D(), 1.0, 0.0, 0.0);
    tuneCaption(axesActor->GetYAxisCaptionActor2D(), 1.0, 1.0, 0.0);
    tuneCaption(axesActor->GetZAxisCaptionActor2D(), 0.0, 0.80, 0.0);

    m_corner_axes_actor = axesActor;
    m_corner_axes_widget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    m_corner_axes_widget->SetOrientationMarker(m_corner_axes_actor);
    m_corner_axes_widget->SetInteractor(m_vtkWidget->interactor());
    m_corner_axes_widget->SetViewport(0.015, 0.015, 0.145, 0.145);
    RefreshCornerAxesVisibility();
#endif
}

void RobotVtkView::RefreshCornerAxesVisibility()
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_corner_axes_widget == nullptr)
    {
        return;
    }

    // 中文说明：角落方向坐标轴固定在屏幕空间，不参与主场景重建和相机包围盒计算。
    m_corner_axes_widget->SetEnabled(m_showCornerAxes ? 1 : 0);
    if (m_showCornerAxes)
    {
        m_corner_axes_widget->InteractiveOff();
    }
    if (m_renderWindow != nullptr)
    {
        m_renderWindow->Render();
    }
#endif
}

void RobotVtkView::RefreshScene(bool resetCamera)
{
    if (m_statusLabel != nullptr)
    {
        m_statusLabel->setText(BuildStatusText());
    }

#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer != nullptr)
    {
        // 【修复】场景重建会调用 RemoveAllViewProps()，之前缓存的 Actor 指针全部失效，
        // 必须清空 m_last_picked_actor 避免后续点击时野指针崩溃。
        // 注意：不清除 m_current_picked_link（QString），因为它不是指针类型，
        // 在场景重建后仍保持有效。模型切换时的清理由 ClearCache() 负责。
        m_last_picked_actor = nullptr;

        UrdfPreviewDisplayOptions displayOptions;
        displayOptions.show_skeleton = m_showSkeleton;
        displayOptions.show_visual_meshes = m_showVisualMesh;
        displayOptions.show_collision_meshes = m_showCollisionMesh;
        displayOptions.show_joint_axes = m_showJointAxes;
        displayOptions.show_axes = m_showAxes;
        displayOptions.show_ground_grid = m_showGroundGrid;
        displayOptions.show_link_labels = m_showLinkLabels;
        displayOptions.show_joint_labels = m_showJointLabels;
        displayOptions.reset_camera = resetCamera;
        if (m_currentScene.IsEmpty())
        {
            VtkSceneBuilder::BuildMinimalTestScene(
                m_renderer,
                displayOptions.show_axes,
                displayOptions.show_ground_grid,
                displayOptions.reset_camera);
                m_watermark_actor->SetInput("View: Minimal Test Scene");
                m_watermark_actor->GetTextProperty()->SetColor(0.6, 0.6, 0.6); 
        }
        else
        {
            VtkSceneBuilder::BuildUrdfPreviewScene(
                m_renderer,
                m_currentScene,
                displayOptions,
                m_link_actors,
                m_link_mesh_geometries,
                m_label_renderer);

           // 🔽🔽🔽 【修改这里的判断条件】 🔽🔽🔽
            // 不再判断有没有外壳，而是判断场景数据里有没有携带真实的 URDF 文件路径
            if (!m_currentScene.urdf_file_path.trimmed().isEmpty()) {
                // 如果有路径，说明它是从一个真实的 URDF 文件里读出来的
                m_watermark_actor->SetInput("[ Driven by: URDF Physical Model ]");
                m_watermark_actor->GetTextProperty()->SetColor(0.4, 0.8, 1.0); // 科技蓝
            } else {
                // 如果没有路径，说明它是我们在内存里用 DH 参数硬算出来的骨架
                m_watermark_actor->SetInput("[ Driven by: DH/MDH Kinematic Skeleton ]");
                m_watermark_actor->GetTextProperty()->SetColor(1.0, 0.6, 0.2); // 警示橙色
            }
            // 🔼🔼🔼 【修改结束】 🔼🔼🔼


        }
        // 🔽🔽🔽 【关键修复】：重新把水印贴回画布 🔽🔽🔽
        // 因为 VtkSceneBuilder 内部调用了 RemoveAllViewProps() 清空了图层，
        // 所以每次刷新后，必须重新把水印层注册进渲染器。
        m_renderer->AddActor2D(m_watermark_actor);
        // 🔼🔼🔼 【修复结束】 🔼🔼🔼

        // 🔽🔽🔽 重新添加 TCP 坐标系指示器（同样被 RemoveAllViewProps 清掉）🔽🔽🔽
        if (m_tcp_axes_actor != nullptr)
        {
            m_renderer->AddActor(m_tcp_axes_actor);
        }
        // 🔼🔼🔼 🔼🔼🔼

        if (m_hasIkPoseComparison)
        {
            RenderIkPoseComparisonLayer();
        }
        RenderAnalysisLayers(false);
        RenderRequirementKeyPoseLayer();

        // ── 【逆向驱动】TCP 3D Gizmo 初始化（vtkBoxWidget2）────────────
        // 默认关闭：该控件表示“IK 目标拖拽”，不是机械臂尺寸编辑器，必须由显式交互模式打开。
        if (m_currentScene.IsEmpty() || !m_showTcpGizmo)
        {
            // 空场景下禁用 Gizmo
            if (m_tcp_gizmo_widget != nullptr)
            {
                m_tcp_gizmo_widget->SetEnabled(0);
            }
        }
        else
        {
            // 有机器人场景 → 确保 Gizmo 创建并启用
            if (m_tcp_gizmo_widget == nullptr)
            {
                // 创建 vtkBoxRepresentation（变换框的表现层）
                vtkNew<vtkBoxRepresentation> boxRep;

                // 创建 vtkBoxWidget2（变换框的交互控件）
                m_tcp_gizmo_widget = vtkSmartPointer<vtkBoxWidget2>::New();
                m_tcp_gizmo_widget->SetInteractor(m_vtkWidget->interactor());
                m_tcp_gizmo_widget->SetRepresentation(boxRep);
                m_tcp_gizmo_widget->SetTranslationEnabled(1);
                m_tcp_gizmo_widget->SetRotationEnabled(1);
                m_tcp_gizmo_widget->SetScalingEnabled(0);   // 不缩放
                m_tcp_gizmo_widget->SetMoveFacesEnabled(1);

                // 注册 InteractionEvent 回调：拖动时发射位姿信号
                vtkNew<vtkCallbackCommand> tcpDragCallback;
                tcpDragCallback->SetCallback([](vtkObject* caller,
                                                 long unsigned int,
                                                 void* clientData,
                                                 void*) {
                    auto* widget = vtkBoxWidget2::SafeDownCast(caller);
                    if (widget == nullptr) return;
                    auto* view = static_cast<RoboSDP::Desktop::Vtk::RobotVtkView*>(clientData);
                    if (view == nullptr) return;
                    // 从关联的 Representation 中提取变换矩阵
                    auto* rep = vtkBoxRepresentation::SafeDownCast(
                        widget->GetRepresentation());
                    if (rep == nullptr) return;
                    vtkNew<vtkTransform> tcpTransform;
                    rep->GetTransform(tcpTransform);
                    view->EmitTcpDrag(tcpTransform);
                });
                tcpDragCallback->SetClientData(this);
                m_tcp_gizmo_widget->AddObserver(vtkCommand::InteractionEvent, tcpDragCallback);
            }

            // 将 Gizmo 定位到末端（取场景中最后一个节点的位置）
            if (!m_currentScene.nodes.empty())
            {
                const auto& lastNode = m_currentScene.nodes.back();
                const auto& pos = lastNode.world_pose.position_m;
                // 用小包围盒放置，然后通过 SetTransform 移动到 TCP 位置
                const double boxHalfSize = 0.05; // 5cm 的半边长
                double bounds[6] = {
                    -boxHalfSize, boxHalfSize,
                    -boxHalfSize, boxHalfSize,
                    -boxHalfSize, boxHalfSize
                };
                auto* boxRep = vtkBoxRepresentation::SafeDownCast(
                    m_tcp_gizmo_widget->GetRepresentation());
                if (boxRep != nullptr)
                {
                    boxRep->PlaceWidget(bounds);
                    // 用 SetTransform 将 Gizmo 平移到末端位置
                    vtkNew<vtkTransform> tcpTransform;
                    tcpTransform->Translate(pos[0], pos[1], pos[2]);
                    boxRep->SetTransform(tcpTransform);
                }
            }

            // 启用 Gizmo 使其可交互
            m_tcp_gizmo_widget->SetEnabled(1);
            m_tcp_gizmo_widget->SetProcessEvents(1);
        }
        // ── 【逆向驱动】TCP Gizmo 初始化结束 ──────────────────────

        if (resetCamera)
        {
            // 中文说明：无论当前是空参考场景还是机器人预览场景，首次展示都统一使用等轴测视角。
            ApplyCameraPreset(1.0, -1.0, 1.0, 0.0, 0.0, 1.0);
        }
    }

    if (m_renderWindow != nullptr)
    {
        m_renderWindow->Render();
        UpdateScaleBar();
    }
#endif
}

void RobotVtkView::UpdateScaleBar()
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer == nullptr || m_scaleBarOverlay == nullptr)
    {
        return;
    }

    vtkCamera* camera = m_renderer->GetActiveCamera();
    if (camera == nullptr)
    {
        return;
    }

    // 中文说明：在视口底部选择两个归一化坐标点，通过 vtkCoordinate 转换到世界坐标，
    // 计算两点世界距离，再圆整为"好看"的刻度值，传递给 QPainter 覆盖层重绘。
    vtkNew<vtkCoordinate> coordLeft, coordRight;
    coordLeft->SetCoordinateSystemToNormalizedViewport();
    coordLeft->SetViewport(m_renderer);
    coordRight->SetCoordinateSystemToNormalizedViewport();
    coordRight->SetViewport(m_renderer);

    const double barLeftVp = 0.10;
    const double barRightVp = 0.42;
    const double barY = 0.07;

    coordLeft->SetValue(barLeftVp, barY);
    double* worldLeft = coordLeft->GetComputedWorldValue(m_renderer);
    coordRight->SetValue(barRightVp, barY);
    double* worldRight = coordRight->GetComputedWorldValue(m_renderer);

    if (worldLeft == nullptr || worldRight == nullptr)
    {
        return;
    }

    double refWorldDist = std::sqrt(
        (worldRight[0] - worldLeft[0]) * (worldRight[0] - worldLeft[0]) +
        (worldRight[1] - worldLeft[1]) * (worldRight[1] - worldLeft[1]) +
        (worldRight[2] - worldLeft[2]) * (worldRight[2] - worldLeft[2]));

    if (!std::isfinite(refWorldDist) || refWorldDist < 1.0e-9)
    {
        return;
    }

    // 圆整为"好看"的刻度值：1×10ⁿ、2×10ⁿ、2.5×10ⁿ、5×10ⁿ 系列
    double expFloor = std::floor(std::log10(refWorldDist));
    double pow10 = std::pow(10.0, expFloor);
    double mantissa = refWorldDist / pow10;
    double niceDist;
    if (mantissa <= 1.0)
        niceDist = pow10;
    else if (mantissa <= 2.0)
        niceDist = 2.0 * pow10;
    else if (mantissa <= 2.5)
        niceDist = 2.5 * pow10;
    else if (mantissa <= 5.0)
        niceDist = 5.0 * pow10;
    else
        niceDist = 10.0 * pow10;

    const QString unit = QStringLiteral("m");
    const double barRatio = niceDist / refWorldDist;

    m_scaleBarNiceDist = niceDist;
    m_scaleBarUnit = unit;

    auto* overlay = static_cast<ScaleBarOverlay*>(m_scaleBarOverlay);
    if (overlay != nullptr)
    {
        overlay->setScale(niceDist, unit, barRatio);
    }
#endif
}

bool RobotVtkView::eventFilter(QObject* watched, QEvent* event)
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_cameraToolbar != nullptr && m_cameraToolbar->parentWidget() != nullptr)
    {
        auto* viewportFrame = m_cameraToolbar->parentWidget();
        if (event->type() == QEvent::MouseMove)
        {
            if (auto* mouseEvent = dynamic_cast<QMouseEvent*>(event))
            {
                QWidget* sourceWidget = qobject_cast<QWidget*>(watched);
                if (sourceWidget != nullptr)
                {
                    const QPoint viewportPos = sourceWidget->mapTo(viewportFrame, mouseEvent->pos());
                    UpdateCameraToolbarVisibility(viewportPos);
                }
            }
        }
        else if (event->type() == QEvent::Leave && watched == m_vtkWidget)
        {
            const QPoint viewportPos = viewportFrame->mapFromGlobal(QCursor::pos());
            UpdateCameraToolbarVisibility(viewportPos);
        }
    }

    if (watched == m_vtkWidget
        && (event->type() == QEvent::MouseButtonRelease
            || event->type() == QEvent::Wheel))
    {
        UpdateScaleBar();
    }

    // 中文说明：viewportFrame 尺寸变化时重新定位比例尺覆盖层到底部。
    if (event->type() == QEvent::Resize && m_scaleBarOverlay != nullptr
        && watched == m_scaleBarOverlay->parent())
    {
        auto* resizeEvent = static_cast<QResizeEvent*>(event);
        int w = resizeEvent->size().width();
        int h = resizeEvent->size().height();
        Q_UNUSED(w);
        m_scaleBarOverlay->setGeometry(8, h - 44, 260, 40);
        PositionCameraToolbar();
        UpdateCameraToolbarVisibility(resizeEvent->size().isValid()
                                          ? m_scaleBarOverlay->parentWidget()->mapFromGlobal(QCursor::pos())
                                          : QPoint(-1, -1));
        PositionAnalysisLayerPanel();
        RaiseViewOverlays();
    }
#else
    Q_UNUSED(watched);
    Q_UNUSED(event);
#endif

    return QWidget::eventFilter(watched, event);
}

void RobotVtkView::ApplyCameraPreset(
    double directionX,
    double directionY,
    double directionZ,
    double upX,
    double upY,
    double upZ)
{
#if defined(ROBOSDP_HAVE_VTK)
    if (m_renderer == nullptr)
    {
        return;
    }

    double directionLength = std::sqrt(directionX * directionX + directionY * directionY + directionZ * directionZ);
    if (directionLength < 1.0e-9)
    {
        directionX = 1.0;
        directionY = -1.0;
        directionZ = 1.0;
        directionLength = std::sqrt(3.0);
    }

    directionX /= directionLength;
    directionY /= directionLength;
    directionZ /= directionLength;

    vtkCamera* camera = m_renderer->GetActiveCamera();
    if (camera == nullptr)
    {
        return;
    }

    // 中文说明：保留当前相机的焦距/距离，只覆写观察方向和上向量。
    // 不再调用 ResetCamera()，避免用户手工缩放后被视角切换按钮强制重置。
    double focalPoint[3] = {0.0, 0.0, 0.0};
    double position[3] = {0.0, 0.0, 1.0};
    camera->GetFocalPoint(focalPoint);
    camera->GetPosition(position);

    double distance = std::sqrt(
        (position[0] - focalPoint[0]) * (position[0] - focalPoint[0]) +
        (position[1] - focalPoint[1]) * (position[1] - focalPoint[1]) +
        (position[2] - focalPoint[2]) * (position[2] - focalPoint[2]));
    if (!std::isfinite(distance) || distance < 1.0e-6)
    {
        distance = 3.0;
    }

    camera->SetPosition(
        focalPoint[0] + directionX * distance,
        focalPoint[1] + directionY * distance,
        focalPoint[2] + directionZ * distance);
    camera->SetViewUp(upX, upY, upZ);
    camera->OrthogonalizeViewUp();
    m_renderer->ResetCameraClippingRange();

    if (m_renderWindow != nullptr)
    {
        UpdateScaleBar();
        m_renderWindow->Render();
    }
#else
    Q_UNUSED(directionX);
    Q_UNUSED(directionY);
    Q_UNUSED(directionZ);
    Q_UNUSED(upX);
    Q_UNUSED(upY);
    Q_UNUSED(upZ);
#endif
}

// =========================================================================
// 【逆向驱动】滚轮关节角度发射（修复版）
// =========================================================================
void RobotVtkView::EmitJointScroll(double deltaDeg)
{
    const QString& linkName = m_current_picked_link;
    if (linkName.isEmpty())
    {
        return;
    }

    // 中文说明：优先使用 m_link_to_joint_index 映射表查关节索引。
    // 该映射由 RebuildLinkToJointMap() 从 m_currentScene.segments 构建，
    // 以子 link 名称（child_link_name）为键，segment 索引（=joint 索引）为值。
    // 支持任意 link 命名约定（URDF 导入、拓扑构型生成等）。
    int jointIndex = -1;
    auto it = m_link_to_joint_index.find(linkName);
    if (it != m_link_to_joint_index.end())
    {
        jointIndex = it->second;
    }
    else
    {
        // 降级：兼容旧式命名规则 "J1", "J2", ...（拓扑构型生成）
        // 以及 "Link_1", "Link_2", ...（旧版 URDF 导入约定）
        if (linkName.startsWith(QLatin1Char('J')))
        {
            bool ok = false;
            const int idx = linkName.mid(1).toInt(&ok);
            if (ok && idx >= 1) jointIndex = idx - 1;
        }
        else if (linkName.startsWith(QStringLiteral("Link_")))
        {
            bool ok = false;
            const int idx = linkName.mid(5).toInt(&ok);
            if (ok && idx >= 1) jointIndex = idx - 1;
        }
    }

    if (jointIndex >= 0)
    {
        emit signalJointAngleScrolled(jointIndex, deltaDeg);
    }
}

// =========================================================================
// 【逆向驱动】link_name → joint_index 映射表重建
// =========================================================================
void RobotVtkView::RebuildLinkToJointMap()
{
    // 中文说明：遍历所有骨架段，以 child_link_name 为键，segment 索引为值。
    // 对于串联机械臂，segment[i] 的子 link 对应的 joint 索引即为 i。
    m_link_to_joint_index.clear();
    for (int i = 0; i < static_cast<int>(m_currentScene.segments.size()); ++i)
    {
        const auto& segment = m_currentScene.segments[i];
        const QString& childName = segment.child_link_name;
        if (!childName.isEmpty())
        {
            // 如果多个 segment 共享同一个子 link 名（分支拓扑），只保留第一个。
            if (m_link_to_joint_index.find(childName) == m_link_to_joint_index.end())
            {
                m_link_to_joint_index[childName] = i;
            }
        }
    }
}

// =========================================================================
// 【逆向驱动】TCP Gizmo 拖动位姿发射
// =========================================================================
void RobotVtkView::EmitTcpDrag(vtkTransform* transform)
{
#if defined(ROBOSDP_HAVE_VTK)
    if (transform == nullptr || !m_showTcpGizmo) return;

    // 提取 vtkTransform 的位置和 Z-Y-X 欧拉角（VTK 原生格式）
    const double* pos = transform->GetPosition();
    const double* orientation = transform->GetOrientation(); // Z-Y-X 度数

    RoboSDP::Kinematics::Dto::CartesianPoseDto pose;
    pose.position_m = {pos[0], pos[1], pos[2]};
    // 注意：VTK GetOrientation() 返回 Z-Y-X Euler 角（度），
    // 当前 IK 求解器统一使用 RPY 约定，此处直接透传。
    // 若后续需要坐标系转换，在此处增加适配逻辑。
    pose.rpy_deg = {orientation[0], orientation[1], orientation[2]};

    emit signalTcpPoseDragged(pose);
#else
    Q_UNUSED(transform);
#endif
}

QString RobotVtkView::BuildStatusText() const
{
#if !defined(ROBOSDP_HAVE_VTK)
    if (!m_currentScene.IsEmpty())
    {
        return QStringLiteral(
            "中央三维主视图区：已加载骨架预览数据（模型 %1，节点 %2，连杆段 %3），"
            "但当前构建未启用 VTK，因此只能显示文字摘要。请检查 ROBOSDP_VTK_DIR 与 VTK 组件配置。")
            .arg(m_currentScene.model_name.isEmpty() ? QStringLiteral("未命名模型") : m_currentScene.model_name)
            .arg(m_currentScene.nodes.size())
            .arg(m_currentScene.segments.size());
    }
#endif

    if (!m_currentScene.IsEmpty())
    {
        const QString skeletonState = m_showSkeleton ? QStringLiteral("开") : QStringLiteral("关");
        const QString visualState = m_showVisualMesh ? QStringLiteral("开") : QStringLiteral("关");
        const QString collisionState = m_showCollisionMesh ? QStringLiteral("开") : QStringLiteral("关");
        const QString jointAxisState = m_showJointAxes ? QStringLiteral("开") : QStringLiteral("关");
        const QString axesState = m_showAxes ? QStringLiteral("开") : QStringLiteral("关");
        const QString groundGridState = m_showGroundGrid ? QStringLiteral("开") : QStringLiteral("关");
        const QString cornerAxesState = m_showCornerAxes ? QStringLiteral("开") : QStringLiteral("关");
        const QString linkLabelState = m_showLinkLabels ? QStringLiteral("开") : QStringLiteral("关");
        const QString jointLabelState = m_showJointLabels ? QStringLiteral("开") : QStringLiteral("关");
        return QStringLiteral("中央三维主视图区：预览模型，骨架=%1，Visual=%2，Collision=%3，关节轴=%4，坐标系=%5，地面网格=%6，角落坐标轴=%7，Link 标签=%8，Joint 标签=%9，模型 %10，节点 %11，连杆段 %12，visual mesh %13，collision mesh %14。")
            .arg(skeletonState)
            .arg(visualState)
            .arg(collisionState)
            .arg(jointAxisState)
            .arg(axesState)
            .arg(groundGridState)
            .arg(cornerAxesState)
            .arg(linkLabelState)
            .arg(jointLabelState)
            .arg(m_currentScene.model_name.isEmpty() ? QStringLiteral("未命名模型") : m_currentScene.model_name)
            .arg(m_currentScene.nodes.size())
            .arg(m_currentScene.segments.size())
            .arg(m_currentScene.visual_geometries.size())
            .arg(m_currentScene.collision_geometries.size());
    }

#if defined(ROBOSDP_HAVE_VTK)
    const QString axesState = m_showAxes ? QStringLiteral("开") : QStringLiteral("关");
    const QString groundGridState = m_showGroundGrid ? QStringLiteral("开") : QStringLiteral("关");
    const QString cornerAxesState = m_showCornerAxes ? QStringLiteral("开") : QStringLiteral("关");
    return QStringLiteral("中央三维主视图区：当前显示默认三维参考场景，坐标系=%1，地面网格=%2，角落坐标轴=%3，等待新的机器人骨架同步。")
        .arg(axesState)
        .arg(groundGridState)
        .arg(cornerAxesState);
#else
    return QStringLiteral(
        "中央三维主视图区：当前构建环境未检测到与 Qt 工具链匹配的 VTK。\n"
        "本轮仍可执行机器人骨架同步，但中央区域只显示文字摘要。");
#endif
}

} // namespace RoboSDP::Desktop::Vtk
