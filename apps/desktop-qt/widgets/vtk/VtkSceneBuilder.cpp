#include "apps/desktop-qt/widgets/vtk/VtkSceneBuilder.h"

#include <QDebug>
#include <QFileInfo>
#include <QPoint>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

#if defined(ROBOSDP_HAVE_VTK)
#include <vtkActor.h>
#include <vtkArrowSource.h>
#include <vtkAxesActor.h>
#include <vtkTubeFilter.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkCaptionActor2D.h>
#include <vtkLineSource.h>
#include <vtkMatrix4x4.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTextProperty.h>
#include <vtkTextActor.h>       // <--- 【新增这一行】
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#if defined(ROBOSDP_HAVE_VTK_IOGEOMETRY)
#include <vtkSTLReader.h>
#endif
#endif

namespace RoboSDP::Desktop::Vtk
{

#if defined(ROBOSDP_HAVE_VTK)
namespace
{
// 【新增】：跨平台中文字体寻找器
static QString GetChineseFontPath()
{
    const QStringList fontPaths = {
        QStringLiteral("C:/Windows/Fonts/msyh.ttc"),       // Windows: 微软雅黑
        QStringLiteral("C:/Windows/Fonts/simhei.ttf"),     // Windows: 黑体
        QStringLiteral("/System/Library/Fonts/PingFang.ttc"), // macOS: 苹方
        QStringLiteral("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"), // Linux: Noto Sans
        QStringLiteral("/usr/share/fonts/truetype/wqy/wqy-microhei.ttc")          // Linux: 文泉驿
    };
    for (const QString& path : fontPaths)
    {
        if (QFile::exists(path)) return path;
    }
    return QString(); // 没找到则返回空
}

constexpr double kPi = 3.14159265358979323846;

double DegToRad(double value)
{
    return value * kPi / 180.0;
}

double VectorNorm(const std::array<double, 3>& value)
{
    return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

std::array<double, 3> NormalizeVector(
    const std::array<double, 3>& value,
    const std::array<double, 3>& fallback)
{
    const double norm = VectorNorm(value);
    if (!std::isfinite(norm) || norm <= 1.0e-12)
    {
        return fallback;
    }

    return {value[0] / norm, value[1] / norm, value[2] / norm};
}

std::array<double, 3> CrossVector(
    const std::array<double, 3>& left,
    const std::array<double, 3>& right)
{
    return {
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]};
}

std::array<double, 3> ComputeSceneMinBounds(const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene)
{
    std::array<double, 3> bounds {
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max()};

    for (const auto& node : previewScene.nodes)
    {
        for (std::size_t index = 0; index < bounds.size(); ++index)
        {
            bounds[index] = std::min(bounds[index], node.position_m[index]);
        }
    }

    return bounds;
}

std::array<double, 3> ComputeSceneMaxBounds(const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene)
{
    std::array<double, 3> bounds {
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest()};

    for (const auto& node : previewScene.nodes)
    {
        for (std::size_t index = 0; index < bounds.size(); ++index)
        {
            bounds[index] = std::max(bounds[index], node.position_m[index]);
        }
    }

    return bounds;
}

double ComputeNodeRadius(const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene)
{
    if (previewScene.nodes.empty())
    {
        return 0.02;
    }

    const auto minBounds = ComputeSceneMinBounds(previewScene);
    const auto maxBounds = ComputeSceneMaxBounds(previewScene);
    const double spanX = maxBounds[0] - minBounds[0];
    const double spanY = maxBounds[1] - minBounds[1];
    const double spanZ = maxBounds[2] - minBounds[2];
    const double maxSpan = std::max({spanX, spanY, spanZ, 0.2});
    // 🔽🔽🔽 【核心修改：调小比例系数】 🔽🔽🔽
    // 原代码是：return std::max(maxSpan * 0.025, 0.015);
    // 意思是取最大跨度的 2.5%，且最小不低于 15 毫米。
    
    // 【修改 2】：把比例从 0.025 降到 0.012 (即 1.2%)，最小半径保底改为 0.008 (8 毫米)
    return std::max(maxSpan * 0.012, 0.008); 
    // 🔼🔼🔼 【修改结束】 🔼🔼🔼
}

double ComputeLabelOffset(double nodeRadius)
{
    return std::max(nodeRadius * 1.8, 0.03);
}

std::optional<int> ParseIndexedLabel(const QString& label, const QChar prefix)
{
    if (!label.startsWith(prefix))
    {
        return std::nullopt;
    }

    bool ok = false;
    const int index = label.mid(1).toInt(&ok);
    if (!ok || index < 1)
    {
        return std::nullopt;
    }

    return index;
}

QPoint LabelDisplayOffset(const QString& text, bool isJointLabel, bool compactDenseLabels)
{
    const auto index = ParseIndexedLabel(text, QLatin1Char(isJointLabel ? 'J' : 'L'));
    if (!index.has_value())
    {
        return isJointLabel ? QPoint(0, 15) : QPoint(15, 0);
    }

    if (compactDenseLabels && *index >= 5)
    {
        return QPoint(0, 0);
    }

    if (isJointLabel)
    {
        switch ((*index - 1) % 4)
        {
        case 0: return QPoint(-26, 18);
        case 1: return QPoint(24, 18);
        case 2: return QPoint(-30, -18);
        default: return QPoint(28, -18);
        }
    }

    switch ((*index - 1) % 4)
    {
    case 0: return QPoint(22, 8);
    case 1: return QPoint(-28, 10);
    case 2: return QPoint(20, -24);
    default: return QPoint(-30, -22);
    }
}

bool ShouldShowDenseLabel(const QString& text, bool compactDenseLabels, int alwaysShowFromEnd)
{
    if (!compactDenseLabels)
    {
        return true;
    }

    const auto linkIndex = ParseIndexedLabel(text, QLatin1Char('L'));
    if (linkIndex.has_value())
    {
        return *linkIndex <= alwaysShowFromEnd;
    }

    const auto jointIndex = ParseIndexedLabel(text, QLatin1Char('J'));
    if (jointIndex.has_value())
    {
        return *jointIndex <= 4;
    }

    return true;
}

double ComputeSceneMaxSpan(const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene)
{
    if (previewScene.nodes.empty())
    {
        return 1.0;
    }

    const auto minBounds = ComputeSceneMinBounds(previewScene);
    const auto maxBounds = ComputeSceneMaxBounds(previewScene);
    return std::max({
        maxBounds[0] - minBounds[0],
        maxBounds[1] - minBounds[1],
        maxBounds[2] - minBounds[2],
        1.0});
}

void AddAxes(vtkRenderer* renderer, double axisLength)
{
    vtkNew<vtkAxesActor> axesActor;
    const double displayLength = axisLength * 0.62;
    axesActor->SetTotalLength(displayLength, displayLength, displayLength);
    axesActor->SetShaftTypeToCylinder();
    axesActor->SetCylinderRadius(0.006);
    axesActor->SetConeRadius(0.025);
    axesActor->SetSphereRadius(0.018);

    // 中文说明：按 ParaView 常用方向色约定显示：X=红，Y=黄，Z=绿。
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
        vtkTextProperty* textProp = captionActor->GetCaptionTextProperty();

        // 🔽🔽🔽 【新增】：应用中文字体
        const QString fontPath = GetChineseFontPath();
        if (!fontPath.isEmpty()) {
            textProp->SetFontFamily(VTK_FONT_FILE);
            textProp->SetFontFile(fontPath.toStdString().c_str());
        }

        // 🔽🔽🔽 【核心修复：彻底禁用自动缩放】 🔽🔽🔽
        if (captionActor->GetTextActor())
        {
            captionActor->GetTextActor()->SetTextScaleModeToNone();
        }
        // 关闭难看的边框和指引线
        captionActor->BorderOff();
        captionActor->LeaderOff();
        // 🔼🔼🔼 【修复结束】 🔼🔼🔼
        captionActor->GetCaptionTextProperty()->SetFontSize(14);
        captionActor->GetCaptionTextProperty()->SetBold(true);
        captionActor->GetCaptionTextProperty()->SetItalic(false);
        captionActor->GetCaptionTextProperty()->SetShadow(false);
        captionActor->GetCaptionTextProperty()->SetColor(red, green, blue);
    };
    tuneCaption(axesActor->GetXAxisCaptionActor2D(), 1.0, 0.0, 0.0);
    tuneCaption(axesActor->GetYAxisCaptionActor2D(), 1.0, 1.0, 0.0);
    tuneCaption(axesActor->GetZAxisCaptionActor2D(), 0.0, 0.80, 0.0);
    renderer->AddActor(axesActor);
}

void AddGroundGrid(vtkRenderer* renderer, double halfSize, double spacing)
{
    if (renderer == nullptr)
    {
        return;
    }

    const double safeHalfSize = std::max(halfSize, 0.5);
    double safeSpacing = std::max(spacing, 0.05);
    int lineCountPerSide = static_cast<int>(std::ceil(safeHalfSize / safeSpacing));
    if (lineCountPerSide > 60)
    {
        lineCountPerSide = 60;
        safeSpacing = safeHalfSize / static_cast<double>(lineCountPerSide);
    }
    const double gridLimit = static_cast<double>(lineCountPerSide) * safeSpacing;

    const auto isMajorGridCoordinate = [safeSpacing](double coordinate) {
        const double nearestMeter = std::round(coordinate);
        return std::abs(coordinate - nearestMeter) <= std::max(safeSpacing * 0.25, 1.0e-6);
    };

    auto addLine = [renderer](double x1, double y1, double x2, double y2, bool majorLine) {
        vtkNew<vtkLineSource> lineSource;
        lineSource->SetPoint1(x1, y1, 0.0);
        lineSource->SetPoint2(x2, y2, 0.0);

        vtkNew<vtkPolyDataMapper> lineMapper;
        lineMapper->SetInputConnection(lineSource->GetOutputPort());

        vtkNew<vtkActor> lineActor;
        lineActor->SetMapper(lineMapper);
        if (majorLine)
        {
            // 中文说明：参考图中的网格线整体较细，主刻度仅略深，避免破坏蓝紫渐变背景。
            lineActor->GetProperty()->SetColor(0.29, 0.29, 0.34);
            lineActor->GetProperty()->SetOpacity(0.70);
            lineActor->GetProperty()->SetLineWidth(1.0);
        }
        else
        {
            // 中文说明：副网格使用同色系浅灰细线，形成截图中的均匀线框平面。
            lineActor->GetProperty()->SetColor(0.34, 0.34, 0.40);
            lineActor->GetProperty()->SetOpacity(0.56);
            lineActor->GetProperty()->SetLineWidth(1.0);
        }
        renderer->AddActor(lineActor);
    };

    for (int index = -lineCountPerSide; index <= lineCountPerSide; ++index)
    {
        const double coordinate = static_cast<double>(index) * safeSpacing;
        const bool majorLine = isMajorGridCoordinate(coordinate);
        addLine(-gridLimit, coordinate, gridLimit, coordinate, majorLine);
        addLine(coordinate, -gridLimit, coordinate, gridLimit, majorLine);
    }
}

void ApplyStableViewportBackground(vtkRenderer* renderer)
{
    if (renderer == nullptr)
    {
        return;
    }

    // 中文说明：匹配参考图的白色到蓝紫色纵向渐变视口。
    renderer->SetGradientBackground(true);
    renderer->SetBackground(0.18, 0.20, 1.0);
    renderer->SetBackground2(0.97, 0.97, 1.0);
}

/**
 * @brief 通过 link 名称查找预览节点，用于将 Mesh 几何体绑定到共享内核算出的 link 全局位姿。
 */
const RoboSDP::Kinematics::Dto::UrdfPreviewNodeDto* FindNodeByLinkName(
    const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene,
    const QString& linkName)
{
    for (const auto& node : previewScene.nodes)
    {
        if (node.link_name == linkName)
        {
            return &node;
        }
    }

    return nullptr;
}

/**
 * @brief 将项目统一的 CartesianPoseDto 转换为显式 4x4 矩阵。
 * @details
 * 这里不依赖 vtkTransform 的 Pre/PostMultiply 调用顺序，而是直接生成
 * T * Rz(yaw) * Ry(pitch) * Rx(roll)，确保骨架 link 位姿与 mesh 局部 origin
 * 使用同一套列向量矩阵语义。
 */
vtkSmartPointer<vtkMatrix4x4> BuildPoseMatrix(const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    const double roll = DegToRad(pose.rpy_deg[0]);
    const double pitch = DegToRad(pose.rpy_deg[1]);
    const double yaw = DegToRad(pose.rpy_deg[2]);

    const double cx = std::cos(roll);
    const double sx = std::sin(roll);
    const double cy = std::cos(pitch);
    const double sy = std::sin(pitch);
    const double cz = std::cos(yaw);
    const double sz = std::sin(yaw);

    vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
    matrix->Identity();
    matrix->SetElement(0, 0, cz * cy);
    matrix->SetElement(0, 1, cz * sy * sx - sz * cx);
    matrix->SetElement(0, 2, cz * sy * cx + sz * sx);
    matrix->SetElement(1, 0, sz * cy);
    matrix->SetElement(1, 1, sz * sy * sx + cz * cx);
    matrix->SetElement(1, 2, sz * sy * cx - cz * sx);
    matrix->SetElement(2, 0, -sy);
    matrix->SetElement(2, 1, cy * sx);
    matrix->SetElement(2, 2, cy * cx);
    matrix->SetElement(0, 3, pose.position_m[0]);
    matrix->SetElement(1, 3, pose.position_m[1]);
    matrix->SetElement(2, 3, pose.position_m[2]);
    return matrix;
}

std::array<double, 3> TransformDirectionByPose(
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose,
    const std::array<double, 3>& localDirection)
{
    const auto poseMatrix = BuildPoseMatrix(pose);
    std::array<double, 3> worldDirection {
        poseMatrix->GetElement(0, 0) * localDirection[0] +
            poseMatrix->GetElement(0, 1) * localDirection[1] +
            poseMatrix->GetElement(0, 2) * localDirection[2],
        poseMatrix->GetElement(1, 0) * localDirection[0] +
            poseMatrix->GetElement(1, 1) * localDirection[1] +
            poseMatrix->GetElement(1, 2) * localDirection[2],
        poseMatrix->GetElement(2, 0) * localDirection[0] +
            poseMatrix->GetElement(2, 1) * localDirection[1] +
            poseMatrix->GetElement(2, 2) * localDirection[2]};
    return NormalizeVector(worldDirection, {0.0, 0.0, 1.0});
}

std::array<double, 3> ComputeJointAxisWorld(
    const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene,
    const RoboSDP::Kinematics::Dto::UrdfPreviewSegmentDto& segment)
{
    const std::array<double, 3> localAxis = NormalizeVector(segment.joint_axis_xyz, {0.0, 0.0, 1.0});
    const auto* parentNode = FindNodeByLinkName(previewScene, segment.parent_link_name);
    return parentNode != nullptr
        ? TransformDirectionByPose(parentNode->world_pose, localAxis)
        : localAxis;
}

bool IsRotatingJointType(const QString& jointType)
{
    const QString normalizedJointType = jointType.trimmed().toLower();
    return normalizedJointType == QStringLiteral("revolute") ||
           normalizedJointType == QStringLiteral("continuous");
}

vtkSmartPointer<vtkActor> CreateAxisAlignedCylinderActor(
    const std::array<double, 3>& center,
    const std::array<double, 3>& axisWorld,
    double radius,
    double height,
    const std::array<double, 3>& color,
    double opacity)
{
    const std::array<double, 3> axis = NormalizeVector(axisWorld, {0.0, 0.0, 1.0});
    const double halfHeight = height * 0.5;

    vtkNew<vtkLineSource> axisLine;
    axisLine->SetPoint1(
        center[0] - axis[0] * halfHeight,
        center[1] - axis[1] * halfHeight,
        center[2] - axis[2] * halfHeight);
    axisLine->SetPoint2(
        center[0] + axis[0] * halfHeight,
        center[1] + axis[1] * halfHeight,
        center[2] + axis[2] * halfHeight);

    vtkNew<vtkTubeFilter> cylinderTube;
    cylinderTube->SetInputConnection(axisLine->GetOutputPort());
    cylinderTube->SetRadius(radius);
    cylinderTube->SetNumberOfSides(32);
    cylinderTube->CappingOn();

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(cylinderTube->GetOutputPort());

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color[0], color[1], color[2]);
    actor->GetProperty()->SetOpacity(opacity);
    actor->GetProperty()->SetSpecular(0.5);
    actor->GetProperty()->SetSpecularPower(30.0);
    actor->GetProperty()->SetAmbient(0.1);
    return actor;
}

/**
 * @brief 为三维缩放构建独立矩阵。
 * @details 单独拆出 scale 变换，便于最终按 Global * Local * Scale 的顺序拼接矩阵。
 */
vtkSmartPointer<vtkMatrix4x4> BuildScaleMatrix(const std::array<double, 3>& scale)
{
    vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
    matrix->Identity();
    matrix->SetElement(0, 0, scale[0]);
    matrix->SetElement(1, 1, scale[1]);
    matrix->SetElement(2, 2, scale[2]);
    return matrix;
}

/**
 * @brief 组装 Mesh Actor 最终变换矩阵。
 * @details 动态刷新时只替换 globalPose，local_pose 与 scale 来自缓存的 GeometryObjectDto。
 */
vtkSmartPointer<vtkTransform> BuildMeshActorTransform(
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& globalPose,
    const RoboSDP::Kinematics::Dto::GeometryObjectDto& geometry)
{
    const auto globalMatrix = BuildPoseMatrix(globalPose);
    const auto localMatrix = BuildPoseMatrix(geometry.local_pose);
    const auto scaleMatrix = BuildScaleMatrix(geometry.scale);

    vtkSmartPointer<vtkMatrix4x4> globalLocalMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    vtkSmartPointer<vtkMatrix4x4> finalMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    vtkMatrix4x4::Multiply4x4(globalMatrix, localMatrix, globalLocalMatrix);
    vtkMatrix4x4::Multiply4x4(globalLocalMatrix, scaleMatrix, finalMatrix);

    vtkSmartPointer<vtkTransform> finalTransform = vtkSmartPointer<vtkTransform>::New();
    finalTransform->SetMatrix(finalMatrix);
    return finalTransform;
}

bool IsSupportedStaticMeshFile(const QString& filePath)
{
    const QString suffix = QFileInfo(filePath).suffix().trimmed().toLower();
    return suffix == QStringLiteral("stl");
}

QString BuildGeometryActorKey(
    const QString& layerName,
    int geometryIndex,
    const RoboSDP::Kinematics::Dto::GeometryObjectDto& geometry)
{
    return QStringLiteral("%1:%2:%3")
        .arg(layerName, geometry.link_name)
        .arg(geometryIndex);
}

/**
 * @brief 将单个 STL Mesh 以静态零位方式加载到场景中。
 * @details
 * 最终矩阵按 Global * Local * Scale 组装，这样点坐标真实生效顺序就是：
 * 1. 先按 GeometryObjectDto.scale 缩放；
 * 2. 再应用 local_pose；
 * 3. 最后跟随 link 的全局绝对位姿。
 */
void AddStaticMeshActor(
    vtkRenderer* renderer,
    vtkNamedColors* colors,
    const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene,
    const RoboSDP::Kinematics::Dto::GeometryObjectDto& geometry,
    const QString& actorKey,
    const QString& layerName,
    const QString& colorName,
    double opacity,
    bool wireframe,
    std::map<QString, vtkSmartPointer<vtkActor>>& linkActors,
    std::map<QString, RoboSDP::Kinematics::Dto::GeometryObjectDto>& linkGeometries)
{
    if (renderer == nullptr || colors == nullptr)
    {
        return;
    }

    if (!geometry.resource_available)
    {
        qWarning().noquote()
            << QStringLiteral("[VTK] 跳过 Mesh 渲染：link=%1，原因=%2")
                   .arg(geometry.link_name, geometry.status_message.trimmed().isEmpty()
                                                ? QStringLiteral("资源不可用")
                                                : geometry.status_message.trimmed());
        return;
    }

    if (!IsSupportedStaticMeshFile(geometry.absolute_file_path))
    {
        qWarning().noquote()
            << QStringLiteral("[VTK] 暂仅支持 STL Mesh 渲染，已跳过：link=%1 -> %2")
                   .arg(geometry.link_name, geometry.absolute_file_path);
        return;
    }

    const auto* node = FindNodeByLinkName(previewScene, geometry.link_name);
    if (node == nullptr)
    {
        qWarning().noquote()
            << QStringLiteral("[VTK] 未找到 Mesh 所属 link 的全局位姿，已跳过：%1")
                   .arg(geometry.link_name);
        return;
    }

#if defined(ROBOSDP_HAVE_VTK_IOGEOMETRY)
    vtkSmartPointer<vtkActor> meshActor;
    const auto cachedActor = linkActors.find(actorKey);
    if (cachedActor != linkActors.end() && cachedActor->second != nullptr)
    {
        meshActor = cachedActor->second;
        qInfo().noquote()
            << QStringLiteral("[VTK] 复用缓存 %1 Mesh Actor: %2 -> %3")
                   .arg(layerName, geometry.link_name, geometry.absolute_file_path);
    }
    else
    {
        vtkNew<vtkSTLReader> meshReader;
        meshReader->SetFileName(geometry.absolute_file_path.toStdString().c_str());
        meshReader->Update();

        if (meshReader->GetOutput() == nullptr || meshReader->GetOutput()->GetNumberOfPoints() <= 0)
        {
            qWarning().noquote()
                << QStringLiteral("[VTK] STL Mesh 加载失败或为空，已跳过：%1")
                       .arg(geometry.absolute_file_path);
            return;
        }

        double rawBounds[6] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        meshReader->GetOutput()->GetBounds(rawBounds);

        vtkNew<vtkPolyDataMapper> meshMapper;
        meshMapper->SetInputConnection(meshReader->GetOutputPort());

        meshActor = vtkSmartPointer<vtkActor>::New();
        meshActor->SetMapper(meshMapper);
        const std::string colorNameStd = colorName.toStdString();
        meshActor->GetProperty()->SetColor(colors->GetColor3d(colorNameStd.c_str()).GetData());
        meshActor->GetProperty()->SetOpacity(opacity);
        if (wireframe)
        {
            meshActor->GetProperty()->SetRepresentationToWireframe();
            meshActor->GetProperty()->SetLineWidth(1.5);
        }
        linkActors[actorKey] = meshActor;
        linkGeometries[actorKey] = geometry;

        qInfo().noquote()
            << QStringLiteral("[VTK] 成功加载 %1 Mesh: link=%2 file=%3 bounds=[%4,%5,%6,%7,%8,%9] local_xyz=(%10,%11,%12) scale=(%13,%14,%15)")
                   .arg(layerName, geometry.link_name, geometry.absolute_file_path)
                   .arg(rawBounds[0], 0, 'f', 6)
                   .arg(rawBounds[1], 0, 'f', 6)
                   .arg(rawBounds[2], 0, 'f', 6)
                   .arg(rawBounds[3], 0, 'f', 6)
                   .arg(rawBounds[4], 0, 'f', 6)
                   .arg(rawBounds[5], 0, 'f', 6)
                   .arg(geometry.local_pose.position_m[0], 0, 'f', 6)
                   .arg(geometry.local_pose.position_m[1], 0, 'f', 6)
                   .arg(geometry.local_pose.position_m[2], 0, 'f', 6)
                   .arg(geometry.scale[0], 0, 'f', 6)
                   .arg(geometry.scale[1], 0, 'f', 6)
                   .arg(geometry.scale[2], 0, 'f', 6);
    }

    const auto finalTransform = BuildMeshActorTransform(node->world_pose, geometry);
    meshActor->SetUserTransform(finalTransform);

    const vtkMatrix4x4* matrix = finalTransform->GetMatrix();
    qInfo().noquote()
        << QStringLiteral("[VTK] %1 Mesh 最终变换: link=%2 actor=%3 link_xyz=(%4,%5,%6) tx=%7 ty=%8 tz=%9")
               .arg(layerName, geometry.link_name, actorKey)
               .arg(node->world_pose.position_m[0], 0, 'f', 6)
               .arg(node->world_pose.position_m[1], 0, 'f', 6)
               .arg(node->world_pose.position_m[2], 0, 'f', 6)
               .arg(matrix->GetElement(0, 3), 0, 'f', 6)
               .arg(matrix->GetElement(1, 3), 0, 'f', 6)
               .arg(matrix->GetElement(2, 3), 0, 'f', 6);

    renderer->AddActor(meshActor);
#else
    Q_UNUSED(previewScene);
    Q_UNUSED(linkActors);
    Q_UNUSED(linkGeometries);
    qWarning().noquote()
        << QStringLiteral("[VTK] 当前 VTK 缺少 IOGeometry/vtkSTLReader，已跳过 STL Mesh：link=%1 -> %2")
               .arg(geometry.link_name, geometry.absolute_file_path);
#endif
}

void AddNodeLabel(
    vtkRenderer* renderer,
    const QString& linkName,
    const std::array<double, 3>& position,
    const QPoint& displayOffset,
    vtkRenderer* labelRenderer = nullptr)
{
    vtkNew<vtkBillboardTextActor3D> labelActor;
    labelActor->SetInput(linkName.toStdString().c_str());
    labelActor->SetPosition(position[0], position[1], position[2]);
    vtkTextProperty* textProp = labelActor->GetTextProperty();

    const QString fontPath = GetChineseFontPath();
    if (!fontPath.isEmpty()) {
        textProp->SetFontFamily(VTK_FONT_FILE);
        textProp->SetFontFile(fontPath.toStdString().c_str());
    }
    labelActor->GetTextProperty()->SetFontSize(13);
    labelActor->GetTextProperty()->SetColor(0.74, 0.84, 0.92);
    labelActor->GetTextProperty()->SetBold(true);
    labelActor->GetTextProperty()->SetShadow(false);

    labelActor->SetDisplayOffset(displayOffset.x(), displayOffset.y());

    vtkRenderer* target = labelRenderer != nullptr ? labelRenderer : renderer;
    target->AddActor(labelActor);
}

std::array<double, 3> ComputeLinkLabelPosition(
    const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene,
    const RoboSDP::Kinematics::Dto::UrdfPreviewNodeDto& node,
    double labelOffset)
{
    for (const auto& segment : previewScene.segments)
    {
        if (segment.child_link_name != node.link_name)
        {
            continue;
        }

        const std::array<double, 3> linkDirection = NormalizeVector(
            {
                segment.end_position_m[0] - segment.start_position_m[0],
                segment.end_position_m[1] - segment.start_position_m[1],
                segment.end_position_m[2] - segment.start_position_m[2]},
            {1.0, 0.0, 0.0});
        const std::array<double, 3> helper =
            std::abs(linkDirection[2]) < 0.9 ? std::array<double, 3>{0.0, 0.0, 1.0}
                                             : std::array<double, 3>{0.0, 1.0, 0.0};
        const std::array<double, 3> sideOffset = NormalizeVector(CrossVector(linkDirection, helper), {0.0, 1.0, 0.0});

        // 中文说明：Link 标签放在父子 link 线段中点附近，并轻微侧向偏移，避免与关节端点标签重叠。
        return {
            (segment.start_position_m[0] + segment.end_position_m[0]) * 0.5 + sideOffset[0] * labelOffset * 0.45,
            (segment.start_position_m[1] + segment.end_position_m[1]) * 0.5 + sideOffset[1] * labelOffset * 0.45,
            (segment.start_position_m[2] + segment.end_position_m[2]) * 0.5 + sideOffset[2] * labelOffset * 0.45};
    }

    return {
        node.position_m[0] + labelOffset * 0.35,
        node.position_m[1] - labelOffset * 0.35,
        node.position_m[2] + labelOffset * 0.35};
}

void AddJointLabel(
    vtkRenderer* renderer,
    const QString& text,
    const std::array<double, 3>& position,
    const QPoint& displayOffset,
    vtkRenderer* labelRenderer = nullptr)
{
    vtkNew<vtkBillboardTextActor3D> labelActor;
    labelActor->SetInput(text.toStdString().c_str());
    labelActor->SetPosition(position[0], position[1], position[2]);
    vtkTextProperty* textProp = labelActor->GetTextProperty();

    const QString fontPath = GetChineseFontPath();
    if (!fontPath.isEmpty()) {
        textProp->SetFontFamily(VTK_FONT_FILE);
        textProp->SetFontFile(fontPath.toStdString().c_str());
    }
    labelActor->GetTextProperty()->SetFontSize(12);
    labelActor->GetTextProperty()->SetColor(0.84, 0.78, 0.42);
    labelActor->GetTextProperty()->SetBold(false);
    labelActor->GetTextProperty()->SetShadow(false);

    labelActor->SetDisplayOffset(displayOffset.x(), displayOffset.y());

    vtkRenderer* target = labelRenderer != nullptr ? labelRenderer : renderer;
    target->AddActor(labelActor);
}

void AddJointHighlight(
    vtkRenderer* renderer,
    const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene,
    const RoboSDP::Kinematics::Dto::UrdfPreviewSegmentDto& segment,
    double nodeRadius,
    double opacity)
{
    if (renderer == nullptr || !IsRotatingJointType(segment.joint_type))
    {
        return;
    }

    const std::array<double, 3> axisWorld = ComputeJointAxisWorld(previewScene, segment);
    const auto jointActor = CreateAxisAlignedCylinderActor(
        segment.start_position_m,
        axisWorld,
        std::max(nodeRadius * 0.68, 0.008),
        std::max(nodeRadius * 2.70, 0.036),
        {1.0, 1.0, 0.0},
        opacity);

    // 中文说明：给关节圆柱添加轻微高光，便于和连杆圆管区分。
    if (opacity > 0.9)
    { // 如果是不透明状态，强化光影
        jointActor->GetProperty()->SetSpecular(0.5);
        jointActor->GetProperty()->SetSpecularPower(30.0);
        jointActor->GetProperty()->SetAmbient(0.1);
    }

    renderer->AddActor(jointActor);
}

void AddJointAxisArrow(
    vtkRenderer* renderer,
    const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene,
    const RoboSDP::Kinematics::Dto::UrdfPreviewSegmentDto& segment,
    double nodeRadius)
{
    if (renderer == nullptr)
    {
        return;
    }

    if (!IsRotatingJointType(segment.joint_type))
    {
        return;
    }

    const auto* parentNode = FindNodeByLinkName(previewScene, segment.parent_link_name);
    if (parentNode == nullptr)
    {
        return;
    }

    const std::array<double, 3> localAxis = NormalizeVector(segment.joint_axis_xyz, {0.0, 0.0, 1.0});
    const std::array<double, 3> axisWorld = TransformDirectionByPose(parentNode->world_pose, localAxis);
    const std::array<double, 3> helper =
        std::abs(axisWorld[2]) < 0.92 ? std::array<double, 3>{0.0, 0.0, 1.0}
                                      : std::array<double, 3>{0.0, 1.0, 0.0};
    const std::array<double, 3> basisY =
        NormalizeVector(CrossVector(helper, axisWorld), {0.0, 1.0, 0.0});
    const std::array<double, 3> basisZ =
        NormalizeVector(CrossVector(axisWorld, basisY), {0.0, 0.0, 1.0});
    const double axisLength = std::max(nodeRadius * 5.0, 0.08);

    vtkNew<vtkArrowSource> arrowSource;
    arrowSource->SetTipResolution(24);
    arrowSource->SetShaftResolution(18);
    arrowSource->SetTipLength(0.25);
    arrowSource->SetTipRadius(0.04);
    arrowSource->SetShaftRadius(0.012);

    vtkNew<vtkMatrix4x4> axisMatrix;
    axisMatrix->Identity();
    for (int row = 0; row < 3; ++row)
    {
        axisMatrix->SetElement(row, 0, axisWorld[static_cast<std::size_t>(row)] * axisLength);
        axisMatrix->SetElement(row, 1, basisY[static_cast<std::size_t>(row)] * axisLength);
        axisMatrix->SetElement(row, 2, basisZ[static_cast<std::size_t>(row)] * axisLength);
        axisMatrix->SetElement(row, 3, segment.start_position_m[static_cast<std::size_t>(row)]);
    }

    vtkNew<vtkTransform> axisTransform;
    axisTransform->SetMatrix(axisMatrix);

    vtkNew<vtkTransformPolyDataFilter> transformFilter;
    transformFilter->SetInputConnection(arrowSource->GetOutputPort());
    transformFilter->SetTransform(axisTransform);

    vtkNew<vtkPolyDataMapper> axisMapper;
    axisMapper->SetInputConnection(transformFilter->GetOutputPort());

    vtkNew<vtkActor> axisActor;
    axisActor->SetMapper(axisMapper);
    axisActor->GetProperty()->SetColor(0.25, 1.0, 0.20);
    axisActor->GetProperty()->SetOpacity(0.92);
    // 【新增】：禁用光照，让坐标轴保持纯色高亮，不受场景阴影影响
    axisActor->GetProperty()->SetLighting(false);
    renderer->AddActor(axisActor);
}

} // namespace

void VtkSceneBuilder::BuildMinimalTestScene(
    vtkRenderer* renderer,
    bool showAxes,
    bool showGroundGrid,
    bool resetCamera)
{
    if (renderer == nullptr)
    {
        return;
    }

    renderer->RemoveAllViewProps();

    vtkNew<vtkNamedColors> colors;
    ApplyStableViewportBackground(renderer);

    if (showGroundGrid)
    {
        // 中文说明：地面网格是独立空间参考层，由顶部“视图”页签单独控制。
        AddGroundGrid(renderer, 5.0, 0.5);
    }

    if (showAxes)
    {
        // 中文说明：世界坐标系仅表达模型空间原点方向，不再捆绑地面网格显示。
        AddAxes(renderer, 1.5);
    }

    if (resetCamera)
    {
        renderer->ResetCamera();
    }
}

void VtkSceneBuilder::BuildUrdfPreviewScene(
    vtkRenderer* renderer,
    const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene,
    bool showLinkLabels,
    bool showJointLabels,
    bool showAxes)
{
    std::map<QString, vtkSmartPointer<vtkActor>> temporaryActors;
    std::map<QString, RoboSDP::Kinematics::Dto::GeometryObjectDto> temporaryGeometries;
    UrdfPreviewDisplayOptions displayOptions;
    displayOptions.show_link_labels = showLinkLabels;
    displayOptions.show_joint_labels = showJointLabels;
    displayOptions.show_axes = showAxes;
    displayOptions.show_ground_grid = showAxes;
    BuildUrdfPreviewScene(
        renderer,
        previewScene,
        displayOptions,
        temporaryActors,
        temporaryGeometries);
}

void VtkSceneBuilder::BuildUrdfPreviewScene(
    vtkRenderer* renderer,
    const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene,
    const UrdfPreviewDisplayOptions& displayOptions)
{
    std::map<QString, vtkSmartPointer<vtkActor>> temporaryActors;
    std::map<QString, RoboSDP::Kinematics::Dto::GeometryObjectDto> temporaryGeometries;
    BuildUrdfPreviewScene(renderer, previewScene, displayOptions, temporaryActors, temporaryGeometries);
}

void VtkSceneBuilder::BuildUrdfPreviewScene(
    vtkRenderer* renderer,
    const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene,
    bool showLinkLabels,
    bool showJointLabels,
    bool showAxes,
    std::map<QString, vtkSmartPointer<vtkActor>>& linkActors,
    std::map<QString, RoboSDP::Kinematics::Dto::GeometryObjectDto>& linkGeometries)
{
    UrdfPreviewDisplayOptions displayOptions;
    displayOptions.show_link_labels = showLinkLabels;
    displayOptions.show_joint_labels = showJointLabels;
    displayOptions.show_axes = showAxes;
    displayOptions.show_ground_grid = showAxes;
    BuildUrdfPreviewScene(renderer, previewScene, displayOptions, linkActors, linkGeometries);
}

void VtkSceneBuilder::BuildUrdfPreviewScene(
    vtkRenderer* renderer,
    const RoboSDP::Kinematics::Dto::UrdfPreviewSceneDto& previewScene,
    const UrdfPreviewDisplayOptions& displayOptions,
    std::map<QString, vtkSmartPointer<vtkActor>>& linkActors,
    std::map<QString, RoboSDP::Kinematics::Dto::GeometryObjectDto>& linkGeometries,
    vtkRenderer* labelRenderer)
{
    if (renderer == nullptr)
    {
        return;
    }

    if (previewScene.IsEmpty())
    {
        BuildMinimalTestScene(
            renderer,
            displayOptions.show_axes,
            displayOptions.show_ground_grid,
            displayOptions.reset_camera);
        return;
    }

    renderer->RemoveAllViewProps();
    if (labelRenderer != nullptr)
    {
        labelRenderer->RemoveAllViewProps();
    }

    vtkNew<vtkNamedColors> colors;
    ApplyStableViewportBackground(renderer);

    const bool hasVisualMeshes = displayOptions.show_visual_meshes && !previewScene.visual_geometries.empty();
    const double skeletonSegmentOpacity = hasVisualMeshes ? 0.58 : 1.0;
    const double jointHighlightOpacity = 1.0;
    const double nodeRadius = ComputeNodeRadius(previewScene);
    const double labelOffset = ComputeLabelOffset(nodeRadius);
    if (displayOptions.show_ground_grid)
    {
        // 中文说明：URDF 场景中的地面网格是辅助尺度参考，不参与 Mesh/骨架 Actor 缓存。
        const double gridHalfSize = std::max(ComputeSceneMaxSpan(previewScene) * 0.9, 1.0);
        const double gridSpacing = gridHalfSize <= 1.5 ? 0.1 : 0.2;
        AddGroundGrid(renderer, gridHalfSize, gridSpacing);
    }

    if (displayOptions.show_axes)
    {
        // 中文说明：世界坐标系与地面网格拆分控制，便于用户保留网格但隐藏大坐标轴。
        AddAxes(renderer, std::max(nodeRadius * 8.0, 0.4));
    }

    // 中文说明：本阶段先支持静态 STL Mesh 渲染。即使 Mesh 缺失或加载失败，也不会影响后续骨架绘制。
    if (displayOptions.show_visual_meshes)
    {
        for (int index = 0; index < static_cast<int>(previewScene.visual_geometries.size()); ++index)
        {
            const auto& geometry = previewScene.visual_geometries.at(static_cast<std::size_t>(index));
            AddStaticMeshActor(
                renderer,
                colors,
                previewScene,
                geometry,
                BuildGeometryActorKey(QStringLiteral("visual"), index, geometry),
                QStringLiteral("visual"),
                QStringLiteral("LightSteelBlue"),
                0.88,
                false,
                linkActors,
                linkGeometries);
        }
    }

    if (displayOptions.show_collision_meshes)
    {
        for (int index = 0; index < static_cast<int>(previewScene.collision_geometries.size()); ++index)
        {
            const auto& geometry = previewScene.collision_geometries.at(static_cast<std::size_t>(index));
            AddStaticMeshActor(
                renderer,
                colors,
                previewScene,
                geometry,
                BuildGeometryActorKey(QStringLiteral("collision"), index, geometry),
                QStringLiteral("collision"),
                QStringLiteral("Tomato"),
                0.36,
                true,
                linkActors,
                linkGeometries);
        }
    }

    if (displayOptions.show_joint_axes)
    {
        // 中文说明：关节轴是独立诊断层，即使隐藏骨架也允许保留，便于检查 URDF axis 与几何姿态。
        for (const auto& segment : previewScene.segments)
        {
            AddJointAxisArrow(renderer, previewScene, segment, nodeRadius);
        }
    }

    if (displayOptions.show_skeleton)
    {
        // 🔽🔽🔽 【新增】：重叠位置跟踪器，用于文字智能排版 🔽🔽🔽
        std::vector<std::pair<std::array<double, 3>, int>> occupiedPositions;
        auto getShiftedPosition = [&occupiedPositions](const std::array<double, 3>& basePos, double offsetStep) {
            for (auto& occ : occupiedPositions) {
                double dx = occ.first[0] - basePos[0];
                double dy = occ.first[1] - basePos[1];
                double dz = occ.first[2] - basePos[2];
                // 如果两点距离极小（小于0.1毫米），认为物理重叠
                if (std::sqrt(dx*dx + dy*dy + dz*dz) < 1e-4) {
                    occ.second++; // 增加重叠计数
                    // 沿着 Z 轴往下排布，形成清晰的文字列表
                    return std::array<double, 3>{basePos[0], basePos[1], basePos[2] - occ.second * offsetStep};
                }
            }
            // 第一次出现在该位置，记录下来
            occupiedPositions.push_back({basePos, 0});
            return basePos;
        };
        // 🔼🔼🔼 【新增结束】 🔼🔼🔼

        // 中文说明：关节高亮层先画半透明包络球，便于在不引入 mesh 的前提下快速识别关节位置。
        int jointIndex = 0;
        for (const auto& segment : previewScene.segments)
        {
            AddJointHighlight(renderer, previewScene, segment, nodeRadius, jointHighlightOpacity);

            if (displayOptions.show_joint_labels && segment.joint_type != QStringLiteral("fixed"))
            {
                ++jointIndex;
                const QString jointLabel = QStringLiteral("J%1").arg(jointIndex);
                if (ShouldShowDenseLabel(jointLabel, displayOptions.compact_dense_labels, 5))
                {
                    auto shiftedPos = getShiftedPosition(segment.start_position_m, labelOffset * 0.8);
                    AddJointLabel(
                        renderer,
                        jointLabel,
                        shiftedPos,
                        LabelDisplayOffset(jointLabel, true, displayOptions.compact_dense_labels),
                        labelRenderer);
                }
            }
        }

        // 中文说明：link 标签仍使用节点位置，但不再绘制额外节点实体，避免与关节轴向圆柱混淆。
        int linkIndex = 0;
        for (const auto& node : previewScene.nodes)
        {
            ++linkIndex;
            if (displayOptions.show_link_labels)
            {
                const QString linkLabel = node.link_name.trimmed().isEmpty()
                    ? QStringLiteral("L%1").arg(linkIndex)
                    : node.link_name.trimmed();
                if (ShouldShowDenseLabel(linkLabel, displayOptions.compact_dense_labels, 5))
                {
                    auto basePos = ComputeLinkLabelPosition(previewScene, node, labelOffset);
                    auto shiftedPos = getShiftedPosition(basePos, labelOffset * 0.8);
                    AddNodeLabel(
                        renderer,
                        linkLabel,
                        shiftedPos,
                        LabelDisplayOffset(linkLabel, false, displayOptions.compact_dense_labels),
                        labelRenderer);
                }
            }
        }

        // 中文说明：父子 link 之间的线段用于还原关节拓扑骨架，满足本轮 3D 预览目标。
        for (const auto& segment : previewScene.segments)
        {
            vtkNew<vtkLineSource> lineSource;
            lineSource->SetPoint1(
                segment.start_position_m[0],
                segment.start_position_m[1],
                segment.start_position_m[2]);
            lineSource->SetPoint2(
                segment.end_position_m[0],
                segment.end_position_m[1],
                segment.end_position_m[2]);

            // 【新增】：使用 vtkTubeFilter 将线条变成 3D 圆管
            vtkNew<vtkTubeFilter> tubeFilter;
            tubeFilter->SetInputConnection(lineSource->GetOutputPort());
            tubeFilter->SetRadius(nodeRadius * 0.5); // 连杆稍微比关节圆柱细一点
            tubeFilter->SetNumberOfSides(16);        // 圆柱切面数，保证圆滑

            vtkNew<vtkPolyDataMapper> lineMapper;
            lineMapper->SetInputConnection(tubeFilter->GetOutputPort()); // 改为接收 tubeFilter

            vtkNew<vtkActor> lineActor;
            lineActor->SetMapper(lineMapper);
            lineActor->GetProperty()->SetColor(colors->GetColor3d("DeepSkyBlue").GetData());
            // 【新增】：添加高光和漫反射，增强 3D 金属/塑料质感
            lineActor->GetProperty()->SetDiffuse(0.8);
            lineActor->GetProperty()->SetSpecular(0.8);
            lineActor->GetProperty()->SetSpecularPower(50.0);
            lineActor->GetProperty()->SetAmbient(0.15);
            lineActor->GetProperty()->SetOpacity(skeletonSegmentOpacity);
            renderer->AddActor(lineActor);
        }
    }

    if (displayOptions.reset_camera)
    {
        renderer->ResetCamera();
    }
}

bool VtkSceneBuilder::UpdateCachedMeshActorTransforms(
    const std::map<QString, RoboSDP::Kinematics::Dto::CartesianPoseDto>& linkWorldPoses,
    std::map<QString, vtkSmartPointer<vtkActor>>& linkActors,
    const std::map<QString, RoboSDP::Kinematics::Dto::GeometryObjectDto>& linkGeometries)
{
    bool updatedAnyActor = false;
    int updatedActorCount = 0;

    for (const auto& [actorKey, geometry] : linkGeometries)
    {
        const auto actorIt = linkActors.find(actorKey);
        const auto poseIt = linkWorldPoses.find(geometry.link_name);
        if (actorIt == linkActors.end() || actorIt->second == nullptr || poseIt == linkWorldPoses.end())
        {
            continue;
        }

        // 中文说明：高频路径只替换 GlobalPose，LocalPose 与 Scale 保持首次导入时缓存的几何语义。
        const auto finalTransform = BuildMeshActorTransform(poseIt->second, geometry);
        actorIt->second->SetUserTransform(finalTransform);
        updatedAnyActor = true;
        ++updatedActorCount;
    }

    if (updatedAnyActor)
    {
        qInfo().noquote()
            << QStringLiteral("[VTK] 已快速更新 %1 个 link 姿态候选，实际命中 %2/%3 个 Mesh Actor。")
                   .arg(linkWorldPoses.size())
                   .arg(updatedActorCount)
                   .arg(linkActors.size());
    }

    return updatedAnyActor;
}
#endif

} // namespace RoboSDP::Desktop::Vtk
