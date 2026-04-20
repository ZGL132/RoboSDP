#include "apps/desktop-qt/widgets/vtk/VtkSceneBuilder.h"

#include <QDebug>
#include <QFileInfo>

#include <algorithm>
#include <array>
#include <limits>

#if defined(ROBOSDP_HAVE_VTK)
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkLineSource.h>
#include <vtkMatrix4x4.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkSmartPointer.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>
#if defined(ROBOSDP_HAVE_VTK_IOGEOMETRY)
#include <vtkSTLReader.h>
#endif
#endif

namespace RoboSDP::Desktop::Vtk
{

#if defined(ROBOSDP_HAVE_VTK)
namespace
{

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
        return 0.04;
    }

    const auto minBounds = ComputeSceneMinBounds(previewScene);
    const auto maxBounds = ComputeSceneMaxBounds(previewScene);
    const double spanX = maxBounds[0] - minBounds[0];
    const double spanY = maxBounds[1] - minBounds[1];
    const double spanZ = maxBounds[2] - minBounds[2];
    const double maxSpan = std::max({spanX, spanY, spanZ, 0.2});
    return std::max(maxSpan * 0.025, 0.015);
}

double ComputeLabelOffset(double nodeRadius)
{
    return std::max(nodeRadius * 1.8, 0.03);
}

void AddAxes(vtkRenderer* renderer, double axisLength)
{
    vtkNew<vtkAxesActor> axesActor;
    axesActor->SetTotalLength(axisLength, axisLength, axisLength);
    axesActor->SetShaftTypeToCylinder();
    axesActor->SetCylinderRadius(0.02);
    renderer->AddActor(axesActor);
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
 * @brief 将项目统一的 CartesianPoseDto 转换为 VTK 变换对象。
 * @details
 * 这里统一按 Translate -> RotateZ -> RotateY -> RotateX 组装，
 * 使矩阵语义与项目现有的 yaw * pitch * roll 约定保持一致。
 */
vtkSmartPointer<vtkTransform> BuildPoseTransform(const RoboSDP::Kinematics::Dto::CartesianPoseDto& pose)
{
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->PostMultiply();
    transform->Identity();
    transform->Translate(pose.position_m[0], pose.position_m[1], pose.position_m[2]);
    transform->RotateZ(pose.rpy_deg[2]);
    transform->RotateY(pose.rpy_deg[1]);
    transform->RotateX(pose.rpy_deg[0]);
    return transform;
}

/**
 * @brief 为三维缩放构建独立变换。
 * @details 单独拆出 scale 变换，便于最终按 Global * Local * Scale 的顺序拼接矩阵。
 */
vtkSmartPointer<vtkTransform> BuildScaleTransform(const std::array<double, 3>& scale)
{
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->PostMultiply();
    transform->Identity();
    transform->Scale(scale[0], scale[1], scale[2]);
    return transform;
}

/**
 * @brief 组装 Mesh Actor 最终变换矩阵。
 * @details 动态刷新时只替换 globalPose，local_pose 与 scale 来自缓存的 GeometryObjectDto。
 */
vtkSmartPointer<vtkTransform> BuildMeshActorTransform(
    const RoboSDP::Kinematics::Dto::CartesianPoseDto& globalPose,
    const RoboSDP::Kinematics::Dto::GeometryObjectDto& geometry)
{
    const auto globalTransform = BuildPoseTransform(globalPose);
    const auto localTransform = BuildPoseTransform(geometry.local_pose);
    const auto scaleTransform = BuildScaleTransform(geometry.scale);

    vtkSmartPointer<vtkTransform> finalTransform = vtkSmartPointer<vtkTransform>::New();
    finalTransform->PostMultiply();
    finalTransform->Identity();
    finalTransform->Concatenate(globalTransform->GetMatrix());
    finalTransform->Concatenate(localTransform->GetMatrix());
    finalTransform->Concatenate(scaleTransform->GetMatrix());
    return finalTransform;
}

bool IsSupportedStaticMeshFile(const QString& filePath)
{
    const QString suffix = QFileInfo(filePath).suffix().trimmed().toLower();
    return suffix == QStringLiteral("stl");
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
    const auto cachedActor = linkActors.find(geometry.link_name);
    if (cachedActor != linkActors.end() && cachedActor->second != nullptr)
    {
        meshActor = cachedActor->second;
        qInfo().noquote()
            << QStringLiteral("[VTK] 复用缓存 Mesh Actor: %1 -> %2")
                   .arg(geometry.link_name, geometry.absolute_file_path);
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

        vtkNew<vtkPolyDataMapper> meshMapper;
        meshMapper->SetInputConnection(meshReader->GetOutputPort());

        meshActor = vtkSmartPointer<vtkActor>::New();
        meshActor->SetMapper(meshMapper);
        meshActor->GetProperty()->SetColor(colors->GetColor3d("LightSteelBlue").GetData());
        meshActor->GetProperty()->SetOpacity(0.88);
        linkActors[geometry.link_name] = meshActor;
        linkGeometries[geometry.link_name] = geometry;

        qInfo().noquote()
            << QStringLiteral("[VTK] 成功加载 Mesh: %1 -> %2")
                   .arg(geometry.link_name, geometry.absolute_file_path);
    }

    const auto finalTransform = BuildMeshActorTransform(node->world_pose, geometry);
    meshActor->SetUserTransform(finalTransform);

    const vtkMatrix4x4* matrix = finalTransform->GetMatrix();
    qInfo().noquote()
        << QStringLiteral("[VTK] Mesh 最终变换平移: tx=%1 ty=%2 tz=%3")
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
    const RoboSDP::Kinematics::Dto::UrdfPreviewNodeDto& node,
    double labelOffset)
{
    vtkNew<vtkBillboardTextActor3D> labelActor;
    labelActor->SetInput(node.link_name.toStdString().c_str());
    labelActor->SetPosition(
        node.position_m[0] + labelOffset,
        node.position_m[1] + labelOffset * 0.35,
        node.position_m[2] + labelOffset);
    labelActor->GetTextProperty()->SetFontSize(18);
    labelActor->GetTextProperty()->SetColor(0.96, 0.98, 1.0);
    labelActor->GetTextProperty()->SetBold(true);
    renderer->AddActor(labelActor);
}

void AddJointLabel(
    vtkRenderer* renderer,
    const RoboSDP::Kinematics::Dto::UrdfPreviewSegmentDto& segment,
    double labelOffset)
{
    vtkNew<vtkBillboardTextActor3D> labelActor;
    labelActor->SetInput(segment.joint_name.toStdString().c_str());
    labelActor->SetPosition(
        segment.end_position_m[0] - labelOffset * 0.9,
        segment.end_position_m[1] + labelOffset * 0.25,
        segment.end_position_m[2] + labelOffset * 0.55);
    labelActor->GetTextProperty()->SetFontSize(16);
    labelActor->GetTextProperty()->SetColor(1.0, 0.95, 0.55);
    labelActor->GetTextProperty()->SetBold(true);
    renderer->AddActor(labelActor);
}

void AddJointHighlight(
    vtkRenderer* renderer,
    vtkNamedColors* colors,
    const RoboSDP::Kinematics::Dto::UrdfPreviewSegmentDto& segment,
    double nodeRadius,
    double opacity)
{
    vtkNew<vtkSphereSource> jointSphere;
    jointSphere->SetRadius(nodeRadius * 1.35);
    jointSphere->SetThetaResolution(24);
    jointSphere->SetPhiResolution(24);
    jointSphere->SetCenter(
        segment.end_position_m[0],
        segment.end_position_m[1],
        segment.end_position_m[2]);

    vtkNew<vtkPolyDataMapper> jointMapper;
    jointMapper->SetInputConnection(jointSphere->GetOutputPort());

    vtkNew<vtkActor> jointActor;
    jointActor->SetMapper(jointMapper);
    jointActor->GetProperty()->SetColor(colors->GetColor3d("Yellow").GetData());
    jointActor->GetProperty()->SetOpacity(opacity);
    renderer->AddActor(jointActor);
}

} // namespace

void VtkSceneBuilder::BuildMinimalTestScene(vtkRenderer* renderer, bool showAxes)
{
    if (renderer == nullptr)
    {
        return;
    }

    renderer->RemoveAllViewProps();

    vtkNew<vtkNamedColors> colors;
    renderer->SetBackground(colors->GetColor3d("SlateGray").GetData());

    if (showAxes)
    {
        // 中文说明：坐标系只作为空间参考层，由 UI 开关控制，不影响主体几何和相机逻辑。
        AddAxes(renderer, 1.5);
    }

    vtkNew<vtkSphereSource> sphereSource;
    sphereSource->SetRadius(0.35);
    sphereSource->SetThetaResolution(48);
    sphereSource->SetPhiResolution(48);
    sphereSource->SetCenter(0.45, 0.2, 0.25);

    vtkNew<vtkPolyDataMapper> sphereMapper;
    sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

    vtkNew<vtkActor> sphereActor;
    sphereActor->SetMapper(sphereMapper);
    sphereActor->GetProperty()->SetColor(colors->GetColor3d("Orange").GetData());
    renderer->AddActor(sphereActor);

    renderer->ResetCamera();
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
    BuildUrdfPreviewScene(
        renderer,
        previewScene,
        showLinkLabels,
        showJointLabels,
        showAxes,
        temporaryActors,
        temporaryGeometries);
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
    if (renderer == nullptr)
    {
        return;
    }

    if (previewScene.IsEmpty())
    {
        BuildMinimalTestScene(renderer, showAxes);
        return;
    }

    renderer->RemoveAllViewProps();

    vtkNew<vtkNamedColors> colors;
    renderer->SetBackground(colors->GetColor3d("MidnightBlue").GetData());

    const bool hasVisualMeshes = !previewScene.visual_geometries.empty();
    const double skeletonNodeOpacity = hasVisualMeshes ? 0.42 : 1.0;
    const double skeletonSegmentOpacity = hasVisualMeshes ? 0.58 : 1.0;
    const double jointHighlightOpacity = hasVisualMeshes ? 0.18 : 0.30;
    const double nodeRadius = ComputeNodeRadius(previewScene);
    const double labelOffset = ComputeLabelOffset(nodeRadius);
    if (showAxes)
    {
        // 中文说明：URDF 场景中的坐标系是辅助参考，不参与 Mesh/骨架 Actor 缓存。
        AddAxes(renderer, std::max(nodeRadius * 8.0, 0.4));
    }

    // 中文说明：本阶段先支持静态 STL Mesh 渲染。即使 Mesh 缺失或加载失败，也不会影响后续骨架绘制。
    for (const auto& geometry : previewScene.visual_geometries)
    {
        AddStaticMeshActor(renderer, colors, previewScene, geometry, linkActors, linkGeometries);
    }

    // 中文说明：关节高亮层先画半透明包络球，便于在不引入 mesh 的前提下快速识别关节位置。
    for (const auto& segment : previewScene.segments)
    {
        AddJointHighlight(renderer, colors, segment, nodeRadius, jointHighlightOpacity);

        if (showJointLabels && !segment.joint_name.trimmed().isEmpty())
        {
            // 中文说明：关节标签直接显示 joint_name，便于识别每段骨架对应的实际关节编号。
            AddJointLabel(renderer, segment, labelOffset);
        }
    }

    // 中文说明：节点球用于标识每个 link 的参考点，先验证骨架拓扑是否正确。
    for (const auto& node : previewScene.nodes)
    {
        vtkNew<vtkSphereSource> nodeSphere;
        nodeSphere->SetRadius(nodeRadius);
        nodeSphere->SetThetaResolution(24);
        nodeSphere->SetPhiResolution(24);
        nodeSphere->SetCenter(node.position_m[0], node.position_m[1], node.position_m[2]);

        vtkNew<vtkPolyDataMapper> nodeMapper;
        nodeMapper->SetInputConnection(nodeSphere->GetOutputPort());

        vtkNew<vtkActor> nodeActor;
        nodeActor->SetMapper(nodeMapper);
        nodeActor->GetProperty()->SetColor(colors->GetColor3d("Orange").GetData());
        nodeActor->GetProperty()->SetOpacity(skeletonNodeOpacity);
        renderer->AddActor(nodeActor);

        if (showLinkLabels && !node.link_name.trimmed().isEmpty())
        {
            // 中文说明：节点标签直接显示 link 名称，减少导入后需要回到文本区对照的成本。
            AddNodeLabel(renderer, node, labelOffset);
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

        vtkNew<vtkPolyDataMapper> lineMapper;
        lineMapper->SetInputConnection(lineSource->GetOutputPort());

        vtkNew<vtkActor> lineActor;
        lineActor->SetMapper(lineMapper);
        lineActor->GetProperty()->SetColor(colors->GetColor3d("DeepSkyBlue").GetData());
        lineActor->GetProperty()->SetLineWidth(4.0);
        lineActor->GetProperty()->SetOpacity(skeletonSegmentOpacity);
        renderer->AddActor(lineActor);
    }

    renderer->ResetCamera();
}

bool VtkSceneBuilder::UpdateCachedMeshActorTransforms(
    const std::map<QString, RoboSDP::Kinematics::Dto::CartesianPoseDto>& linkWorldPoses,
    std::map<QString, vtkSmartPointer<vtkActor>>& linkActors,
    const std::map<QString, RoboSDP::Kinematics::Dto::GeometryObjectDto>& linkGeometries)
{
    bool updatedAnyActor = false;

    for (const auto& [linkName, worldPose] : linkWorldPoses)
    {
        const auto actorIt = linkActors.find(linkName);
        const auto geometryIt = linkGeometries.find(linkName);
        if (actorIt == linkActors.end() || actorIt->second == nullptr || geometryIt == linkGeometries.end())
        {
            continue;
        }

        // 中文说明：高频路径只替换 GlobalPose，LocalPose 与 Scale 保持首次导入时缓存的几何语义。
        const auto finalTransform = BuildMeshActorTransform(worldPose, geometryIt->second);
        actorIt->second->SetUserTransform(finalTransform);
        updatedAnyActor = true;
    }

    if (updatedAnyActor)
    {
        qInfo().noquote()
            << QStringLiteral("[VTK] 已快速更新 %1 个 link 姿态候选，实际命中 %2 个 Mesh Actor。")
                   .arg(linkWorldPoses.size())
                   .arg(linkActors.size());
    }

    return updatedAnyActor;
}
#endif

} // namespace RoboSDP::Desktop::Vtk
