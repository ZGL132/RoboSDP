#include <QCoreApplication>
#include <QByteArray>
#include <QDebug>
#include <QFile>
#include <QTemporaryDir>
#include <QTextStream>

#if defined(ROBOSDP_TEST_HAVE_VTK_IOGEOMETRY)
#include <vtkActor.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkSTLReader.h>
#include <vtkTransform.h>
#endif

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

#if !defined(ROBOSDP_TEST_HAVE_VTK)
    qInfo().noquote() << QStringLiteral("[VTK Smoke] 当前构建未启用 VTK，已验证优雅跳过路径。");
    return 0;
#elif !defined(ROBOSDP_TEST_HAVE_VTK_IOGEOMETRY)
    qInfo().noquote()
        << QStringLiteral("[VTK Smoke] 当前 VTK 已可链接，但缺少 IOGeometry/vtkSTLReader，已跳过 STL 读取路径。");
    return 0;
#else
    QTemporaryDir tempDir;
    if (!tempDir.isValid())
    {
        qCritical().noquote() << QStringLiteral("[VTK Smoke] 无法创建临时目录。");
        return 1;
    }

    const QString stlPath = tempDir.filePath(QStringLiteral("single_triangle.stl"));
    QFile stlFile(stlPath);
    if (!stlFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qCritical().noquote() << QStringLiteral("[VTK Smoke] 无法写入 STL 夹具：%1").arg(stlPath);
        return 2;
    }

    {
        QTextStream stream(&stlFile);
        // 中文说明：最小 ASCII STL 只包含一个三角面，用于验证 vtkSTLReader -> Mapper -> Actor -> Transform 链路。
        stream << "solid robosdp_smoke\n";
        stream << "  facet normal 0 0 1\n";
        stream << "    outer loop\n";
        stream << "      vertex 0 0 0\n";
        stream << "      vertex 1 0 0\n";
        stream << "      vertex 0 1 0\n";
        stream << "    endloop\n";
        stream << "  endfacet\n";
        stream << "endsolid robosdp_smoke\n";
    }
    stlFile.close();

    vtkNew<vtkSTLReader> reader;
    const QByteArray stlPathBytes = stlPath.toLocal8Bit();
    reader->SetFileName(stlPathBytes.constData());
    reader->Update();

    vtkPolyData* polyData = reader->GetOutput();
    if (polyData == nullptr || polyData->GetNumberOfPoints() <= 0 || polyData->GetNumberOfPolys() <= 0)
    {
        qCritical().noquote() << QStringLiteral("[VTK Smoke] STL 读取失败或几何为空：%1").arg(stlPath);
        return 3;
    }

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(reader->GetOutputPort());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);

    vtkNew<vtkTransform> transform;
    transform->PostMultiply();
    transform->Identity();
    transform->Scale(1.0, 2.0, 3.0);
    transform->RotateZ(15.0);
    transform->Translate(0.1, 0.2, 0.3);
    actor->SetUserTransform(transform);

    if (actor->GetMapper() == nullptr || actor->GetUserTransform() == nullptr)
    {
        qCritical().noquote() << QStringLiteral("[VTK Smoke] Actor 或 UserTransform 未正确绑定。");
        return 4;
    }

    vtkMatrix4x4* matrix = actor->GetUserTransform()->GetMatrix();
    if (matrix == nullptr)
    {
        qCritical().noquote() << QStringLiteral("[VTK Smoke] Transform 矩阵为空。");
        return 5;
    }

    qInfo().noquote()
        << QStringLiteral("[VTK Smoke] 成功加载 STL: %1，点=%2，面=%3，平移=(%4,%5,%6)")
               .arg(stlPath)
               .arg(polyData->GetNumberOfPoints())
               .arg(polyData->GetNumberOfPolys())
               .arg(matrix->GetElement(0, 3), 0, 'f', 3)
               .arg(matrix->GetElement(1, 3), 0, 'f', 3)
               .arg(matrix->GetElement(2, 3), 0, 'f', 3);
    return 0;
#endif
}
