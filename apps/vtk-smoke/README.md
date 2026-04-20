# VTK Smoke

这个目录提供一个独立的 Qt + VTK 最小验证工程。

目标：
- 验证 CMake 是否能找到 VTK
- 验证 QVTKOpenGLNativeWidget + vtkGenericOpenGLRenderWindow 是否能正常创建
- 验证最小三维对象是否能显示

运行前提：
1. 当前仓库使用 Qt Widgets + C++17。
2. 如果当前工具链是 Qt MinGW，那么 VTK 也必须由同一套 MinGW 工具链编译。
3. 若使用 CMake 手动指定依赖，建议设置 `VTK_DIR` 指向 VTK 的 CMake 配置目录。

推荐构建方式：
1. 配置：
   - `cmake -S . -B build-vtk-smoke -G Ninja -DCMAKE_PREFIX_PATH="D:/software/Qt/5.15.2/mingw81_64"`
   - 若 CMake 无法自动找到 VTK，可补：
     - `-DROBOSDP_VTK_DIR="你的 VTK CMake 配置目录"`
     - 或预先设置环境变量 `VTK_DIR`
2. 构建：
   - `cmake --build build-vtk-smoke --target RoboSDPVtkSmoke`
3. 运行：
   - `build-vtk-smoke/apps/vtk-smoke/RoboSDPVtkSmoke.exe`

运行结果说明：
- 若成功检测到 VTK，将显示一个最小球体对象。
- 若未检测到 VTK，将显示 fallback 说明文本，用于提示依赖未接通。

当前机器的排查结论：
- 当前主项目 Qt 工具链为 `Qt 5.15.2 + MinGW 8.1.0`。
- 已检查 `D:/software` 和 `D:/software/Miniconda3` 下的常见 VTK 安装位置，未发现现成的 `VTKConfig.cmake`。
- 本机存在 `C:/msys64`，且仓库中有 `mingw-w64-x86_64-vtk 9.4.2-2`。
- 但该 MSYS2 VTK 对 Qt 渲染的可选依赖是 `mingw-w64-x86_64-qt6-base`，不是当前项目使用的 Qt 5。
- 因此它不能视为“与当前 Qt MinGW 匹配的 VTK”直接用于本项目。
- 当前已通过源码方式构建出一套可运行的最小 VTK，并将稳定安装目录收口到：
  - `D:/10_Source_Repos/21_robot/_thirdparty-build/vtk-qt5-mingw81/install-ready`
  - 对应 CMake 包目录为：
    - `D:/10_Source_Repos/21_robot/_thirdparty-build/vtk-qt5-mingw81/install-ready/lib/cmake/vtk-9.2`

当前推荐接线方式：
1. 配置 `vtk-smoke` 时优先显式指定：
   - `-DROBOSDP_VTK_DIR="D:/10_Source_Repos/21_robot/_thirdparty-build/vtk-qt5-mingw81/install-ready/lib/cmake/vtk-9.2"`
2. 运行 `RoboSDPVtkSmoke.exe` 前，建议把以下目录加入 `PATH`：
   - `D:/software/Qt/5.15.2/mingw81_64/bin`
   - `D:/10_Source_Repos/21_robot/_thirdparty-build/vtk-qt5-mingw81/install-ready/bin`

结论：
- 若要真正跑通当前项目的 QVTKOpenGLNativeWidget 路径，需要一套与 `Qt 5.15.2 + MinGW 8.1.0` 匹配编译的 VTK。
- 当前这条源码构建路径已经验证可用于最小 `vtk-smoke`。
- 后续若要并入主项目，建议继续复用 `install-ready`，不要再依赖仓库内的临时安装目录。
