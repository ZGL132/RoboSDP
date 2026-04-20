set(ROBOSDP_DESKTOP_SOURCES
    AppBootstrap.cpp
    AppBootstrap.h
    main.cpp
    MainWindow.cpp
    MainWindow.h
    widgets/ribbon/RibbonBarWidget.cpp
    widgets/ribbon/RibbonBarWidget.h
    widgets/vtk/RobotVtkView.cpp
    widgets/vtk/RobotVtkView.h
    widgets/vtk/VtkSceneBuilder.cpp
    widgets/vtk/VtkSceneBuilder.h
)

set(ROBOSDP_VTK_DIR "" CACHE PATH "可选：手动指定与当前 Qt/MinGW 工具链匹配的 VTK CMake 配置目录")

if(ROBOSDP_VTK_DIR)
    # 中文说明：优先使用显式 CMake 参数，避免多套 VTK 并存时误选到 MSVC/Qt6 等不兼容版本。
    set(VTK_DIR "${ROBOSDP_VTK_DIR}" CACHE PATH "VTK CMake package directory" FORCE)
elseif(DEFINED ENV{ROBOSDP_VTK_DIR} AND NOT "$ENV{ROBOSDP_VTK_DIR}" STREQUAL "")
    # 中文说明：Qt Creator 可以通过环境变量注入 MinGW 版 VTK 的 lib/cmake/vtk-x.y 目录。
    set(VTK_DIR "$ENV{ROBOSDP_VTK_DIR}" CACHE PATH "VTK CMake package directory" FORCE)
elseif(DEFINED ENV{VTK_DIR} AND NOT "$ENV{VTK_DIR}" STREQUAL "")
    # 中文说明：兼容 VTK 官方常用变量，便于本机调试和 CI 脚本复用。
    set(VTK_DIR "$ENV{VTK_DIR}" CACHE PATH "VTK CMake package directory" FORCE)
endif()

set(ROBOSDP_VTK_AUTO_DIR
    "${PROJECT_SOURCE_DIR}/../_thirdparty-build/vtk-qt5-mingw81/install-iogeometry/lib/cmake/vtk-9.2")
set(ROBOSDP_VTK_LEGACY_AUTO_DIR
    "${PROJECT_SOURCE_DIR}/../_thirdparty-build/vtk-qt5-mingw81/install-ready/lib/cmake/vtk-9.2")

if(NOT DEFINED VTK_DIR OR "${VTK_DIR}" STREQUAL "")
    if(EXISTS "${ROBOSDP_VTK_AUTO_DIR}/vtk-config.cmake")
        # 中文说明：优先使用带 IOGeometry/vtkSTLReader 的 VTK 包，保证骨架和 STL Mesh 都能启用。
        set(VTK_DIR "${ROBOSDP_VTK_AUTO_DIR}" CACHE PATH "VTK CMake package directory" FORCE)
    elseif(EXISTS "${ROBOSDP_VTK_LEGACY_AUTO_DIR}/vtk-config.cmake")
        # 中文说明：旧包仍可作为骨架渲染兜底，但它不包含 vtkSTLReader，不能加载 STL Mesh。
        set(VTK_DIR "${ROBOSDP_VTK_LEGACY_AUTO_DIR}" CACHE PATH "VTK CMake package directory" FORCE)
    endif()
endif()

find_package(VTK QUIET COMPONENTS
    CommonColor
    CommonCore
    CommonTransforms
    FiltersSources
    GUISupportQt
    InteractionStyle
    RenderingAnnotation
    RenderingCore
    RenderingOpenGL2
    RenderingUI
    OPTIONAL_COMPONENTS
    IOGeometry
)

if(QT_VERSION_MAJOR GREATER_EQUAL 6)
    qt_add_executable(RoboSDPDesktop
        WIN32
        ${ROBOSDP_DESKTOP_SOURCES}
    )
else()
    add_executable(RoboSDPDesktop WIN32 ${ROBOSDP_DESKTOP_SOURCES})
endif()

target_include_directories(RoboSDPDesktop
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(RoboSDPDesktop
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Widgets
        robosdp_errors
        robosdp_logging
        robosdp_config
        robosdp_infrastructure
        robosdp_repository
        robosdp_requirement
        robosdp_topology
        robosdp_kinematics
        robosdp_dynamics
        robosdp_selection
        robosdp_planning
        robosdp_scheme
)

if(VTK_FOUND)
    target_compile_definitions(RoboSDPDesktop PRIVATE ROBOSDP_HAVE_VTK)
    if(TARGET VTK::IOGeometry)
        # 中文说明：STL Mesh 读取依赖 IOGeometry；缺失时仍启用骨架 VTK，只跳过 Mesh 皮肤渲染。
        target_compile_definitions(RoboSDPDesktop PRIVATE ROBOSDP_HAVE_VTK_IOGEOMETRY)
    else()
        message(WARNING "RoboSDPDesktop: 当前 VTK 缺少 IOGeometry/vtkSTLReader，已启用骨架 VTK 渲染，但会跳过 STL Mesh 静态加载。")
    endif()
    target_link_libraries(RoboSDPDesktop PRIVATE ${VTK_LIBRARIES})
    if(COMMAND vtk_module_autoinit)
        vtk_module_autoinit(
            TARGETS RoboSDPDesktop
            MODULES ${VTK_LIBRARIES})
    endif()
    message(STATUS "RoboSDPDesktop: 已检测到 VTK，中央三维视图将启用 Qt + VTK 渲染分支。")
else()
    message(WARNING "RoboSDPDesktop: 未检测到可用 VTK，已优雅降级为非 VTK 视图。${VTK_NOT_FOUND_MESSAGE} 请通过 -DROBOSDP_VTK_DIR=... 或环境变量 ROBOSDP_VTK_DIR 指定 VTK CMake 目录。")
endif()

if(MINGW)
    message(STATUS "RoboSDPDesktop: 当前 Qt 工具链为 MinGW，VTK 必须使用同一套 MinGW 工具链编译。")
endif()
