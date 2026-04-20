set(ROBOSDP_VTK_SMOKE_SOURCES
    main.cpp
    VtkSmokeWindow.cpp
    VtkSmokeWindow.h
)

set(ROBOSDP_VTK_DIR "" CACHE PATH "可选：手动指定与当前 Qt 工具链匹配的 VTK CMake 配置目录")

if(ROBOSDP_VTK_DIR)
    # 优先允许调用方显式指定 VTK_DIR，避免在多套工具链并存时误选错误的 VTK。
    set(VTK_DIR "${ROBOSDP_VTK_DIR}" CACHE PATH "VTK CMake package directory" FORCE)
elseif(DEFINED ENV{ROBOSDP_VTK_DIR} AND NOT "$ENV{ROBOSDP_VTK_DIR}" STREQUAL "")
    # 标准化入口：与桌面端保持一致，支持通过 ROBOSDP_VTK_DIR 指定 MinGW 版 VTK。
    set(VTK_DIR "$ENV{ROBOSDP_VTK_DIR}" CACHE PATH "VTK CMake package directory" FORCE)
elseif(DEFINED ENV{VTK_DIR} AND NOT "$ENV{VTK_DIR}" STREQUAL "")
    # 若外部已提供 VTK_DIR 环境变量，则复用该入口，便于本机手动联调。
    set(VTK_DIR "$ENV{VTK_DIR}" CACHE PATH "VTK CMake package directory" FORCE)
endif()

set(ROBOSDP_VTK_AUTO_DIR
    "${PROJECT_SOURCE_DIR}/../_thirdparty-build/vtk-qt5-mingw81/install-iogeometry/lib/cmake/vtk-9.2")
set(ROBOSDP_VTK_LEGACY_AUTO_DIR
    "${PROJECT_SOURCE_DIR}/../_thirdparty-build/vtk-qt5-mingw81/install-ready/lib/cmake/vtk-9.2")

if(NOT DEFINED VTK_DIR OR "${VTK_DIR}" STREQUAL "")
    if(EXISTS "${ROBOSDP_VTK_AUTO_DIR}/vtk-config.cmake")
        # 中文说明：Smoke 窗口优先验证带 IOGeometry 的完整 VTK Qt 包。
        set(VTK_DIR "${ROBOSDP_VTK_AUTO_DIR}" CACHE PATH "VTK CMake package directory" FORCE)
    elseif(EXISTS "${ROBOSDP_VTK_LEGACY_AUTO_DIR}/vtk-config.cmake")
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
    RenderingCore
    RenderingOpenGL2
    RenderingUI
)

if(QT_VERSION_MAJOR GREATER_EQUAL 6)
    qt_add_executable(RoboSDPVtkSmoke
        WIN32
        ${ROBOSDP_VTK_SMOKE_SOURCES}
    )
else()
    add_executable(RoboSDPVtkSmoke WIN32 ${ROBOSDP_VTK_SMOKE_SOURCES})
endif()

target_include_directories(RoboSDPVtkSmoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(RoboSDPVtkSmoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Widgets
)

if(VTK_FOUND)
    target_compile_definitions(RoboSDPVtkSmoke PRIVATE ROBOSDP_VTK_SMOKE_HAVE_VTK)
    target_link_libraries(RoboSDPVtkSmoke PRIVATE ${VTK_LIBRARIES})
    if(COMMAND vtk_module_autoinit)
        vtk_module_autoinit(
            TARGETS RoboSDPVtkSmoke
            MODULES ${VTK_LIBRARIES})
    endif()
    message(STATUS "RoboSDPVtkSmoke: 已检测到 VTK，QVTKOpenGLNativeWidget 最小验证路径已启用。")
else()
    message(WARNING "RoboSDPVtkSmoke: 当前未检测到 VTK，将仅构建 fallback 说明窗口。${VTK_NOT_FOUND_MESSAGE} 请通过 -DROBOSDP_VTK_DIR=... 或环境变量 ROBOSDP_VTK_DIR 指定 VTK CMake 目录。")
endif()

if(MINGW)
    message(STATUS "RoboSDPVtkSmoke: 当前 Qt 工具链为 MinGW，VTK 必须使用同一套 MinGW 工具链编译。")
endif()
