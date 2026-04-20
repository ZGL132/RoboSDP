set(ROBOSDP_VTK_RENDER_SMOKE_SOURCES
    VtkRenderPipelineSmokeTest.cpp
)

set(ROBOSDP_VTK_DIR "" CACHE PATH "可选：手动指定与当前 Qt/MinGW 工具链匹配的 VTK CMake 配置目录")

if(ROBOSDP_VTK_DIR)
    # 中文说明：单测与桌面端共用同一个 VTK 定位入口，避免测试和 UI 链到不同工具链的 VTK。
    set(VTK_DIR "${ROBOSDP_VTK_DIR}" CACHE PATH "VTK CMake package directory" FORCE)
elseif(DEFINED ENV{ROBOSDP_VTK_DIR} AND NOT "$ENV{ROBOSDP_VTK_DIR}" STREQUAL "")
    set(VTK_DIR "$ENV{ROBOSDP_VTK_DIR}" CACHE PATH "VTK CMake package directory" FORCE)
elseif(DEFINED ENV{VTK_DIR} AND NOT "$ENV{VTK_DIR}" STREQUAL "")
    set(VTK_DIR "$ENV{VTK_DIR}" CACHE PATH "VTK CMake package directory" FORCE)
endif()

set(ROBOSDP_VTK_AUTO_DIR
    "${PROJECT_SOURCE_DIR}/../_thirdparty-build/vtk-qt5-mingw81/install-iogeometry/lib/cmake/vtk-9.2")
set(ROBOSDP_VTK_LEGACY_AUTO_DIR
    "${PROJECT_SOURCE_DIR}/../_thirdparty-build/vtk-qt5-mingw81/install-ready/lib/cmake/vtk-9.2")

if(NOT DEFINED VTK_DIR OR "${VTK_DIR}" STREQUAL "")
    if(EXISTS "${ROBOSDP_VTK_AUTO_DIR}/vtk-config.cmake")
        # 中文说明：单测优先使用带 vtkSTLReader 的完整 VTK 包，确保 STL 读取路径真验证。
        set(VTK_DIR "${ROBOSDP_VTK_AUTO_DIR}" CACHE PATH "VTK CMake package directory" FORCE)
    elseif(EXISTS "${ROBOSDP_VTK_LEGACY_AUTO_DIR}/vtk-config.cmake")
        set(VTK_DIR "${ROBOSDP_VTK_LEGACY_AUTO_DIR}" CACHE PATH "VTK CMake package directory" FORCE)
    endif()
endif()

find_package(VTK QUIET COMPONENTS
    CommonCore
    CommonDataModel
    CommonTransforms
    RenderingCore
    OPTIONAL_COMPONENTS
    IOGeometry
)

add_executable(robosdp_vtk_render_pipeline_smoke
    ${ROBOSDP_VTK_RENDER_SMOKE_SOURCES}
)

target_include_directories(robosdp_vtk_render_pipeline_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_vtk_render_pipeline_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
)

set(ROBOSDP_VTK_TEST_PATH "$ENV{PATH}")

if(VTK_FOUND)
    target_compile_definitions(robosdp_vtk_render_pipeline_smoke PRIVATE ROBOSDP_TEST_HAVE_VTK)
    if(TARGET VTK::IOGeometry)
        target_compile_definitions(robosdp_vtk_render_pipeline_smoke PRIVATE ROBOSDP_TEST_HAVE_VTK_IOGEOMETRY)
    else()
        message(WARNING "robosdp_vtk_render_pipeline_smoke: 当前 VTK 缺少 IOGeometry/vtkSTLReader，将验证 VTK 基础链接后跳过 STL 读取路径。")
    endif()
    target_link_libraries(robosdp_vtk_render_pipeline_smoke PRIVATE ${VTK_LIBRARIES})
    if(COMMAND vtk_module_autoinit)
        vtk_module_autoinit(
            TARGETS robosdp_vtk_render_pipeline_smoke
            MODULES ${VTK_LIBRARIES})
    endif()
    if(DEFINED VTK_DIR AND NOT "${VTK_DIR}" STREQUAL "")
        get_filename_component(ROBOSDP_VTK_RUNTIME_DIR "${VTK_DIR}/../../../bin" ABSOLUTE)
        set(ROBOSDP_VTK_TEST_PATH "${ROBOSDP_VTK_RUNTIME_DIR};${ROBOSDP_VTK_TEST_PATH}")
    endif()
else()
    message(WARNING "robosdp_vtk_render_pipeline_smoke: 未检测到 VTK，测试将执行优雅跳过路径。${VTK_NOT_FOUND_MESSAGE} 请通过 -DROBOSDP_VTK_DIR=... 或环境变量 ROBOSDP_VTK_DIR 指定 VTK CMake 目录。")
endif()

add_test(
    NAME robosdp_vtk_render_pipeline_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_VTK_TEST_PATH}"
        $<TARGET_FILE:robosdp_vtk_render_pipeline_smoke>
)
