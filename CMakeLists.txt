cmake_minimum_required(VERSION 3.21)

project(RoboSDP VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND WIN32)
    # 中文说明：本地 Pinocchio/coal 的 CMake 包会注入 MSVC 专用 /bigobj，
    # MinGW 无法识别。这里通过极小的编译启动包装器过滤掉该参数，
    # 保持现有 Qt + MinGW 构建链可继续验证。
    set(CMAKE_CXX_COMPILER_LAUNCHER
        "${CMAKE_SOURCE_DIR}/.deps/pinocchio-env/python.exe"
        "${CMAKE_SOURCE_DIR}/tools/build/strip-msvc-bigobj.py")
endif()

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Widgets Network)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets Network)
include(CTest)

option(ROBOSDP_INCLUDE_TESTS_IN_ALL "Build smoke/unit/integration test targets as part of the default all target." OFF)
option(ROBOSDP_INCLUDE_VTK_SMOKE_IN_ALL "Build the standalone VTK smoke executable as part of the default all target." OFF)

add_subdirectory(core/errors)
add_subdirectory(core/logging)
add_subdirectory(core/config)
add_subdirectory(core/infrastructure)
add_subdirectory(core/repository)
add_subdirectory(modules/requirement)
add_subdirectory(modules/topology)
add_subdirectory(modules/kinematics)
add_subdirectory(modules/dynamics)
add_subdirectory(modules/selection)
add_subdirectory(modules/planning)
add_subdirectory(modules/scheme)
add_subdirectory(core/schema)
add_subdirectory(apps/desktop-qt)

if(ROBOSDP_INCLUDE_VTK_SMOKE_IN_ALL)
    add_subdirectory(apps/vtk-smoke)
else()
    # 中文说明：默认构建只面向主程序，VTK smoke 仍可通过显式 target 构建，避免 Qt Creator 每次运行前重复链接。
    add_subdirectory(apps/vtk-smoke EXCLUDE_FROM_ALL)
endif()

if(BUILD_TESTING)
    if(ROBOSDP_INCLUDE_TESTS_IN_ALL)
        add_subdirectory(tests)
    else()
        # 中文说明：保留测试 target 的配置能力，但从默认 all 中剥离，避免 smoke/integration 测试拖慢日常调试。
        add_subdirectory(tests EXCLUDE_FROM_ALL)
    endif()
endif()
