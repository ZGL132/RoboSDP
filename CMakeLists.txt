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

add_subdirectory(core/errors)
add_subdirectory(core/logging)
add_subdirectory(core/config)
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
add_subdirectory(apps/vtk-smoke)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
