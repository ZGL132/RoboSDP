add_library(robosdp_dynamics STATIC
    ${PROJECT_SOURCE_DIR}/apps/desktop-qt/third_party/qcustomplot/qcustomplot.h
    ${PROJECT_SOURCE_DIR}/apps/desktop-qt/third_party/qcustomplot/qcustomplot.cpp
    dto/DynamicModelDto.h
    dto/DynamicsResultDto.h
    persistence/DynamicJsonStorage.h
    persistence/DynamicJsonStorage.cpp
    service/DynamicsService.h
    service/DynamicsService.cpp
    trajectory/BenchmarkTrajectoryFactory.h
    trajectory/BenchmarkTrajectoryFactory.cpp
    trajectory/TimeParameterizationSkeleton.h
    trajectory/TimeParameterizationSkeleton.cpp
    adapter/IDynamicsBackendAdapter.h
    adapter/IDynamicsBackendDiagnosticsAdapter.h
    adapter/PinocchioDynamicsBackendAdapter.h
    adapter/PinocchioDynamicsBackendAdapter.cpp
    ui/DynamicsWidget.h
    ui/DynamicsWidget.cpp
)

target_include_directories(robosdp_dynamics
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

# 优先从项目内依赖目录查找 Pinocchio，避免要求用户先全局安装。
set(ROBOSDP_PINOCCHIO_RUNTIME_DIRS "" CACHE INTERNAL "RoboSDP Pinocchio runtime directories")
set(ROBOSDP_PINOCCHIO_HINT_PATHS
    "${PROJECT_SOURCE_DIR}/.deps/pinocchio-env/Library"
    "${PROJECT_SOURCE_DIR}/.deps/pinocchio-env"
)
list(PREPEND CMAKE_PREFIX_PATH ${ROBOSDP_PINOCCHIO_HINT_PATHS})

find_package(pinocchio CONFIG QUIET
    PATHS ${ROBOSDP_PINOCCHIO_HINT_PATHS}
    NO_DEFAULT_PATH
)

if(NOT pinocchio_FOUND)
    find_package(pinocchio CONFIG QUIET)
endif()

if(pinocchio_FOUND AND TARGET pinocchio::pinocchio)
    target_compile_definitions(robosdp_dynamics PRIVATE ROBOSDP_HAVE_PINOCCHIO=1)
    # MinGW 下优先采用 Pinocchio 头文件 + Eigen 的最小原生接入，
    # 避免直接链接 conda-forge 提供的 MSVC 风格 import library。
    if(TARGET Eigen3::Eigen)
        target_link_libraries(robosdp_dynamics PUBLIC Eigen3::Eigen)
    endif()

    target_include_directories(robosdp_dynamics SYSTEM PRIVATE ${pinocchio_INCLUDE_DIRS})

    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_definitions(robosdp_dynamics PUBLIC
            WIN32=1
            __FUNCSIG__=__PRETTY_FUNCTION__
            COAL_DISABLE_HPP_FCL_WARNINGS=1
        )
        target_compile_options(robosdp_dynamics PUBLIC -Wa,-mbig-obj)
    endif()
endif()

target_link_libraries(robosdp_dynamics
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Widgets
        robosdp_errors
        robosdp_logging
        robosdp_infrastructure
        robosdp_repository
        robosdp_kinematics
)
