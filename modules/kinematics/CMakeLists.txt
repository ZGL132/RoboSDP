add_library(robosdp_kinematics STATIC
    ${PROJECT_SOURCE_DIR}/core/kinematics/SharedRobotKernelRegistry.h
    ${PROJECT_SOURCE_DIR}/core/kinematics/SharedRobotKernelRegistry.cpp
    dto/KinematicBackendBuildContextDto.h
    dto/KinematicModelDto.h
    dto/KinematicSolverResultDto.h
    dto/UrdfPreviewSceneDto.h
    adapter/IKinematicBackendAdapter.h
    adapter/IKinematicBackendDiagnosticsAdapter.h
    adapter/IIkSolverAdapter.h
    adapter/PinocchioKinematicBackendAdapter.h
    adapter/PinocchioKinematicBackendAdapter.cpp
    adapter/PinocchioIkSolverAdapter.h
    adapter/PinocchioIkSolverAdapter.cpp
    persistence/KinematicJsonStorage.h
    persistence/KinematicJsonStorage.cpp
    service/KinematicsService.h
    service/KinematicsService.cpp
    ui/KinematicsWidget.h
    ui/KinematicsWidget.cpp
)

target_include_directories(robosdp_kinematics
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

# 运动学模块同样通过 Adapter 接入 Pinocchio，但默认业务主链仍保持 Stage1Legacy。
# 这里仅为 PinocchioKinematicBackendAdapter 的原生 Model/Data 诊断构建提供依赖。
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
    if(TARGET Eigen3::Eigen)
        target_link_libraries(robosdp_kinematics PUBLIC Eigen3::Eigen)
    endif()

    target_include_directories(robosdp_kinematics SYSTEM PRIVATE ${pinocchio_INCLUDE_DIRS})

    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set_source_files_properties(
            ${PROJECT_SOURCE_DIR}/core/kinematics/SharedRobotKernelRegistry.cpp
            adapter/PinocchioKinematicBackendAdapter.cpp
            adapter/PinocchioIkSolverAdapter.cpp
            service/KinematicsService.cpp
            PROPERTIES
            COMPILE_DEFINITIONS "ROBOSDP_HAVE_PINOCCHIO=1;WIN32=1;__FUNCSIG__=__PRETTY_FUNCTION__;COAL_DISABLE_HPP_FCL_WARNINGS=1"
            COMPILE_OPTIONS "-Wa,-mbig-obj")
    else()
        set_source_files_properties(
            ${PROJECT_SOURCE_DIR}/core/kinematics/SharedRobotKernelRegistry.cpp
            adapter/PinocchioKinematicBackendAdapter.cpp
            adapter/PinocchioIkSolverAdapter.cpp
            service/KinematicsService.cpp
            PROPERTIES
            COMPILE_DEFINITIONS "ROBOSDP_HAVE_PINOCCHIO=1;COAL_DISABLE_HPP_FCL_WARNINGS=1")
    endif()
endif()

target_link_libraries(robosdp_kinematics
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Widgets
        robosdp_errors
        robosdp_logging
        robosdp_infrastructure
        robosdp_repository
        robosdp_topology
)
