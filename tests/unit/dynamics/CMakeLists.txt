add_executable(robosdp_dynamics_smoke
    DynamicsSmokeTest.cpp
)

add_executable(robosdp_dynamics_backend_switch_smoke
    DynamicsBackendSwitchSmokeTest.cpp
)

add_executable(robosdp_dynamics_rnea_dry_run_smoke
    DynamicsNativeRneaDryRunSmokeTest.cpp
)

add_executable(robosdp_dynamics_shared_kernel_smoke
    SharedRobotKernelRegistrySmokeTest.cpp
)

add_executable(robosdp_dynamics_observability_smoke
    DynamicsObservabilitySmokeTest.cpp
)

target_include_directories(robosdp_dynamics_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_dynamics_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_repository
        robosdp_logging
        robosdp_kinematics
        robosdp_dynamics
)

target_include_directories(robosdp_dynamics_backend_switch_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_compile_definitions(robosdp_dynamics_backend_switch_smoke
    PRIVATE
        ROBOSDP_TEST_PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

target_link_libraries(robosdp_dynamics_backend_switch_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_repository
        robosdp_kinematics
        robosdp_dynamics
)

target_include_directories(robosdp_dynamics_rnea_dry_run_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_dynamics_rnea_dry_run_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_repository
        robosdp_kinematics
        robosdp_dynamics
)

target_include_directories(robosdp_dynamics_shared_kernel_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_include_directories(robosdp_dynamics_observability_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_dynamics_shared_kernel_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_kinematics
        robosdp_dynamics
)

target_link_libraries(robosdp_dynamics_observability_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Widgets
        robosdp_repository
        robosdp_logging
        robosdp_kinematics
        robosdp_dynamics
)

if(QT_VERSION_MAJOR EQUAL 5)
    get_filename_component(ROBOSDP_QT_RUNTIME_DIR "${Qt5_DIR}/../../../bin" ABSOLUTE)
else()
    get_filename_component(ROBOSDP_QT_RUNTIME_DIR "${Qt6_DIR}/../../../bin" ABSOLUTE)
endif()

set(ROBOSDP_DYNAMICS_TEST_RUNTIME_PATHS "${ROBOSDP_QT_RUNTIME_DIR}")
foreach(_runtime_dir IN LISTS ROBOSDP_PINOCCHIO_RUNTIME_DIRS)
    if(EXISTS "${_runtime_dir}")
        list(APPEND ROBOSDP_DYNAMICS_TEST_RUNTIME_PATHS "${_runtime_dir}")
    endif()
endforeach()
list(REMOVE_DUPLICATES ROBOSDP_DYNAMICS_TEST_RUNTIME_PATHS)
string(JOIN ";" ROBOSDP_DYNAMICS_TEST_PATH ${ROBOSDP_DYNAMICS_TEST_RUNTIME_PATHS} "$ENV{PATH}")

add_test(
    NAME robosdp_dynamics_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_DYNAMICS_TEST_PATH}"
        $<TARGET_FILE:robosdp_dynamics_smoke>
)

add_test(
    NAME robosdp_dynamics_backend_switch_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_DYNAMICS_TEST_PATH}"
        $<TARGET_FILE:robosdp_dynamics_backend_switch_smoke>
)

add_test(
    NAME robosdp_dynamics_rnea_dry_run_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_DYNAMICS_TEST_PATH}"
        $<TARGET_FILE:robosdp_dynamics_rnea_dry_run_smoke>
)

add_test(
    NAME robosdp_dynamics_shared_kernel_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_DYNAMICS_TEST_PATH}"
        $<TARGET_FILE:robosdp_dynamics_shared_kernel_smoke>
)

add_test(
    NAME robosdp_dynamics_observability_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_DYNAMICS_TEST_PATH}"
        $<TARGET_FILE:robosdp_dynamics_observability_smoke>
)
