add_executable(robosdp_kinematics_smoke
    KinematicsSmokeTest.cpp
)

target_include_directories(robosdp_kinematics_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_kinematics_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_kinematics
)

if(QT_VERSION_MAJOR EQUAL 5)
    get_filename_component(ROBOSDP_QT_RUNTIME_DIR "${Qt5_DIR}/../../../bin" ABSOLUTE)
else()
    get_filename_component(ROBOSDP_QT_RUNTIME_DIR "${Qt6_DIR}/../../../bin" ABSOLUTE)
endif()

set(ROBOSDP_KINEMATICS_TEST_PATH "${ROBOSDP_QT_RUNTIME_DIR};$ENV{PATH}")
if(DEFINED urdfdom_DIR AND NOT "${urdfdom_DIR}" STREQUAL "")
    get_filename_component(ROBOSDP_URDF_RUNTIME_DIR "${urdfdom_DIR}/../../../bin" ABSOLUTE)
    set(ROBOSDP_KINEMATICS_TEST_PATH "${ROBOSDP_QT_RUNTIME_DIR};${ROBOSDP_URDF_RUNTIME_DIR};$ENV{PATH}")
endif()
if(DEFINED pinocchio_DIR AND NOT "${pinocchio_DIR}" STREQUAL "")
    get_filename_component(ROBOSDP_PINOCCHIO_RUNTIME_DIR "${pinocchio_DIR}/../../../bin" ABSOLUTE)
    set(ROBOSDP_KINEMATICS_TEST_PATH "${ROBOSDP_QT_RUNTIME_DIR};${ROBOSDP_PINOCCHIO_RUNTIME_DIR};${ROBOSDP_KINEMATICS_TEST_PATH}")
endif()

add_test(
    NAME robosdp_kinematics_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_KINEMATICS_TEST_PATH}"
        $<TARGET_FILE:robosdp_kinematics_smoke>
)

add_executable(robosdp_kinematics_urdf_smoke
    KinematicsUrdfImportSmokeTest.cpp
)

target_include_directories(robosdp_kinematics_urdf_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_compile_definitions(robosdp_kinematics_urdf_smoke
    PRIVATE
        ROBOSDP_TEST_PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

target_link_libraries(robosdp_kinematics_urdf_smoke
    PRIVATE
        robosdp_kinematics
)

add_test(
    NAME robosdp_kinematics_urdf_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_KINEMATICS_TEST_PATH}"
        $<TARGET_FILE:robosdp_kinematics_urdf_smoke>
)

add_executable(robosdp_kinematics_urdf_geometry_smoke
    KinematicsUrdfGeometrySmokeTest.cpp
)

target_include_directories(robosdp_kinematics_urdf_geometry_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_kinematics_urdf_geometry_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_kinematics
        robosdp_repository
)

add_test(
    NAME robosdp_kinematics_urdf_geometry_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_KINEMATICS_TEST_PATH}"
        $<TARGET_FILE:robosdp_kinematics_urdf_geometry_smoke>
)

add_executable(robosdp_kinematics_refactor_prep_smoke
    KinematicsRefactorPreparationSmokeTest.cpp
)

target_include_directories(robosdp_kinematics_refactor_prep_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_kinematics_refactor_prep_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_kinematics
        robosdp_repository
)

add_test(
    NAME robosdp_kinematics_refactor_prep_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_KINEMATICS_TEST_PATH}"
        $<TARGET_FILE:robosdp_kinematics_refactor_prep_smoke>
)

add_executable(robosdp_kinematics_build_context_smoke
    KinematicsBuildContextSmokeTest.cpp
)

target_include_directories(robosdp_kinematics_build_context_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_kinematics_build_context_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_kinematics
        robosdp_repository
)

target_compile_definitions(robosdp_kinematics_build_context_smoke
    PRIVATE
        ROBOSDP_TEST_PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

add_test(
    NAME robosdp_kinematics_build_context_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_KINEMATICS_TEST_PATH}"
        $<TARGET_FILE:robosdp_kinematics_build_context_smoke>
)

add_executable(robosdp_kinematics_ik_switch_smoke
    KinematicsIkSwitchSmokeTest.cpp
)

target_include_directories(robosdp_kinematics_ik_switch_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_compile_definitions(robosdp_kinematics_ik_switch_smoke
    PRIVATE
        ROBOSDP_TEST_PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

target_link_libraries(robosdp_kinematics_ik_switch_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_kinematics
        robosdp_repository
)

add_test(
    NAME robosdp_kinematics_ik_switch_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_KINEMATICS_TEST_PATH}"
        $<TARGET_FILE:robosdp_kinematics_ik_switch_smoke>
)

add_executable(robosdp_kinematics_workspace_switch_smoke
    KinematicsWorkspaceSwitchSmokeTest.cpp
)

target_include_directories(robosdp_kinematics_workspace_switch_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_compile_definitions(robosdp_kinematics_workspace_switch_smoke
    PRIVATE
        ROBOSDP_TEST_PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

target_link_libraries(robosdp_kinematics_workspace_switch_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_kinematics
        robosdp_repository
)

add_test(
    NAME robosdp_kinematics_workspace_switch_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_KINEMATICS_TEST_PATH}"
        $<TARGET_FILE:robosdp_kinematics_workspace_switch_smoke>
)
