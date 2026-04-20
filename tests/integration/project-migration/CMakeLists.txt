add_executable(robosdp_project_migration_integration
    E2E_ProjectMigrationIntegrationTest.cpp
)

target_include_directories(robosdp_project_migration_integration
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_compile_definitions(robosdp_project_migration_integration
    PRIVATE
        ROBOSDP_TEST_PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

target_link_libraries(robosdp_project_migration_integration
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_errors
        robosdp_logging
        robosdp_repository
        robosdp_requirement
        robosdp_topology
        robosdp_kinematics
        robosdp_dynamics
)

if(QT_VERSION_MAJOR EQUAL 5)
    get_filename_component(ROBOSDP_QT_RUNTIME_DIR "${Qt5_DIR}/../../../bin" ABSOLUTE)
else()
    get_filename_component(ROBOSDP_QT_RUNTIME_DIR "${Qt6_DIR}/../../../bin" ABSOLUTE)
endif()

set(ROBOSDP_PROJECT_MIGRATION_RUNTIME_PATHS "${ROBOSDP_QT_RUNTIME_DIR}")
foreach(_runtime_dir IN LISTS ROBOSDP_PINOCCHIO_RUNTIME_DIRS)
    if(EXISTS "${_runtime_dir}")
        list(APPEND ROBOSDP_PROJECT_MIGRATION_RUNTIME_PATHS "${_runtime_dir}")
    endif()
endforeach()
list(REMOVE_DUPLICATES ROBOSDP_PROJECT_MIGRATION_RUNTIME_PATHS)
string(JOIN ";" ROBOSDP_PROJECT_MIGRATION_PATH ${ROBOSDP_PROJECT_MIGRATION_RUNTIME_PATHS} "$ENV{PATH}")

add_test(
    NAME robosdp_project_migration_integration
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_PROJECT_MIGRATION_PATH}"
        $<TARGET_FILE:robosdp_project_migration_integration>
)

set_tests_properties(robosdp_project_migration_integration
    PROPERTIES
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)
