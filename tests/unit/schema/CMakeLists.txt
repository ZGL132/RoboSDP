add_executable(robosdp_schema_smoke
    SchemaConsistencySmokeTest.cpp
)

add_executable(robosdp_project_validation_smoke
    ProjectValidationSmokeTest.cpp
)

add_executable(robosdp_json_roundtrip_smoke
    JsonRoundTripSmokeTest.cpp
)

target_include_directories(robosdp_schema_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_schema_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_repository
        robosdp_requirement
        robosdp_topology
        robosdp_kinematics
        robosdp_dynamics
        robosdp_selection
        robosdp_planning
        robosdp_scheme
        robosdp_schema
)

target_include_directories(robosdp_project_validation_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_project_validation_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_repository
        robosdp_requirement
        robosdp_topology
        robosdp_kinematics
        robosdp_dynamics
        robosdp_selection
        robosdp_planning
        robosdp_scheme
        robosdp_schema
)

target_include_directories(robosdp_json_roundtrip_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_json_roundtrip_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_repository
        robosdp_requirement
        robosdp_topology
        robosdp_kinematics
        robosdp_dynamics
        robosdp_selection
        robosdp_planning
        robosdp_scheme
)

if(QT_VERSION_MAJOR EQUAL 5)
    get_filename_component(ROBOSDP_QT_RUNTIME_DIR "${Qt5_DIR}/../../../bin" ABSOLUTE)
else()
    get_filename_component(ROBOSDP_QT_RUNTIME_DIR "${Qt6_DIR}/../../../bin" ABSOLUTE)
endif()

set(ROBOSDP_SCHEMA_TEST_RUNTIME_PATHS "${ROBOSDP_QT_RUNTIME_DIR}")
foreach(_runtime_dir IN LISTS ROBOSDP_PINOCCHIO_RUNTIME_DIRS)
    if(EXISTS "${_runtime_dir}")
        list(APPEND ROBOSDP_SCHEMA_TEST_RUNTIME_PATHS "${_runtime_dir}")
    endif()
endforeach()
list(REMOVE_DUPLICATES ROBOSDP_SCHEMA_TEST_RUNTIME_PATHS)
string(JOIN ";" ROBOSDP_SCHEMA_TEST_PATH ${ROBOSDP_SCHEMA_TEST_RUNTIME_PATHS} "$ENV{PATH}")

add_test(
    NAME robosdp_schema_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_SCHEMA_TEST_PATH}"
        $<TARGET_FILE:robosdp_schema_smoke>
)

add_test(
    NAME robosdp_project_validation_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_SCHEMA_TEST_PATH}"
        $<TARGET_FILE:robosdp_project_validation_smoke>
)

add_test(
    NAME robosdp_json_roundtrip_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_SCHEMA_TEST_PATH}"
        $<TARGET_FILE:robosdp_json_roundtrip_smoke>
)
