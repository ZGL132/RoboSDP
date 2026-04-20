add_executable(robosdp_scheme_smoke
    SchemeSmokeTest.cpp
)

target_include_directories(robosdp_scheme_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_scheme_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_errors
        robosdp_logging
        robosdp_repository
        robosdp_requirement
        robosdp_topology
        robosdp_kinematics
        robosdp_dynamics
        robosdp_selection
        robosdp_planning
        robosdp_scheme
)

get_target_property(ROBOSDP_QT_CORE_LOCATION Qt${QT_VERSION_MAJOR}::Core IMPORTED_LOCATION_RELEASE)
if(NOT ROBOSDP_QT_CORE_LOCATION)
    get_target_property(ROBOSDP_QT_CORE_LOCATION Qt${QT_VERSION_MAJOR}::Core IMPORTED_LOCATION)
endif()

get_filename_component(ROBOSDP_QT_BIN_DIR "${ROBOSDP_QT_CORE_LOCATION}" DIRECTORY)
get_filename_component(ROBOSDP_CXX_BIN_DIR "${CMAKE_CXX_COMPILER}" DIRECTORY)

add_test(
    NAME robosdp_scheme_smoke
    COMMAND ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_QT_BIN_DIR};${ROBOSDP_CXX_BIN_DIR};$ENV{PATH}"
        $<TARGET_FILE:robosdp_scheme_smoke>
)

set_tests_properties(robosdp_scheme_smoke
    PROPERTIES
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)
