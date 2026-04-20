add_executable(robosdp_selection_smoke
    SelectionSmokeTest.cpp
)

target_include_directories(robosdp_selection_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_selection_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_errors
        robosdp_logging
        robosdp_repository
        robosdp_kinematics
        robosdp_dynamics
        robosdp_selection
)

get_target_property(ROBOSDP_QT_CORE_LOCATION Qt${QT_VERSION_MAJOR}::Core IMPORTED_LOCATION_RELEASE)
if(NOT ROBOSDP_QT_CORE_LOCATION)
    get_target_property(ROBOSDP_QT_CORE_LOCATION Qt${QT_VERSION_MAJOR}::Core IMPORTED_LOCATION)
endif()

get_filename_component(ROBOSDP_QT_BIN_DIR "${ROBOSDP_QT_CORE_LOCATION}" DIRECTORY)
get_filename_component(ROBOSDP_CXX_BIN_DIR "${CMAKE_CXX_COMPILER}" DIRECTORY)

add_test(
    NAME robosdp_selection_smoke
    COMMAND ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_QT_BIN_DIR};${ROBOSDP_CXX_BIN_DIR};$ENV{PATH}"
        $<TARGET_FILE:robosdp_selection_smoke>
)

set_tests_properties(robosdp_selection_smoke
    PROPERTIES
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)
