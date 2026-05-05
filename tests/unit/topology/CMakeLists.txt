add_executable(robosdp_topology_validation_smoke
    TopologyValidationSmokeTest.cpp
)

target_include_directories(robosdp_topology_validation_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_topology_validation_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_topology
)

if(QT_VERSION_MAJOR EQUAL 5)
    get_filename_component(ROBOSDP_QT_RUNTIME_DIR "${Qt5_DIR}/../../../bin" ABSOLUTE)
else()
    get_filename_component(ROBOSDP_QT_RUNTIME_DIR "${Qt6_DIR}/../../../bin" ABSOLUTE)
endif()

set(ROBOSDP_TOPOLOGY_TEST_PATH "${ROBOSDP_QT_RUNTIME_DIR};$ENV{PATH}")

add_test(
    NAME robosdp_topology_validation_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_TOPOLOGY_TEST_PATH}"
        $<TARGET_FILE:robosdp_topology_validation_smoke>
)
