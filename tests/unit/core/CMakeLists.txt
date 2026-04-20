add_executable(robosdp_core_infra_smoke
    CoreInfrastructureSmokeTest.cpp
)

target_include_directories(robosdp_core_infra_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_core_infra_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_errors
        robosdp_logging
        robosdp_config
)

if(QT_VERSION_MAJOR EQUAL 5)
    get_filename_component(ROBOSDP_QT_RUNTIME_DIR "${Qt5_DIR}/../../../bin" ABSOLUTE)
else()
    get_filename_component(ROBOSDP_QT_RUNTIME_DIR "${Qt6_DIR}/../../../bin" ABSOLUTE)
endif()

string(JOIN ";" ROBOSDP_CORE_TEST_PATH "${ROBOSDP_QT_RUNTIME_DIR}" "$ENV{PATH}")

add_test(
    NAME robosdp_core_infra_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_CORE_TEST_PATH}"
        $<TARGET_FILE:robosdp_core_infra_smoke>
)
