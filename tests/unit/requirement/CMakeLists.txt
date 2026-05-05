add_executable(robosdp_requirement_validation_smoke
    RequirementValidationSmokeTest.cpp
)

target_include_directories(robosdp_requirement_validation_smoke
    PRIVATE
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_requirement_validation_smoke
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_requirement
)

if(QT_VERSION_MAJOR EQUAL 5)
    get_filename_component(ROBOSDP_QT_RUNTIME_DIR "${Qt5_DIR}/../../../bin" ABSOLUTE)
else()
    get_filename_component(ROBOSDP_QT_RUNTIME_DIR "${Qt6_DIR}/../../../bin" ABSOLUTE)
endif()

set(ROBOSDP_REQUIREMENT_TEST_PATH "${ROBOSDP_QT_RUNTIME_DIR};$ENV{PATH}")

add_test(
    NAME robosdp_requirement_validation_smoke
    COMMAND
        ${CMAKE_COMMAND} -E env
        "PATH=${ROBOSDP_REQUIREMENT_TEST_PATH}"
        $<TARGET_FILE:robosdp_requirement_validation_smoke>
)
