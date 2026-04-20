add_library(robosdp_topology STATIC
    dto/RobotTopologyModelDto.h
    dto/TopologyRecommendationDto.h
    persistence/TopologyJsonStorage.h
    persistence/TopologyJsonStorage.cpp
    service/TopologyService.h
    service/TopologyService.cpp
    service/TopologyTemplateLoader.h
    service/TopologyTemplateLoader.cpp
    ui/TopologyWidget.h
    ui/TopologyWidget.cpp
    validator/TopologyValidator.h
    validator/TopologyValidator.cpp
)

target_include_directories(robosdp_topology
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_compile_definitions(robosdp_topology
    PRIVATE
        ROBOSDP_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

target_link_libraries(robosdp_topology
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Widgets
        robosdp_errors
        robosdp_logging
        robosdp_repository
        robosdp_requirement
)
