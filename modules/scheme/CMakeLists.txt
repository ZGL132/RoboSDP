add_library(robosdp_scheme STATIC
    dto/SchemeAggregateDto.h
    dto/SchemeSnapshotDto.h
    dto/SchemeExportDto.h
    persistence/SchemeJsonStorage.h
    persistence/SchemeJsonStorage.cpp
    service/SchemeSnapshotService.h
    service/SchemeSnapshotService.cpp
    service/SchemeExportService.h
    service/SchemeExportService.cpp
    ui/SchemeWidget.h
    ui/SchemeWidget.cpp
)

target_include_directories(robosdp_scheme
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_scheme
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Widgets
        robosdp_errors
        robosdp_logging
        robosdp_repository
        robosdp_requirement
        robosdp_topology
        robosdp_kinematics
        robosdp_dynamics
        robosdp_selection
        robosdp_planning
)
