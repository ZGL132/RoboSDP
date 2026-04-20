add_library(robosdp_selection STATIC
    dto/MotorSelectionDto.h
    dto/ReducerSelectionDto.h
    dto/DriveTrainSelectionDto.h
    catalog/JsonComponentCatalog.h
    catalog/JsonComponentCatalog.cpp
    ui/SelectionWidget.h
    ui/SelectionWidget.cpp
    persistence/SelectionJsonStorage.h
    persistence/SelectionJsonStorage.cpp
    service/MotorSelectionService.h
    service/MotorSelectionService.cpp
    service/ReducerSelectionService.h
    service/ReducerSelectionService.cpp
    service/DriveTrainMatchingService.h
    service/DriveTrainMatchingService.cpp
)

target_include_directories(robosdp_selection
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_selection
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Widgets
        robosdp_errors
        robosdp_logging
        robosdp_repository
        robosdp_dynamics
)
