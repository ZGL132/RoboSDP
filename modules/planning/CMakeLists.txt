add_library(robosdp_planning STATIC
    dto/PlanningSceneDto.h
    dto/PlanningRequestDto.h
    dto/PlanningResponseDto.h
    dto/PlanningVerificationResultDto.h
    adapter/MoveItGrpcAdapter.h
    adapter/MoveItGrpcAdapterGrpcClient.cpp
    persistence/PlanningJsonStorage.h
    persistence/PlanningJsonStorage.cpp
    service/PlanningVerificationService.h
    service/PlanningVerificationService.cpp
    ui/PlanningWidget.h
    ui/PlanningWidget.cpp
)

target_include_directories(robosdp_planning
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_planning
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Widgets
        robosdp_errors
        robosdp_logging
        robosdp_infrastructure
        robosdp_repository
        robosdp_kinematics
        robosdp_selection
)
