add_library(robosdp_requirement STATIC
    dto/RequirementModelDto.h
    persistence/RequirementJsonStorage.h
    persistence/RequirementJsonStorage.cpp
    service/KukaRequirementTemplateProvider.h
    service/KukaRequirementTemplateProvider.cpp
    service/RequirementTemplateCatalog.h
    service/RequirementTemplateCatalog.cpp
    service/RequirementService.h
    service/RequirementService.cpp
    ui/RequirementWidget.h
    ui/RequirementWidget.cpp
    validator/RequirementValidator.h
    validator/RequirementValidator.cpp
)

target_include_directories(robosdp_requirement
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_compile_definitions(robosdp_requirement
    PRIVATE
        ROBOSDP_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

target_link_libraries(robosdp_requirement
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Widgets
        robosdp_errors
        robosdp_logging
        robosdp_infrastructure
        robosdp_repository
)
