add_library(robosdp_schema STATIC
    ValidationMessage.h
    ValidationResult.h
    SchemaDtoConsistencyChecker.h
    SchemaDtoConsistencyChecker.cpp
    ModuleValidationRegistry.h
    ModuleValidationRegistry.cpp
    ProjectStructureChecker.h
    ProjectStructureChecker.cpp
    ProjectReferenceChecker.h
    ProjectReferenceChecker.cpp
)

target_include_directories(robosdp_schema
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_schema
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_errors
        robosdp_repository
        robosdp_requirement
        robosdp_topology
        robosdp_kinematics
        robosdp_dynamics
        robosdp_selection
        robosdp_planning
        robosdp_scheme
)
