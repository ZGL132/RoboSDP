add_library(robosdp_infrastructure STATIC
    ProjectDirtyDependencyGraph.h
    ProjectDirtyDependencyGraph.cpp
    ProjectManager.h
    ProjectManager.cpp
    ProjectSaveCoordinator.h
    ProjectSaveCoordinator.cpp
)

target_include_directories(robosdp_infrastructure
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_infrastructure
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
)
