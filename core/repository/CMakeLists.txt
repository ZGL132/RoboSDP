add_library(robosdp_repository STATIC
    IJsonRepository.h
    LocalJsonRepository.h
    LocalJsonRepository.cpp
)

target_include_directories(robosdp_repository
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_repository
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_errors
)
