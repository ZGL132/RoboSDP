add_library(robosdp_config STATIC
    AppConfigLoader.cpp
    AppConfigLoader.h
    ConfigLoader.cpp
    ConfigLoader.h
)

target_include_directories(robosdp_config
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_config
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_errors
        robosdp_logging
)
