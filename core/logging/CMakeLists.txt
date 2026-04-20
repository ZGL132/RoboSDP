add_library(robosdp_logging STATIC
    ConsoleLogger.cpp
    ConsoleLogger.h
    ILogger.h
    LogMessage.h
    Logger.cpp
    Logger.h
)

target_include_directories(robosdp_logging
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_logging
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
        robosdp_errors
)
