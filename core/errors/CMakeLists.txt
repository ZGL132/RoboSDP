add_library(robosdp_errors STATIC
    ErrorCode.cpp
    ErrorCode.h
    ErrorInfo.h
)

target_include_directories(robosdp_errors
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(robosdp_errors
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
)
