add_library( utilities STATIC
    ArgumentHandler.cpp
    )

target_link_libraries( utilities
        pipeline
        )

target_include_directories( utilities
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})