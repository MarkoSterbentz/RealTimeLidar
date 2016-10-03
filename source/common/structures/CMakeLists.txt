add_library( structures STATIC
    CartesianPoint.cpp
    )

target_link_libraries( structures
        utilities
        )

target_include_directories( structures
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})