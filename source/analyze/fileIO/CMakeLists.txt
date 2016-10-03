add_library( fileIO STATIC
        LoadShaders.cpp
        )

target_link_libraries( fileIO
        ${GLEW_LIBRARIES}
        ${OPENGL_LIBRARIES}
        )

target_include_directories( fileIO
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
