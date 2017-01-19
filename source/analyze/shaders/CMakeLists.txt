add_library( shaders STATIC
    LoadShaders.cpp
    )

target_link_libraries( shaders
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    )

target_include_directories( shaders
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )
