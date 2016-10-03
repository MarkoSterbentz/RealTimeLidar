add_library( gui STATIC
    Camera.cpp
    Controls.cpp
    Graphics.cpp
    QuadTreeDrawer.cpp
    ScreenShot.cpp
    )

target_link_libraries( gui
    pipeline
    fileIO
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    )

target_include_directories( gui
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
