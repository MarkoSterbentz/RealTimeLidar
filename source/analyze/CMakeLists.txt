add_subdirectory( shaders )
add_subdirectory( gui )

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
add_executable( analyze
    main.cpp
    )
target_link_libraries( analyze
    pipeline
    structures
    lasFormat
    shaders
    gui
    BNO055
    ${SDL2_LIBRARY}
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    )
add_custom_command( TARGET analyze PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/assets $<TARGET_FILE_DIR:analyze>/assets )
