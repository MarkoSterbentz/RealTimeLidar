add_subdirectory(fileIO)
add_subdirectory(gui)

if( NOT WIN32 )
    set( CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_CXX_FLAGS} -ggdb3     -Wall -std=c++11 -D_REENTRANT -fopenmp" )
    set( CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_CXX_FLAGS} -Ofast     -Wall -std=c++11 -D_REENTRANT -fopenmp" )
    set( CMAKE_CXX_FLAGS_RELWITHDEBINFO  "${CMAKE_CXX_FLAGS} -Og -ggdb3 -Wall -std=c++11 -D_REENTRANT -fopenmp" )
endif()

add_executable(analyze
  main.cpp
  )
target_link_libraries(analyze
  pipeline
  structures
  fileIO
  gui
  BNO055
  ${SDL2_LIBRARY}
  ${GLEW_LIBRARIES}
  ${OPENGL_LIBRARIES}
  )
add_custom_command(TARGET analyze PRE_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${CMAKE_CURRENT_SOURCE_DIR}/assets $<TARGET_FILE_DIR:analyze>/assets)
