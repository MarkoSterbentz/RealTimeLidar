
if( NOT WIN32 )
    set( CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_CXX_FLAGS} -ggdb3     -Wall -std=c++11 -D_REENTRANT -fopenmp" )
    set( CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_CXX_FLAGS} -Ofast     -Wall -std=c++11 -D_REENTRANT -fopenmp" )
    set( CMAKE_CXX_FLAGS_RELWITHDEBINFO  "${CMAKE_CXX_FLAGS} -Og -ggdb3 -Wall -std=c++11 -D_REENTRANT -fopenmp" )
endif()

add_executable(capture
  main.cpp
  )
target_link_libraries(capture
  BNO055
  )