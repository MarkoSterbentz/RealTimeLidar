# No lesser version of cMake may be used.
cmake_minimum_required(VERSION 3.3)

# Declare the project in question
project(RealTimeLidar)

# Tell cMake where to look for the modules that search for SDL2 (the others come with cMake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmakeModules/")

# Set gcc compiler flags
if( NOT WIN32 )
    # these are the compiler flags to be used on gcc
    set( CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_CXX_FLAGS} -ggdb3     -Wall -std=c++11 -D_REENTRANT -fopenmp" )
    set( CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_CXX_FLAGS} -Ofast     -Wall -std=c++11 -D_REENTRANT -fopenmp" )
    set( CMAKE_CXX_FLAGS_RELWITHDEBINFO  "${CMAKE_CXX_FLAGS} -Og -ggdb3 -Wall -std=c++11 -D_REENTRANT -fopenmp" )
endif()

# Find SDL 2.0
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIR})
set(RTL_LIBS ${RTL_LIBS} ${SDL2_LIBRARY})

# Find GLEW
find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})
set(RTL_LIBS ${RTL_LIBS} ${GLEW_LIBRARIES})

# Find OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
set(RTL_LIBS ${RTL_LIBS} ${OPENGL_LIBRARIES})

# Use IMU submodule
option(BNO055_COMPILE_STANDALONE "Compile BNO055 as lib" OFF)
add_subdirectory(bno055_uartSimple)
include_directories(bno055_uartSimple)
set(RTL_LIBS ${RTL_LIBS} BNO055)

add_subdirectory(fileIO)
add_subdirectory(graphics)
add_subdirectory(grid)
add_subdirectory(pipeline)
add_subdirectory(quadTree)
add_subdirectory(utilities)

include_directories(${RTL_INCLUDE_DIRS})
set(RTL_SOURCE_FILES ${RTL_SOURCE_FILES} main.cpp )

add_executable(${PROJECT_NAME} ${RTL_SOURCE_FILES})

# Link to external libraries that were found above
target_link_libraries(${PROJECT_NAME} ${RTL_LIBS})

# Copy assets into build tree
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/assets
        $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets)

# Copy data directory into build tree
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/data
        $<TARGET_FILE_DIR:${PROJECT_NAME}>/data)
