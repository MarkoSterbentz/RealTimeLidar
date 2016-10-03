cmake_minimum_required(VERSION 3.3)

project(RealTimeLidar)

set( CMAKE_DISABLE_SOURCE_CHANGES ON )
set( CMAKE_DISABLE_IN_SOURCE_BUILD ON )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" )

find_package( SDL2 REQUIRED )
include_directories( SYSTEM ${SDL2_INCLUDE_DIR} )

find_package( GLEW REQUIRED )
include_directories( SYSTEM ${GLEW_INCLUDE_DIRS} )

find_package( OpenGL REQUIRED )
include_directories( SYSTEM ${OPENGL_INCLUDE_DIR} )

option( BNO055_COMPILE_STANDALONE "Compile BNO055 as lib" OFF )
add_subdirectory( extern/bno055_uartSimple )

include_directories( SYSTEM extern )
include_directories( SYSTEM extern/bno055_uartSimple )
include_directories( SYSTEM extern/Eigen )
include_directories( SYSTEM extern/glm )
include_directories( SYSTEM extern/ReaderWriterQueue )

add_subdirectory("./source")
