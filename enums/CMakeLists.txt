# set DIR to point to current directory (relative to top level)
set( DIR ${CMAKE_CURRENT_LIST_DIR} )

# include the current directory
set( RTL_INCLUDE_DIRS ${RTL_INCLUDE_DIRS} ${DIR} PARENT_SCOPE )