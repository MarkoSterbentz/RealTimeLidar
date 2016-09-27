# set DIR to point to current directory (relative to top level)
set( DIR ${CMAKE_CURRENT_LIST_DIR} )

# include the current directory
set( RTL_INCLUDE_DIRS ${RTL_INCLUDE_DIRS} ${DIR} PARENT_SCOPE )

# include all headers/source in current directory
set( RTL_SOURCE_FILES ${RTL_SOURCE_FILES}
        ${DIR}/PacketAnalyzer.cpp ${DIR}/PacketAnalyzer.h
        ${DIR}/BasePacketAnalyzer.cpp ${DIR}/BasePacketAnalyzer.h
        ${DIR}/DataPacketAnalyzer.cpp ${DIR}/DataPacketAnalyzer.h
        ${DIR}/IMUPacketAnalyzer.cpp ${DIR}/IMUPacketAnalyzer.h
        PARENT_SCOPE)
