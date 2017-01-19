add_library( pipeline STATIC
    BasePacketAnalyzer.cpp
    DataPacketAnalyzer.cpp
    IMUPacketAnalyzer.cpp
    IMUPacketTransmitter.cpp
    ImuReader.cpp
    PacketAnalyzer.cpp
    PacketReceiver.cpp
    VelodynePacketTransmitter.cpp
    )
target_link_libraries( pipeline
    structures
    )
target_include_directories( pipeline
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )
