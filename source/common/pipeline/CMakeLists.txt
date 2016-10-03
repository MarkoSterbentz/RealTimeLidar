add_library( pipeline STATIC
        ImuReader.cpp
        PacketAnalyzer.cpp
        PacketReceiver.cpp
        )
target_link_libraries( pipeline
        structures
        )
target_include_directories( pipeline
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
