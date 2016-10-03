//
// Created by marko on 9/11/16.
//

#ifndef REALTIMELIDAR_DATAPACKETANALYZER_H
#define REALTIMELIDAR_DATAPACKETANALYZER_H

#include <cmath>
#include <iostream>
#include <vector>

#include "CartesianPoint.h"
#include "BasePacketAnalyzer.h"
namespace RealTimeLidar {

    #define RAD_CONVERSION 0.01745329251

    struct ChannelInfo {
        float distance;
        float reflectivity;
    };

    struct DataBlockInfo {
        float azimuth1; // for the first firing sequence
        float azimuth2; // for the second firing sequence
        ChannelInfo channels[32];
    };

    struct DataPacketInfo {
        DataBlockInfo blocks[12];
        float timestamp;
        unsigned returnMode;
    };

    class DataPacketAnalyzer : public BasePacketAnalyzer {
    private:
        CartesianPoint getSingleXYZ(float distance, float elevationAngle, float azimuth);
        DataPacketInfo extractDataPacketInfo();
        DataBlockInfo extractDataBlockInfo(unsigned int dbIndex);
        ChannelInfo extractChannelInfo(unsigned int chIndex);
        void interpolateSecondAzimuth(DataPacketInfo &packetInfo);
        float calculateFirstAzimuth(unsigned int azIndex);
        float calculateTimestamp(unsigned int tsIndex);
        float calculateDistance(unsigned int distIndex);
        unsigned char calculateReflectivity(unsigned int refIndex);
        unsigned char calculateReturnMode(unsigned int retIndex);
    public:
        DataPacketAnalyzer();
        ~DataPacketAnalyzer();
        std::vector<CartesianPoint> getCartesianPoints();
    };
}
#endif //REALTIMELIDAR_DATAPACKETANALYZER_H
