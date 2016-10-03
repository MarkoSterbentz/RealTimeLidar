//
// Created by marko on 9/10/16.
//

#ifndef REALTIMELIDAR_IMUPACKETANALYZER_H
#define REALTIMELIDAR_IMUPACKETANALYZER_H

#include "BasePacketAnalyzer.h"
#include "IMUData.h"
namespace RealTimeLidar {
    class IMUPacketAnalyzer : public BasePacketAnalyzer {
    private:

    public:
        IMUPacketAnalyzer();
        ~IMUPacketAnalyzer();
        ExtractedIMUData extractIMUData();
    };
}
#endif //REALTIMELIDAR_IMUPACKETANALYZER_H
