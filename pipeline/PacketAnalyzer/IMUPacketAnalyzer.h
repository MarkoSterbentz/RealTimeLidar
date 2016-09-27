//
// Created by marko on 9/10/16.
//

#ifndef REALTIMELIDAR_IMUPACKETANALYZER_H
#define REALTIMELIDAR_IMUPACKETANALYZER_H

#include "BasePacketAnalyzer.h"
#include "imuData/IMUData.h"
namespace RealTimeLidar {
    class IMUPacketAnalyzer : public BasePacketAnalyzer {
    private:

    public:
        IMUData getIMUData();
    };
}
#endif //REALTIMELIDAR_IMUPACKETANALYZER_H
