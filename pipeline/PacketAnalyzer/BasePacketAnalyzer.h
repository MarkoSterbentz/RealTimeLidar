//
// Created by marko on 9/11/16.
//

#ifndef REALTIMELIDAR_NEWPACKETANALYZER_H
#define REALTIMELIDAR_NEWPACKETANALYZER_H

namespace RealTimeLidar{
    class BasePacketAnalyzer {
    private:

    protected:
        unsigned char* currentPacket;

    public:
        void loadPacket(unsigned char* newPacket);
    };
}
#endif //REALTIMELIDAR_NEWPACKETANALYZER_H
