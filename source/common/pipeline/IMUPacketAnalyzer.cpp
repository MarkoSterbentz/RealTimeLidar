//
// Created by marko on 9/10/16.
//

#include "IMUPacketAnalyzer.h"

namespace RealTimeLidar {
    /*****************************************************
     * CONSTRUCTOR AND DESTRUCTOR
     *****************************************************/
    IMUPacketAnalyzer::IMUPacketAnalyzer() {

    }

    IMUPacketAnalyzer::~IMUPacketAnalyzer() {

    }

    /******************************************************
     * UTILITY METHODS
     *****************************************************/
    ExtractedIMUData IMUPacketAnalyzer::extractIMUData() {
        ExtractedIMUData data = {.orient = {-1,-1,-1}, .linAccel = {-1,-1,-1}};
        for(int i = 0; i < 3; ++i) {
            data.orient[i] = ((float*) currentPacket)[i];
            data.linAccel[i]= ((float*) currentPacket)[i+3];
        }
        return data;
    }
};