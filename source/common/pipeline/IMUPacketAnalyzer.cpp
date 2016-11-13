//
// Created by marko on 9/10/16.
//

#include <iostream>
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
//        ExtractedIMUData data = {.orient = {-1,-1,-1}, .linAccel = {-1,-1,-1}};
//        for(int i = 0; i < 3; ++i) {
//            data.orient[i] = ((float*) currentPacket)[i];
//            data.linAccel[i]= ((float*) currentPacket)[i+3];
//        }
//        return data;
        ExtractedIMUData data = {.quat = {-1.F,-1.F,-1.F,-1.F}, .linAccel = {-1.F,-1.F,-1.F}};
        //TODO: Change these loops to memcpy
        for(int i = 0; i < 4; ++i) {
            data.quat[i] = (float)(((int16_t*) currentPacket)[i]);
        }
        for(int i = 0; i < 3; ++i) {
            data.linAccel[i]= (float)(((int16_t*) currentPacket)[(i+4)]);
        }
        return data;
    }
};