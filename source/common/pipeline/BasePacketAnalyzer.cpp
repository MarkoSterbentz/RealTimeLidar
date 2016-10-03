//
// Created by marko on 9/11/16.
//

#include "BasePacketAnalyzer.h"
namespace RealTimeLidar {
    void BasePacketAnalyzer::loadPacket(unsigned char *newPacket) {
        currentPacket = newPacket;
    }
}
