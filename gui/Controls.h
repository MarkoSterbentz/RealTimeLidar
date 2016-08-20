//
// Created by volundr on 8/19/16.
//

#ifndef REALTIMELIDAR_CONTROLS_H
#define REALTIMELIDAR_CONTROLS_H

#include "PacketReceiver.h"
#include "Camera.h"
#include "GridDrawer.h"
#include "CartesianPoint.h"

namespace RealTimeLidar {
    class Controls {
        unsigned previousTime, currentTime, deltaTime;
    public:
        bool init();
        int handleControls(PacketReceiver& receiver, Camera& camera, GridDrawer<CartesianPoint>& gridDrawer);
    };
}

#endif //REALTIMELIDAR_CONTROLS_H
