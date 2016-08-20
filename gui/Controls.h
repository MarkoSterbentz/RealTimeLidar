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
        int pollEvents(PacketReceiver& receiver, Camera& camera, GridDrawer<CartesianPoint>& gridDrawer);
        void handleKeyState(PacketReceiver& receiver, Camera& camera);
    public:
        bool init();
        int update(PacketReceiver& receiver, Camera& camera, GridDrawer<CartesianPoint>& gridDrawer);
    };
}

#endif //REALTIMELIDAR_CONTROLS_H
