//
// Created by marko on 9/11/16.
//

#ifndef REALTIMELIDAR_IMUDATA_H
#define REALTIMELIDAR_IMUDATA_H

namespace RealTimeLidar {
    struct ExtractedIMUData {
        //float orient[3];                // TODO: Change this to quaternions, and there are four values
        float quat[4];
        float linAccel[3];
    };
}
#endif //REALTIMELIDAR_IMUDATA_H
