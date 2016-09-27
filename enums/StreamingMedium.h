/**
 * Streaming Medium | Marko Sterbentz | 8/19/2016
 * Enum for tracking where the data is being streamed from.
 */

#ifndef REALTIMELIDAR_STREAMINGMEDIUM_H
#define REALTIMELIDAR_STREAMINGMEDIUM_H

/* VELODYNE DEFINES */
#define VELODYNE_PORT_NUMBER 2368
#define VELODYNE_PACKET_SIZE 1249
/* IMU DEFINES */
#define IMU_PORT_NUMBER 0       //TODO: Determine the IMU transmission portNumber
#define IMU_PACKET_SIZE 12      //TODO: Double check actual packetSize

enum StreamingMedium {
    VELODYNE, IMU, INPUTFILE
};

#endif //REALTIMELIDAR_STREAMINGMEDIUM_H
