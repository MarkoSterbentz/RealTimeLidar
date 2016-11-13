/**
 * Streaming Medium | Marko Sterbentz | 8/19/2016
 * Enum for tracking where the data is being streamed from.
 * Also includes defines for transmitting data.
 */

#ifndef REALTIMELIDAR_STREAMINGMEDIUM_H
#define REALTIMELIDAR_STREAMINGMEDIUM_H

/* Streaming Defines: */
#define VELODYNE_TRANSMISSION_PORT "2368"       // port number the Velodyne hardware transmits on

#define VELODYNE_PACKET_SIZE 1249               // size of the packet the Velodyne hardware transmits
#define IMU_PACKET_SIZE 14 //28  //24                // size of the packet we create to transmit. We are transmitting 7 int16_t

#define FORWARD_IP_ADDRESS "10.42.0.1"          // the IP address of the laptop / hardware running the analyze program
#define VELODYNE_FORWARD_PORT "22023"           // port number VelodynePacketTransmitter transmits on
#define IMU_FORWARD_PORT "22022"                // port number IMUPacketTransmitter transmits on


enum StreamingMedium {
    VELODYNE, IMU, INPUTFILE, VELODYNE_FORWARDER
};

#endif //REALTIMELIDAR_STREAMINGMEDIUM_H
