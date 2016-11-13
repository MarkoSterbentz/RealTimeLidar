#ifndef REALTIMELIDAR_IMUPACKETTRANSMITTER_H
#define REALTIMELIDAR_IMUPACKETTRANSMITTER_H
/* IMU Packet Transmitter | Marko Sterbentz 9/11/2016
 * A UDP transmitter for packaging and forwarding data collected from the IMU.
 *  This should be ran on a separate thread.
 */
#include <queue>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <iomanip>

#include "Bno055Interface.h"
#include "StreamingMedium.h"

namespace RealTimeLidar {
    class IMUPacketTransmitter {
    private:
        std::string transmissionPort; // the port number the transmitter will broadcast the packets on
        bno055::Bno055Interface bno055;

        bool initBNO055(); // initialize the interface from which the IMU data can be queried
        void packData(bno055::ImuData_f data, std::vector<unsigned char> &packetOut); // creates a data packet from the data on the front of the queue
        void packData(std::vector<int16_t> &data, std::vector<unsigned char> &packetOut);

        // NETWORKING VARIABLES AND METHODS
        int sockfd;
//        struct addrinfo hints, *servinfo, *p;
        int rv;
        ssize_t numBytes;
        int createSocket();


    public:
        struct addrinfo hints, *servinfo, *p;
        IMUPacketTransmitter();
        ~IMUPacketTransmitter();
        void transmitData(); // broadcasts a newly created data packet
        int init();
        void printOrientation();

        std::string getTransmissionPort();
        void setTransmissionPort(int transmissionPort);
    };
}
#endif //REALTIMELIDAR_IMUPACKETTRANSMITTER_H
