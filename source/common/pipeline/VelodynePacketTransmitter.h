//
// Created by marko on 9/27/16.
//

#ifndef REALTIMELIDAR_VELODYNEPACKETTRANSMITTER_H
#define REALTIMELIDAR_VELODYNEPACKETTRANSMITTER_H

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

#include "StreamingMedium.h"

namespace RealTimeLidar {
    class VelodynePacketTransmitter {
    private:
        int sockfd;
        struct addrinfo hints, *servinfo, *p;
        int rv;
        ssize_t numBytes;

        int createSocket();
    public:
        VelodynePacketTransmitter();
        ~VelodynePacketTransmitter();
        void transmitData(unsigned char *packetToSend);
    };
}

#endif //REALTIMELIDAR_VELODYNEPACKETTRANSMITTER_H
