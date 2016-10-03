//
// Created by marko on 9/27/16.
//

#include "VelodynePacketTransmitter.h"
namespace RealTimeLidar {
    /*****************************************************
     * CONSTRUCTOR AND DESTRUCTOR
     *****************************************************/
    VelodynePacketTransmitter::VelodynePacketTransmitter() {
        if (createSocket() != 0) {
            std::cout << "failed to create Velodyne packet transmitter socket." << std::endl;
        }
    }

    VelodynePacketTransmitter::~VelodynePacketTransmitter() {
        freeaddrinfo(servinfo);
        close(sockfd);
    }

    /******************************************************
     * UTILITY METHODS
     *****************************************************/
    int VelodynePacketTransmitter::createSocket() {
        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;

        if ((rv = getaddrinfo(DESTINATION_IP_ADDRESS, VELODYNE_SERVER_PORT, &hints, &servinfo)) != 0) {
            fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
            return 1;
        }

        // loop through all the results and make a socket
        for(p = servinfo; p != NULL; p = p->ai_next) {
            if ((sockfd = socket(p->ai_family, p->ai_socktype,
                                 p->ai_protocol)) == -1) {
                perror("talker: socket");
                continue;
            }
            break;
        }

        if (p == NULL) {
            fprintf(stderr, "talker: failed to create socket\n");
            return 2;
        }

        // Socket created successfully
        return 0;
    }

    void VelodynePacketTransmitter::transmitData(unsigned char *packetToSend) {
        // send the packet on transmissionPort
        if ((numBytes = sendto(sockfd, packetToSend, VELODYNE_PACKET_SIZE, 0,
                               p->ai_addr, p->ai_addrlen)) == -1) {
            perror("talker: sendto");
            exit(1);
        }
    }
}