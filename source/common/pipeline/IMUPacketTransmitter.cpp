//
// Created by marko on 9/12/16.
//

#include <bno055DataStructures.h>
#include <bno055_uartSimple/bno055DataStructures.h>
#include "IMUPacketTransmitter.h"

#define BNO055_DEVICE_FILE "/dev/ttyS0"

namespace RealTimeLidar {
    /*****************************************************
     * CONSTRUCTOR AND DESTRUCTOR
     *****************************************************/
    IMUPacketTransmitter::IMUPacketTransmitter() {

    }

    IMUPacketTransmitter::~IMUPacketTransmitter() {
        freeaddrinfo(servinfo);
        close(sockfd);
    }

    /******************************************************
     * UTILITY METHODS
     *****************************************************/
    int IMUPacketTransmitter::init() {
        initBNO055();
        this->transmissionPort = IMU_FORWARD_PORT;
        if (createSocket() != 0) {
            std::cout << "failed to create IMU packet transmitter socket." << std::endl;
        }
        return 0;
    }

    bool IMUPacketTransmitter::initBNO055() {
        bno055.init(BNO055_DEVICE_FILE);
        if (bno055.isLive()) {
            printf("BNO055 is live.\n");
        } else {
            printf("Could not contact BNO055!\n");
            return false;
        }
        return true;
    }

    /* Creates a packet of the format: heading, roll, pitch, linAccel1, linAccel2, linAccel3. */
    void IMUPacketTransmitter::packData(bno055::ImuData_f data, std::vector<unsigned char> &packetOut) {
        //*packetOut = new unsigned char[PACKET_SIZE];
        packetOut.resize(IMU_PACKET_SIZE);

        // extract the needed data
        //bno055::Vec3_f orientData = data.names.orient;
        bno055::Vec4_f quatData = data.names.quat;
        bno055::Vec3_f linAccelData = data.names.linAccel;

        // write the bytes
//        for(int i = 0; i < 3; ++i) {
//            ((float*) packetOut.data())[i] = orientData[i];
//            ((float*) packetOut.data())[i+3] = linAccelData[i];
//        }
        // write the bytes
        for(int i = 0; i < 4; ++i) {
            ((float*) packetOut.data())[i] = quatData[i];
        }
        for(int i = 4; i < 7; ++i) {
            ((float*) packetOut.data())[i] = linAccelData[i];
        }
    }

    /* Creates a socket over which the IMU data can be transmitted. */
    int IMUPacketTransmitter::createSocket() {
        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;

        if ((rv = getaddrinfo(FORWARD_IP_ADDRESS, IMU_FORWARD_PORT, &hints, &servinfo)) != 0) {
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

    void IMUPacketTransmitter::transmitData() {
        //printOrientation();


        // TESTING NEW
        bno055::ImuData_16 data;
        int16_t a[7];

        if (bno055.queryChunk_16(a, bno055::QUA_Data_w_LSB, 14)) {
            std::vector<unsigned char> newPacket;
            packData(data.toFloats(), newPacket);

            // send the packet on transmissionPort
            if ((numBytes = sendto(sockfd, newPacket.data(), IMU_PACKET_SIZE, 0,
                                   p->ai_addr, p->ai_addrlen)) == -1) {
                perror("talker: sendto");
                exit(1);
            }
        }
        // END TESTING NEW

//        // query the data
//        bno055::ImuData_16 data;
//        if (bno055.queryImuData(&data)) {
//
//
//            // create the packet
//            //unsigned char *newPacket = 0;
//            std::vector<unsigned char> newPacket;
//            packData(data.toFloats(), newPacket);
//
//            // TESTING
////            printf("New Packet: ");
////            for (int i = 0; i < IMU_PACKET_SIZE; ++i)
////                printf("%u", newPacket[i]);
////
////            printf("\n");
//
////            std::cout << p->ai_addr->sa_data << std::endl;
//            //END TESTING
//
//            // send the packet on transmissionPort
//            if ((numBytes = sendto(sockfd, newPacket.data(), IMU_PACKET_SIZE, 0,
//                                   p->ai_addr, p->ai_addrlen)) == -1) {
//                perror("talker: sendto");
//                exit(1);
//            }
//        }
    }

    void IMUPacketTransmitter::printOrientation() {
//        if ( ! bno055.isLive()) {
//            return;
//        }
//        bno055::Vec3_16 orient;
//        bno055::Vec3_f orientDeg;
//        if (bno055.queryVec3(&orient, bno055::EULER_ORIENT)) {
//            orientDeg = orient.toFusionEulerOrientation();
//            std::cout << "-> ";
//            std::cout << std::setiosflags(std::ios::right);
//            std::cout << std::setiosflags(std::ios::fixed);
//            std::cout << std::setw(10) << std::setprecision(4) << orientDeg.heading
//                      << std::setw(10) << std::setprecision(4) << orientDeg.roll
//                      << std::setw(10) << std::setprecision(4) << orientDeg.pitch;
//            std::cout << std::endl;
//        }
        if ( ! bno055.isLive()) {
            return;
        }
        bno055::ImuData_16 data;
        int16_t a[7];

        if (bno055.queryChunk_16(a, bno055::QUA_Data_w_LSB, 14)) {
            std::cout << "-> ";
            std::cout << std::setiosflags(std::ios::right);
            std::cout << std::setiosflags(std::ios::fixed);
            std::cout << std::setw(10) << std::setprecision(4) << a[1]
                      << std::setw(10) << std::setprecision(4) << a[2]
                      << std::setw(10) << std::setprecision(4) << a[3];
            std::cout << std::endl;
        }
    }

    /******************************************************
     * GETTERS AND SETTERS FOR IMUPacketTransmitter
     *****************************************************/
    std::string IMUPacketTransmitter::getTransmissionPort() {
        return transmissionPort;
    }

    void IMUPacketTransmitter::setTransmissionPort(int transmissionPort) {
        this->transmissionPort = transmissionPort;
    }
}
