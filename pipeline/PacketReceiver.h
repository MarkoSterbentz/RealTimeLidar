#ifndef PACKETANALYZER_PACKETRECEIVER_H
#define PACKETANALYZER_PACKETRECEIVER_H

/**
 * Packet Receiver | Marko Sterbentz | 2/5/2016
 * This class will receieve/read data from outside the application.
 * Supported sources include: VeloDyne VLP-16, binary data file
 *
 * Galen Cochrane - Last modified 2016 June 15
 */

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
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <queue>
#include <string>
#include <stdexcept>

#include "StreamingMedium.h"

#define DATAPORT "2368"    // the Data Packet is broadcasted to this port
#define POSITIONPORT "8308" // the Position Packet is broadcasted to this port

#define DATABUFLEN 1249  //size of the data packet
#define POSBUFLEN 554   // size of the position packet

namespace RealTimeLidar {

    class PacketReceiver {
    private:
        std::ofstream outputFileStream;
        std::ifstream inputFile;
        std::string outputDataFileName;
        std::string inputDataFileName;
        int sockfd;
        int numPacketsToRead;
        unsigned char dataBuf[DATABUFLEN];
        unsigned char posBuf[POSBUFLEN];
        std::queue<unsigned char*> packetQueue;
        StreamingMedium streamMedium;

        void readSingleDataPacketFromFile();

    public:
        PacketReceiver();
        ~PacketReceiver();
        void openOutputFile();
        void openInputFile();
        int bindSocket();
        void listenForDataPacket();
        void writePacketToFile(unsigned char* packet);
        bool endOfInputDataFile();
        void readDataPacketsFromFile(int numPackets);
        unsigned long getPacketQueueSize();
        unsigned char* getNextQueuedPacket();
        void popQueuedPacket();
        void increaseNumPacketsToRead(int num);

        /* Getters for private variables: */
        void setStreamMedium(StreamingMedium medium);
        StreamingMedium getStreamMedium();
        void setNumPacketsToRead(int num);
        int getNumPacketsToRead();
        void setOutputDataFileName(std::string name);
        std::string getOutputDataFileName();
        void setInputDataFileName(std::string name);
        std::string getInputDataFileName();
    };
}
#endif //PACKETANALYZER_PACKETRECEIVER_H
