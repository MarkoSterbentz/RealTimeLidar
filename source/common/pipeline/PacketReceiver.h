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

namespace RealTimeLidar {
    class PacketReceiver {
    private:
        std::ofstream outputFileStream;                 // network
        std::ifstream inputFile;                        // file
        std::string outputDataFileName;                 // network
        std::string inputDataFileName;                  // file
        int sockfd;                                     // network
        int numPacketsToRead;                           // universal
        unsigned char* dataBuf;                         // universal
        std::queue<unsigned char*> packetQueue;         // universal
        StreamingMedium streamMedium;                   // no longer needed

        int packetSize; //(in bytes)                    // universal
        int portNumber;                                 // network

        void readSingleDataPacketFromFile();            // file

    public:
        PacketReceiver();                               // universal
        PacketReceiver(StreamingMedium medium);
        ~PacketReceiver();                              // universal
        void openOutputFile();                          // network
        void openInputFile();                           // file
        int bindSocket();                               // network
        void listenForDataPacket();                     // network
        void writePacketToFile(unsigned char* packet);  // network
        bool endOfInputDataFile();                      // file
        void readDataPacketsFromFile(int numPackets);   // file
        unsigned long getPacketQueueSize();             // universal
        unsigned char* getNextQueuedPacket();           // universal
        void popQueuedPacket();                         // universal
        void increaseNumPacketsToRead(int num);         // universal

        /* Getters for private variables: */
        void setStreamMedium(StreamingMedium medium);   // universal
        StreamingMedium getStreamMedium();              // universal
        void setNumPacketsToRead(int num);              // universal
        int getNumPacketsToRead();                      // universal
        void setOutputDataFileName(std::string name);   // network
        std::string getOutputDataFileName();            // network
        void setInputDataFileName(std::string name);    // file
        std::string getInputDataFileName();             // file
    };
}
#endif //PACKETANALYZER_PACKETRECEIVER_H
