//
// Created by Galen on 1/13/16.
//

#include <iostream>
#include <SDL_timer.h>
#include <SDL_thread.h>

#include "readerwriterqueue.h"
#include "PacketReceiver.h"
#include "ImuReader.h"
#include "ArgumentHandler.h"
#include "IMUPacketTransmitter.h"
#include "VelodynePacketTransmitter.h"

//#define SHOW_NONCONTRIBUTING_POINTS

#define POINTS_PER_PACKET 384                   // known, do not set
#define PACKETS_PER_REVOLUTION_1200_RPM 19      // approximate, from observation
#define POINTS_PER_REVOLUTION POINTS_PER_PACKET * PACKETS_PER_REVOLUTION_1200_RPM
#define REVOLUTIONS_PER_CLOUD 1                 // n revolutions make up a cloud used in ICP
#define CLOUD_SPARSITY 12                       // every n incoming point(s) will be considered for ICP (added to cloud)
#define NUM_HISTS 150                           // number of point cloud histories to use for ICP
#define POINTS_PER_CLOUD (POINTS_PER_REVOLUTION / CLOUD_SPARSITY) * REVOLUTIONS_PER_CLOUD

#ifdef SHOW_NONCONTRIBUTING_POINTS
#define MAX_POINTS_IN_GRID POINTS_PER_REVOLUTION * NUM_HISTS * REVOLUTIONS_PER_CLOUD
#else
#define MAX_POINTS_IN_GRID POINTS_PER_CLOUD * NUM_HISTS
#endif

using namespace RealTimeLidar;

struct ListeningThreadData {
    PacketReceiver* dataReceiver;
    VelodynePacketTransmitter* dataTransmitter;
    bool packetHandlerQuit;
    ArgumentHandler* argHandler;
};
struct ImuThreadData {
    IMUPacketTransmitter* imuTransmitter;
    ImuReader* imu;
    bool imuQuit;
};

// prototypes
void mainLoop(PacketReceiver& receiver, ArgumentHandler &argHandler);
void initPacketHandling(ListeningThreadData& ltd, SDL_Thread** packetListeningThread);
void stopPacketHandling(ListeningThreadData& ltd, SDL_Thread** packetListeningThread);
void initImu(ImuThreadData& itd, SDL_Thread** imuThread);
void stopImu(ImuThreadData& itd, SDL_Thread** imuThread);
int listeningThreadFunction(void* listeningThreadData);
int imuThreadFunction(void* arg);

int main(int argc, char* argv[]) {

    PacketReceiver dataReceiver;
    VelodynePacketTransmitter dataTransmitter;
    IMUPacketTransmitter imuTransmitter;
    ImuReader imuReader;
    ArgumentHandler argHandler(&dataReceiver);

    ListeningThreadData ltd = { &dataReceiver, &dataTransmitter, false, &argHandler };
    SDL_Thread* packetListeningThread = NULL;

    ImuThreadData itd = { &imuTransmitter, &imuReader, false };
    SDL_Thread* imuThread = NULL;

    argHandler.handleCommandLineFlags(argc, argv, dataReceiver);

    if (argHandler.isOptionEnabled(STREAM)) {
        initPacketHandling(ltd, &packetListeningThread);
        initImu(itd, &imuThread);
    }

    /* Begin the main loop on this thread: */
    mainLoop(dataReceiver, argHandler);

    if (argHandler.isOptionEnabled(STREAM)) {
        stopPacketHandling(ltd, &packetListeningThread);
        stopImu(itd, &imuThread);
    }


    return 0;
}
//TODO: Figure out what the main loop needs to be doing
//TODO: Figure out if mainLoop() needs these arguments anymore
void mainLoop(PacketReceiver& receiver, ArgumentHandler &argHandler) {
    bool loop = true;

    while (loop) {
        //TODO: Get this to stop on its own, without needing user input
        if (argHandler.getStringInput("Enter 'q' to quit.\n").compare("q") == 0) {
            loop = false;
        }
    }
}

// This function runs inside the listeningThread.
int listeningThreadFunction(void* arg) {
    std::cout << "Packet handling thread is active.\n";

    ListeningThreadData* ltd = (ListeningThreadData*) arg;
    while (!ltd->packetHandlerQuit) {
        /*************************** HANDLE PACKETS *********************************/
        if (ltd->dataReceiver->getStreamMedium() == VELODYNE) {
            ltd->dataReceiver->listenForDataPacket();
        }
        // handle any incoming packet
        if (ltd->dataReceiver->getPacketQueueSize() > 0) {
            if (ltd->argHandler->isOptionEnabled(WRITE)) {
                ltd->dataReceiver->writePacketToFile(ltd->dataReceiver->getNextQueuedPacket());
            }
            ltd->dataTransmitter->transmitData(ltd->dataReceiver->getNextQueuedPacket());
            ltd->dataReceiver->popQueuedPacket();    // packet has been transmitted, get rid of it
        }
    }
    return 0;
}

int imuThreadFunction(void* arg) {
    std::cout << "IMU reading thread is active.\n";
    ImuThreadData* idt = (ImuThreadData*) arg;

    // TESTING
    for (int i = 0; i < 14; ++i)
        printf("%u", (unsigned char) idt->imuTransmitter->p->ai_addr->sa_data[i]);

    printf("\n%d\n", idt->imuTransmitter->p->ai_addrlen);

    while (!idt->imuQuit) {
        idt->imuTransmitter->transmitData();
        usleep(1000000);
    }
    return 0;
}

void initPacketHandling(ListeningThreadData& ltd, SDL_Thread** packetListeningThread) {
    // packet handler setup
    ltd.dataReceiver->openOutputFile();
    ltd.dataReceiver->bindSocket();

    // spawn the listening thread, passing it information in "data"
    *packetListeningThread = SDL_CreateThread (listeningThreadFunction, "listening thread", (void *) &ltd);
}

void stopPacketHandling(ListeningThreadData& ltd, SDL_Thread** packetListeningThread) {
    // once the main loop has exited, set the listening thread's "quit" to true and wait for the thread to die.
    ltd.packetHandlerQuit = true;
    SDL_WaitThread(*packetListeningThread, NULL);
}

void initImu(ImuThreadData& itd, SDL_Thread** imuThread) {
    itd.imu->init();
    *imuThread = SDL_CreateThread(imuThreadFunction, "imu thread", (void*) &itd);
}

void stopImu(ImuThreadData& itd, SDL_Thread** imuThread) {
    itd.imuQuit = true;
    SDL_WaitThread(*imuThread, NULL);
}
