//
// Created by Galen on 1/13/16.
//

#include <iostream>
#include <SDL_timer.h>
#include <SDL_thread.h>

#include "Graphics.h"
#include "Grid.h"
#include "GridDrawer.h"
#include "readerwriterqueue.h"
#include "PacketReceiver.h"
#include "ImuReader.h"
#include "Controls.h"
#include "ArgumentHandler.h"
#include "DataPacketAnalyzer.h"
#include "IMUPacketAnalyzer.h"

//#define SHOW_NONCONTRIBUTING_POINTS
#include "Registrar.h"

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
    PacketReceiver* receiver;
    DataPacketAnalyzer* analyzer;
    bool packetHandlerQuit;
    ArgumentHandler* argHandler;
};
struct RegistrationThreadData {
    Registrar<CartesianPoint>* registrar;
    bool registrationQuit;
};
struct ImuThreadData {
//    ImuReader* imu;
    PacketReceiver* imuReceiver;
    IMUPacketAnalyzer* imuAnalyzer;
    bool imuQuit;
};

// The grid and drawer
// constructor min/max arguments are in millimeters (the LIDAR device is at the origin)
// Arguments are: minX, maxX, minY, maxY, resX, resY, max number of points present
Grid<CartesianPoint> grid(-5000.f, 5000.f, -5000.f, 5000.f, 10, 10, MAX_POINTS_IN_GRID);
GridDrawer<CartesianPoint> gridDrawer;

// The gui backend
Graphics graphics;

// This is a thread safe queue designed for one producer and one consumer
moodycamel::ReaderWriterQueue<CartesianPoint> rawQueue(MAX_POINTS_IN_GRID);
//moodycamel::ReaderWriterQueue<CartesianPoint> registeredQueue(MAX_POINTS_IN_GRID);

// prototypes
void mainLoop(PacketReceiver& receiver, Camera& camera, Controls& controls, ArgumentHandler &argHandler);
void initPacketHandling(ListeningThreadData& ltd, SDL_Thread** packetListeningThread);
void stopPacketHandling(ListeningThreadData& ltd, SDL_Thread** packetListeningThread);
void initRegistration(RegistrationThreadData& rtd, SDL_Thread** registrationThread);
void stopRegistration(RegistrationThreadData& rtd, SDL_Thread** registrationThread);
void initImu(ImuThreadData& itd, SDL_Thread** imuThread);
void stopImu(ImuThreadData& itd, SDL_Thread** imuThread);
void initKernel();
int initGraphics(Camera& camera);
int listeningThreadFunction(void* listeningThreadData);
int registrationThreadFunction(void* arg);
int imuThreadFunction(void* arg);

int main(int argc, char* argv[]) {

    PacketReceiver receiver;
    DataPacketAnalyzer analyzer;
//    Registrar<CartesianPoint> registrar(&rawQueue, &registeredQueue, POINTS_PER_CLOUD, NUM_HISTS, CLOUD_SPARSITY);
    PacketReceiver imuReceiver;
    IMUPacketAnalyzer imuAnalyzer;
    ImuReader imuReader;
    ArgumentHandler argHandler(&receiver);

    imuReceiver.setStreamMedium(IMU);

    ListeningThreadData ltd = { &receiver, &analyzer, false, &argHandler };
    SDL_Thread* packetListeningThread = NULL;

//    RegistrationThreadData rtd = { &registrar, false };
//    SDL_Thread* registrationThread = NULL;

    ImuThreadData itd = { &imuReceiver, &imuAnalyzer, false };
    SDL_Thread* imuThread = NULL;

    argHandler.handleCommandLineFlags(argc, argv, receiver);
    receiver.openInputFile();

    // This sets up the kerning tools used for data analysis
    initKernel();
    // Camera parameters: Vertical FOV, Near Plane, Far Plane, Aspect, Theta, Phi, Distance, DistMin, DistMax
    Camera camera(1.0, 10.0f, 100000.0, 1, 0.0, 1.2, 2000.0, 100.f, 10000.f);
    Controls controls;

    if (argHandler.isOptionEnabled(GRAPHICS)) {
        if (initGraphics(camera) == 1) {
            return 1;
        }
        controls.init();
    }

    if (argHandler.isOptionEnabled(STREAM)) {
        initPacketHandling(ltd, &packetListeningThread);
//        initRegistration(rtd, &registrationThread);
        initImu(itd, &imuThread);
    }

    /* Begin the main loop on this thread: */
    mainLoop(receiver, camera, controls, argHandler);

    if (argHandler.isOptionEnabled(STREAM)) {
        stopPacketHandling(ltd, &packetListeningThread);
//        stopRegistration(rtd, &registrationThread);
        stopImu(itd, &imuThread);
    }

//    if (!argHandler.isOptionEnabled(GRAPHICS)) {
//        stopImu(itd, &imuThread);
//    }

    return 0;
}

void mainLoop(PacketReceiver& receiver, Camera& camera, Controls& controls, ArgumentHandler &argHandler) {
    bool loop = true;

    while (loop) {
        /**************************** HANDLE INCOMING POINTS ********************************/
        CartesianPoint p;
//        while (registeredQueue.try_dequeue(p)) {
        while (rawQueue.try_dequeue(p)) {
            grid.addPoint(p);
        }

        if (argHandler.isOptionEnabled(GRAPHICS)) {
            /**************************** HANDLE CONTROLS ********************************/
            int timeToQuit = controls.update(receiver, camera, gridDrawer); // returns 1 if quit events happen
            if (timeToQuit) {
                loop = false;
            }
            /**************************** DO THE DRAWING *********************************/
            // draw the grid
            gridDrawer.drawGrid();
            // update the screen
            graphics.render();
        }
    }
}

// This function runs inside the listeningThread.
int listeningThreadFunction(void* arg) {
    std::cout << "Packet handling thread is active.\n";

    ListeningThreadData* ltd = (ListeningThreadData*) arg;
    int packetsReadCount = 0;

    while (!ltd->packetHandlerQuit) {
        /*************************** HANDLE PACKETS *********************************/
        if (ltd->receiver->getStreamMedium() == VELODYNE_FORWARDER) { //VELODYNE) {
            ltd->receiver->listenForDataPacket();
        } else if (ltd->receiver->getStreamMedium() == INPUTFILE
                   && !ltd->receiver->endOfInputDataFile()
                   && packetsReadCount < ltd->receiver->getNumPacketsToRead()) {
            ltd->receiver->readDataPacketsFromFile(1);
            ++packetsReadCount;
        }
        // handle any incoming packet
        if (ltd->receiver->getPacketQueueSize() > 0) {
            if (ltd->argHandler->isOptionEnabled(WRITE)) {
                ltd->receiver->writePacketToFile(ltd->receiver->getNextQueuedPacket());
            }
            ltd->analyzer->loadPacket(ltd->receiver->getNextQueuedPacket());
            ltd->receiver->popQueuedPacket();    // packet has been read, get rid of it
            std::vector<CartesianPoint> newPoints(ltd->analyzer->getCartesianPoints());
            for (unsigned j = 0; j < newPoints.size(); ++j) {
                rawQueue.enqueue(newPoints[j]);
            }
        }
    }
    return 0;
}

int registrationThreadFunction(void* arg) {
    std::cout << "Point registration thread is active.\n";
    RegistrationThreadData* rtd = (RegistrationThreadData*) arg;
    while (!rtd->registrationQuit) {
        rtd->registrar->runICP();
    }
    return 0;
}

int imuThreadFunction(void* arg) {
    std::cout << "IMU reading thread is active.\n";
    ImuThreadData* idt = (ImuThreadData*) arg;
//    while (!idt->imuQuit) {
//        idt->imu->printOrientation();
//    }
    while (!idt->imuQuit) {
        idt->imuReceiver->listenForDataPacket();
        if(idt->imuReceiver->getPacketQueueSize() > 0) {
            //TODO: Set up imu packet writing to file, like in the data listening thread
            idt->imuAnalyzer->loadPacket(idt->imuReceiver->getNextQueuedPacket());
            idt->imuReceiver->popQueuedPacket();
            ExtractedIMUData data = idt->imuAnalyzer->extractIMUData();
//            printf("IMU Data: orientation : {%f, %f, %f,}  |  linAccel : {%f, %f, %f}\n",
//                   data.orient[0], data.orient[1], data.orient[2], data.linAccel[0], data.linAccel[1], data.linAccel[2]);
            printf("IMU Data: orientation : {%f, %f, %f, %f}  |  linAccel : {%f, %f, %f}\n",
                   data.quat[0], data.quat[1], data.quat[2], data.quat[4], data.linAccel[0], data.linAccel[1], data.linAccel[2]);
            //TODO: Send the imu data wherever we want now
        }
    }
    return 0;
}

void initPacketHandling(ListeningThreadData& ltd, SDL_Thread** packetListeningThread) {
    // packet handler setup
    ltd.receiver->openOutputFile();
    ltd.receiver->bindSocket();

    // spawn the listening thread, passing it information in "data"
    *packetListeningThread = SDL_CreateThread (listeningThreadFunction, "listening thread", (void *) &ltd);
}

void stopPacketHandling(ListeningThreadData& ltd, SDL_Thread** packetListeningThread) {
    // once the main loop has exited, set the listening thread's "quit" to true and wait for the thread to die.
    ltd.packetHandlerQuit = true;
    SDL_WaitThread(*packetListeningThread, NULL);
}

void initRegistration(RegistrationThreadData& rtd, SDL_Thread** registrationThread) {
    *registrationThread = SDL_CreateThread(registrationThreadFunction, "point registration thread", (void *) &rtd);
}

void stopRegistration(RegistrationThreadData& rtd, SDL_Thread** registrationThread) {
    rtd.registrationQuit = true;
    SDL_WaitThread(*registrationThread, NULL);
}

void initImu(ImuThreadData& itd, SDL_Thread** imuThread) {
//    itd.imu->init();
    itd.imuReceiver->openOutputFile();
    itd.imuReceiver->bindSocket();

    *imuThread = SDL_CreateThread(imuThreadFunction, "imu thread", (void*) &itd);
}

void stopImu(ImuThreadData& itd, SDL_Thread** imuThread) {
    itd.imuQuit = true;
    SDL_WaitThread(*imuThread, NULL);
}

void initKernel() {
    /* KERNEL IMPLEMENTATIONS: */
    auto avgKernel = [] (Grid<CartesianPoint>* pGrid, int x, int y){ // x and y determine which grid cell is being operated on
        int totalPointsCounted = 0;
        float average = 0;
        int gridIndex = y * pGrid->getXRes() + x; // index into the vector of GridCells
        for (unsigned i = 0; i < pGrid->kernel.contributingCells.size(); ++i) {
            if (x + pGrid->kernel.contributingCells[i].xOffset >= 0 && x + pGrid->kernel.contributingCells[i].xOffset < pGrid->getXRes()) {
                int gridIndexOffset = pGrid->kernel.contributingCells[i].yOffset * pGrid->getXRes(); // offset from the gridIndex to the current contributing cell
                gridIndexOffset += pGrid->kernel.contributingCells[i].xOffset;
                int ccIndex = gridIndex + gridIndexOffset; // contributingCellIndex
                if (ccIndex >= 0 && ccIndex < (int) pGrid->cells.size()) {
                    for (unsigned j = 0; j < pGrid->cells[ccIndex].points.size(); ++j) {
                        average += pGrid->cells[ccIndex].points[j].y;
                    }
                    totalPointsCounted += pGrid->cells[ccIndex].points.size();
                }
            }
        }
        if (totalPointsCounted > 0) {
            average /= totalPointsCounted;
        }
        pGrid->cells[gridIndex].kernelOutput = average;
    };

    avgKernel(&grid, 0, 0);

    int kernelError = grid.specifyKernel(
            {
                    0, 1, 0,
                    1, 1, 1,
                    0, 1, 0,
            }, avgKernel);
    if (kernelError) {
        std::cout << "KERNEL FAILURE!\n";
    }
}

int initGraphics(Camera& camera) {
    std::stringstream log;
    if (!graphics.init(log)) { // if init fails, exit
        std::cout << log.str();
        return 1;
    }
    camera.setAspect(graphics.getAspectRatio());
    gridDrawer.init(&grid, &camera, 0, log);
    std::cout << log.str();
    return 0;
}