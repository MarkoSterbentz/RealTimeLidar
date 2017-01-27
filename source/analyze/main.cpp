//
// Created by Galen on 1/13/16.
//

#include <iostream>
#include <SDL_timer.h>
#include <SDL_thread.h>

#include "Graphics.h"
#include "Grid.h"
#include "GridDrawer.h"
#include "PacketReceiver.h"
#include "ImuReader.h"
#include "Controls.h"
#include "ArgumentHandler.h"
#include "DataPacketAnalyzer.h"
#include "IMUPacketAnalyzer.h"
#include "assemblyLine.hpp"

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

class ListeningModule : public assemblyLine::Module<ListeningModule, void*, CartesianPoint> {
    int packetsReadCount = 0;
public:
    ArgumentHandler argHandler;
    PacketReceiver receiver;
    DataPacketAnalyzer analyzer;
    PacketReceiver imuReceiver;
    IMUPacketAnalyzer imuAnalyzer;
    ListeningModule(int argc, char* argv[]) : argHandler(&receiver) {
        argHandler.handleCommandLineFlags(argc, argv, receiver);
    };
    void init() {
        receiver.openInputFile();
        imuReceiver.setStreamMedium(IMU);
        imuReceiver.openOutputFile();
        imuReceiver.bindSocket();
    }
    void operate() { // Listen for any incoming point data or imu data packets
        // IMU section (used to be separate from point data section, and on a different thread)
        imuReceiver.listenForDataPacket();
        if(imuReceiver.getPacketQueueSize() > 0) {
            //TODO: Set up imu packet writing to file, like in the data listening code above
            imuAnalyzer.loadPacket(imuReceiver.getNextQueuedPacket());
            imuReceiver.popQueuedPacket();
            ExtractedIMUData data = imuAnalyzer.extractIMUData();
            printf("IMU Data: orientation : {%f, %f, %f, %f}  |  linAccel : {%f, %f, %f}\n",
                   data.quat[0], data.quat[1], data.quat[2], data.quat[3], data.linAccel[0], data.linAccel[1], data.linAccel[2]);
            //TODO: Send the imu data wherever we want now
            //TODO: Enqueue the imu data in such a way that registration can use it along with the point data
            //TODO: Maybe every time registrar receives an imu item, it updates some state used in registering the points
        }
        // Point data section (used to be separate from imu section, and on a different thread)
        if (receiver.getStreamMedium() == VELODYNE_FORWARDER) { //VELODYNE) {
            receiver.listenForDataPacket();
        } else if (receiver.getStreamMedium() == INPUTFILE
                   && !receiver.endOfInputDataFile()
                   && packetsReadCount < receiver.getNumPacketsToRead()) {
            receiver.readDataPacketsFromFile(1);
            ++packetsReadCount;
        }
        // handle any incoming packet
        if (receiver.getPacketQueueSize() > 0) {
            if (argHandler.isOptionEnabled(WRITE)) {
                receiver.writePacketToFile(receiver.getNextQueuedPacket());
            }
            analyzer.loadPacket(receiver.getNextQueuedPacket());
            receiver.popQueuedPacket();    // packet has been read, get rid of it
            std::vector<CartesianPoint> newPoints(analyzer.getCartesianPoints());
            for (unsigned j = 0; j < newPoints.size(); ++j) {
                output->enqueue(newPoints[j]);
            }
        }
    }
    void deinit() { }
};

class IcpModule : public assemblyLine::Module<IcpModule, CartesianPoint, CartesianPoint> {
    Registrar<CartesianPoint>* registrar;
public:
    IcpModule() { }
    void init() {
        registrar = new Registrar<CartesianPoint>(&input, output, POINTS_PER_CLOUD, NUM_HISTS, CLOUD_SPARSITY);
    }
    void operate() {
        registrar->runICP();
    }
    void deinit() {
        delete registrar;
    }
};

template <typename P>
void initKernel(Grid<P>& grid) {
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

int main(int argc, char* argv[]) {

    // The grid and drawer
    // constructor min/max arguments are in millimeters (the LIDAR device is at the origin)
    // Arguments are: minX, maxX, minY, maxY, resX, resY, max number of points present
    Grid<CartesianPoint> grid(-5000.f, 5000.f, -5000.f, 5000.f, 10, 10, MAX_POINTS_IN_GRID);
    GridDrawer<CartesianPoint> gridDrawer;

    // The gui backend
    Graphics graphics;

    //TODO: PURE EVIL: Try switching the next two lines around
    IcpModule icpModule;
    ListeningModule listeningModule(argc, argv);
    assemblyLine::Chain<CartesianPoint, ListeningModule, IcpModule> pipeline(&listeningModule, &icpModule);

    // This sets up the kerning tools used for data analysis
    initKernel(grid);
    // Camera parameters: Vertical FOV, Near Plane, Far Plane, Aspect, Theta, Phi, Distance, DistMin, DistMax
    Camera camera(1.0, 10.0f, 100000.0, 1, 0.0, 1.2, 2000.0, 100.f, 10000.f);
    Controls controls;

    if (listeningModule.argHandler.isOptionEnabled(GRAPHICS)) {
        std::stringstream log;
        if (!graphics.init(log)) { // if init fails, exit
            std::cout << log.str();
            return 1;
        }
        camera.setAspect(graphics.getAspectRatio());
        gridDrawer.init(&grid, &camera, 0, log);
        std::cout << log.str();
        controls.init();
    }

    if (listeningModule.argHandler.isOptionEnabled(STREAM)) {
        pipeline.engage();
    }

    /* Main loop on this thread: */
    bool loop = true;
    while (loop) {
        /**************************** HANDLE INCOMING POINTS ********************************/
        CartesianPoint p;
        while (pipeline.results.try_dequeue(p)) {
            grid.addPoint(p);
        }
        if (listeningModule.argHandler.isOptionEnabled(GRAPHICS)) {
            /**************************** HANDLE CONTROLS ********************************/
            int timeToQuit = controls.update(listeningModule.receiver, camera, gridDrawer); // returns 1 if quit events happen
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

    if (listeningModule.argHandler.isOptionEnabled(STREAM)) {
        pipeline.disengage();
    }

    return 0;
}