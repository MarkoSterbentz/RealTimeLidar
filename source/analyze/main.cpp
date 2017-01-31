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

struct PointOrImuPacket {
    union {
        ExtractedIMUData imu;
        CartesianPoint point;
    };
    bool isImuPacket = false;
    float& x() { return point.x; }
    float& y() { return point.y; }
    float& z() { return point.z; }
};

class ListeningModule : public assemblyLine::Module<ListeningModule, void*, PointOrImuPacket> {
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
        receiver.bindSocket();
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
            PointOrImuPacket outputData;
            outputData.imu = data;
            outputData.isImuPacket = true;
            output->enqueue(outputData);
        }
        // Point data section (used to be separate from imu section, and on a different thread)
        if (receiver.getStreamMedium() == VELODYNE_FORWARDER || receiver.getStreamMedium() == VELODYNE) {
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
                PointOrImuPacket outputData;
                outputData.point = newPoints[j];
                outputData.isImuPacket = false;
                output->enqueue(outputData);
            }
        }
    }
    void deinit() { }
};

class IcpModule : public assemblyLine::Module<IcpModule, PointOrImuPacket, CartesianPoint> {
    Registrar<PointOrImuPacket, CartesianPoint>* registrar;
    Eigen::Affine3f ct; // current transform
public:
    IcpModule() { }
    void init() {
        registrar = new Registrar<PointOrImuPacket, CartesianPoint> (
                &input, output, POINTS_PER_CLOUD, NUM_HISTS, CLOUD_SPARSITY );
    }
    void operate() {
        registrar->operate(ct);
    }
    void deinit() {
        delete registrar;
    }
    //TODO: don't return value, pass in reference to avoid copies
    glm::mat4 getCurrentTransform() {
        //TODO: the translation bit
        glm::mat4 result;
        result = {
                {ct(0,0), ct(0,1), ct(0,2), 0.f},
                {ct(1,0), ct(1,1), ct(1,2), 0.f},
                {ct(2,0), ct(2,1), ct(2,2), 0.f},
                {ct(3,0), ct(3,1), ct(3,2), ct(3,3)},
        };
        return result;
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

//TODO: probably make this into a normal class
namespace ArrowRenderer {
    namespace {
        bool isInit = false;
        GLuint shader;
        GLuint vao;
        GLuint vertices;
        GLuint normals;
        GLint shader_mvMat;
        GLint shader_mvpMat;
        GLint shader_color;
        float currentColor[3];
        glm::mat4 modelMat;
    }
    bool init(std::stringstream& log) {
        if (isInit) {
            return false;
        }
        isInit = true;

        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        // set up a simple shader that shades any primitive a single solid color.
        shader = loadShaders("assets/flatColor.vert", "assets/flatColor.frag", log);
        GLint shader_vertex = glGetAttribLocation(shader, "vertex");
        GLint shader_normal = glGetAttribLocation(shader, "normal");
        shader_mvMat = glGetUniformLocation(shader, "mvMat");
        shader_mvpMat = glGetUniformLocation(shader, "mvpMat");
        shader_color = glGetUniformLocation(shader, "color");

        // this is the data that will be buffered up as vertices
        std::vector<float> verts = {
              0.f,  150.f,   0.f,
            -80.f,  -90.f,   0.f,
              0.f,  -90.f,  25.f,

              0.f,  150.f,   0.f,
              0.f,  -90.f,  25.f,
             80.f,  -90.f,   0.f,

              0.f,  150.f,   0.f,
              0.f,  -90.f, -25.f,
            -80.f,  -90.f,   0.f,

              0.f,  150.f,   0.f,
             80.f,  -90.f,   0.f,
              0.f,  -90.f, -25.f,

              0.f,  -90.f,  25.f,
            -80.f,  -90.f,   0.f,
              0.f,  -90.f, -25.f,

              0.f,  -90.f, -25.f,
             80.f,  -90.f,   0.f,
              0.f,  -90.f,  25.f,
        };

        // this is the data that will be buffered up as normals
        std::vector<float> norms = {
            -0.296812f, 0.0989372f,  0.949797f,
            -0.296812f, 0.0989372f,  0.949797f,
            -0.296812f, 0.0989372f,  0.949797f,

             0.296812f, 0.0989372f,  0.949797f,
             0.296812f, 0.0989372f,  0.949797f,
             0.296812f, 0.0989372f,  0.949797f,

            -0.296812f, 0.0989372f, -0.949797f,
            -0.296812f, 0.0989372f, -0.949797f,
            -0.296812f, 0.0989372f, -0.949797f,

             0.296812f, 0.0989372f, -0.949797f,
             0.296812f, 0.0989372f, -0.949797f,
             0.296812f, 0.0989372f, -0.949797f,

             0.f, -1.f, 0.f,
             0.f, -1.f, 0.f,
             0.f, -1.f, 0.f,

             0.f, -1.f, 0.f,
             0.f, -1.f, 0.f,
             0.f, -1.f, 0.f,
        };

        unsigned long vertBufferSizeInBytes = verts.size() * sizeof(float);
        unsigned long normBufferSizeInBytes = norms.size() * sizeof(float);

        // buffer verts
        glGenBuffers(1, &vertices);
        glBindBuffer(GL_ARRAY_BUFFER, vertices);
        glBufferData(GL_ARRAY_BUFFER, vertBufferSizeInBytes, NULL, GL_STATIC_DRAW);
        glEnableVertexAttribArray((GLuint) shader_vertex);
        glVertexAttribPointer((GLuint) shader_vertex, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glBufferSubData(GL_ARRAY_BUFFER, 0, verts.size() * sizeof(float), verts.data());

        // buffer norms
        glGenBuffers(1, &normals);
        glBindBuffer(GL_ARRAY_BUFFER, normals);
        glBufferData(GL_ARRAY_BUFFER, normBufferSizeInBytes, NULL, GL_STATIC_DRAW);
        glEnableVertexAttribArray((GLuint) shader_normal);
        glVertexAttribPointer((GLuint) shader_normal, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glBufferSubData(GL_ARRAY_BUFFER, 0, norms.size() * sizeof(float), norms.data());

        currentColor[0] = 0.3f;
        currentColor[1] = 0.6f;
        currentColor[2] = 0.3f;

        return true;
    }
    void draw(Camera& camera) {
        // Bind the VAO
        glBindVertexArray(vao);
        // Tell GPU to use the colorShader program for following draw calls
        glUseProgram(shader);
        // Upload the mvp matrix to the colorShader program on the GPU
        glUniformMatrix4fv(shader_mvpMat, 1, GL_FALSE, &((glm::mat4)(camera.getVp() * modelMat))[0][0]);
        // Upload the mv matrix to the colorShader program on the GPU
        glUniformMatrix4fv(shader_mvMat, 1, GL_FALSE, &((glm::mat4)(camera.getView() * modelMat))[0][0]);
        // Upload the color you want to the colorShader program on the GPU
        glUniform3f(shader_color, currentColor[0], currentColor[1], currentColor[2]);
        // Draw
        glDrawArrays(GL_TRIANGLES, 0, (GLsizei) 18);
    }
    void setModelMat(const glm::mat4&& mat) {
        modelMat = mat;
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
        log.clear();
        if (!ArrowRenderer::init(log)) {
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
            // draw IMU position
            ArrowRenderer::setModelMat(icpModule.getCurrentTransform());
            ArrowRenderer::draw(camera);
            // update the screen
            graphics.render();
        }
    }

    if (listeningModule.argHandler.isOptionEnabled(STREAM)) {
        pipeline.disengage();
    }

    return 0;
}