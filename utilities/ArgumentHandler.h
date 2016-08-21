/**
 * Argument Handler | Marko Sterbentz | 8/19/2016
 * Handles arguments and other initial input provided by the user.
 */
#ifndef REALTIMELIDAR_ARGUMENTHANDLER_H
#define REALTIMELIDAR_ARGUMENTHANDLER_H

#include <string>
#include <iostream>
#include <fstream>
#include "PacketReceiver.h"
#include "StreamingMedium.h"

namespace RealTimeLidar {

    enum OptionsState {
        OFF = -1, UNKNOWN = 0, ON = 1
    };
    enum Options {
        GRAPHICS = 0, STREAM = 1, WRITE = 2, FORWARD = 3
    };

    class ArgumentHandler {
    private:
        int cmdOptions[4];
        PacketReceiver* receiver;

        void checkOptionInput(char &input, int option);
        int extractIntegerInput(std::string input);
        long getFileSize(std::string fileName);
        int getIntInput(std::string message);
        std::string getStringInput(std::string message);
        bool flagIsValid(char flag);
        void getStreamDevice();
        int getNumberOfPacketsToRead();
        std::string getInputFileName();

    public:
        ArgumentHandler();
        ArgumentHandler(PacketReceiver* receiver);
        ~ArgumentHandler();
        int handleCommandLineFlags(int argc, char* argv[], PacketReceiver& receiver);
        std::string getOptDesc(int option);
        void enableOption(int option);
        void disableOption(int option);
        bool isOptionSpecified(int option);
        bool isOptionEnabled(int option);
    };
}
#endif //REALTIMELIDAR_ARGUMENTHANDLER_H
