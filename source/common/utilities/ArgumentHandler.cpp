//
// Created by marko on 8/19/16.
//

#include "ArgumentHandler.h"

namespace RealTimeLidar {
    ArgumentHandler::ArgumentHandler() : cmdOptions{} {

    }

    ArgumentHandler::ArgumentHandler(PacketReceiver* receiver) : cmdOptions{} {
            this->receiver = receiver;
    }

    ArgumentHandler::~ArgumentHandler() {

    }

    /* Get option description. */
    std::string ArgumentHandler::getOptDesc(int option) {
        switch(option) {
            case GRAPHICS:
                return "Graphical Display";
            case STREAM:
                return "Data Streaming";
            case WRITE:
                return "Data Writing";
            case FORWARD:
                return "Data Forwarding";
            default:
                return "Unknown Option";
        }
    }

    void ArgumentHandler::enableOption(int option) {
        cmdOptions[option] = ON;
        std::cout << getOptDesc(option) << " ENABLED." << std::endl;

        // extra stuff that needs to be done in special cases
        if (option == WRITE) {
            std::string input = getStringInput("What would you like to name the output data file?");
            receiver->setOutputDataFileName(input);
            std::cout << "Data will be written to: " << receiver->getOutputDataFileName() << std::endl;
        } else
        if (option == STREAM) {
            getStreamDevice();
        }
    }

    void ArgumentHandler::disableOption(int option) {
        cmdOptions[option] = OFF;
        std::cout << getOptDesc(option) << " DISABLED." << std::endl;
    }

    /* Prompts the user for the desired method of streaming. */
    void ArgumentHandler::getStreamDevice() {
        int inputInt = -1;
        while (inputInt == -1) {
            inputInt = getIntInput("Select the device you are streaming data from (enter the associated number):\n"
                                    "1. Velodyne\n"
                                    "2. File\n");
            switch (inputInt) {
                case (1): {
                    receiver->setStreamMedium(VELODYNE);
                    std::cout << "Selected streaming medium: VELODYNE." << std::endl;
                    break;
                }
                case (2): {
                    receiver->setStreamMedium(INPUTFILE);
                    std::cout << "Selected streaming medium: FILE." << std::endl;
                    receiver->setInputDataFileName(getInputFileName());
                    receiver->setNumPacketsToRead(getNumberOfPacketsToRead());
                    break;
                }
                default: {
                    std::cout << "Invalid input. Please select a valid number." << std::endl;
                    inputInt = -1;
                    break;
                }
            }
        }
    }

    /* Prompts user for the desired number of packets to be read from the input file. */
    int ArgumentHandler::getNumberOfPacketsToRead() {
        int count = getIntInput("How many packets should be read in from this file?\n");
        if (count < 1)
            count = 0;
        return count;
    }

    /* Prompts the user for the input file they would like to use. */
    std::string ArgumentHandler::getInputFileName() {
        std::string name = getStringInput("Please enter the name of the file to use (be sure to include .dat\n");
        long numBytes = getFileSize(name);
        std::cout << "The file \"" << name << "\" (" << numBytes << " bytes | " << numBytes / 1206 << " packets) " <<
                  "will be used as input." << std::endl;
        return name;
    }

    /* Returns the size of the given file in bytes. */
    long ArgumentHandler::getFileSize(std::string fileName) {
        std::ifstream in(fileName.c_str(), std::ios::binary | std::ios::ate);
        return in.tellg();
    }

    bool ArgumentHandler::isOptionSpecified(int option) {
        return (cmdOptions[option] != UNKNOWN);
    }

    bool ArgumentHandler::isOptionEnabled(int option) {
        return (cmdOptions[option] >= ON);
    }

    bool ArgumentHandler::flagIsValid(char flag) {
        switch (flag) {
            case 'g':
            case 's':
            case 'f':
            case 'w':
            case 'h':
                return true;
            default:
                return false;
        }
    }

    void ArgumentHandler::checkOptionInput(char &input, int option) {
        std::cin.get(input);
        std::cin.ignore(256, '\n');
        if (input == 'y' || input == 'Y') {
            enableOption(option);
        } else if (input == 'n' || input == 'N') {
            disableOption(option);
        } else {
            std::cout << "Please enter either 'y' or 'n'." << std::endl;
        }
    }

    int ArgumentHandler::handleCommandLineFlags(int argc, char* argv[], PacketReceiver& receiver) {
        /* Deal with the command line flags: */
        /* Validate flags: */
        for (int fc = 1; fc < argc; ++fc) {
            std::string arg = std::string(argv[fc]);
            /* Handle multiple flags in one argument: */
            if (arg[0] == '-') {
                for (size_t i = 1; i < arg.length(); ++i) {
                    if (! flagIsValid(arg[i])) {
                        std::cout << arg[i] << " is an invalid flag. Ending program." << std::endl;
                        return 1;
                    }
                }
            } else {
                std::cout << arg << " is an invalid flag." << std::endl;
            }
        }
        /* Default flag values are set to false: */
        for (int fc = 1; fc < argc; ++fc) {
            std::string arg = std::string(argv[fc]);
            /* Handle multiple flags in one argument: */
            if (arg[0] == '-') {
                for(size_t i = 1; i < arg.length(); ++i) {
                    switch(arg[i]) {
                        case 'g':
                            enableOption(GRAPHICS);
                            break;
                        case 's':
                            enableOption(STREAM);
                            break;
                        case 'f':
                            enableOption(FORWARD);
                            break;
                        case 'w':
                            enableOption(WRITE);
                            break;
                        case 'h':
                            std::cout << "HELP ME PLEASE!" << std::endl;                     // ADD HELP FLAG
                            break;
                        default:
                            std::cout << arg[i] << " is an invalid flag. Ending program." << std::endl;
                            return 1;
                    }
                }
            } else {
                std::cout << arg << " is an invalid flag." << std::endl;
            }
        }

        /* For the necessary options that the user did not specify with flags, prompt. */
        char input;
        /* Check for gui flag: */
        while (!isOptionSpecified(GRAPHICS)) {
            std::cout << "Enable graphical display? (y/n) ";
            checkOptionInput(input, GRAPHICS);
        }
        while (!isOptionSpecified(STREAM)) {
            std::cout << "Enable data streaming? (y/n) ";
            checkOptionInput(input, STREAM);
        }
        if (isOptionEnabled(STREAM)) {
            while (!isOptionSpecified(WRITE)) {
                std::cout << "Enable writing data to a file? (y/n) ";
                checkOptionInput(input, WRITE);
            }
            while (!isOptionSpecified(FORWARD)) {
                std::cout << "Enable forwarding of data to a different location? (y/n) ";
                checkOptionInput(input, FORWARD);
            }
        }
        return 0;
    }

    std::string ArgumentHandler::getStringInput(std::string message) {
        std::cout << message;
        std::string input;
        std::getline(std::cin, input);
        return input;
    }

    int ArgumentHandler::getIntInput(std::string message) {
        std::cout << message;
        std::string input;
        std::getline(std::cin, input);
        return extractIntegerInput(input);
    }

    /* Converts a string to an int with necessary exception handling. */
    int ArgumentHandler::extractIntegerInput(std::string input) {
        int ret = -1;
        try {
            ret = std::stoi(input);
        } catch (const std::invalid_argument &ia) {
            std::cout << "Invalid input. Please select a valid number." << std::endl;
        } catch (const std::out_of_range &oor) {
            std::cout << "Input was out of range. Please select a valid number." << std::endl;
        }
        return ret;
    }
}