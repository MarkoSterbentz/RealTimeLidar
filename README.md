# Real-Time LiDAR Mapping and Analysis
#### Background Information
The aim of this project is to develop software for the real-time reception and analysis of data gathered by a LiDAR-equipped UAV. The user can view and analyze the incoming data as the UAV is in flight. Additionally, the user will be able to see an interactive model of the landscape with selected analyses applied on demand. This software will improve researchers capabilities to easily map the environment, which will facilitate the improvement of many ecosystem services.

Development is ongoing. For the time being, this software is Linux only. Also worth noting is that this repository is a continuation of the [LaserMappingDrone](https://github.com/MarkoSterbentz/LaserMappingDrone) project.  

#### Necessary Dependencies:
 * CMake  
 * OpenGL  
 * SDL  
 * GLEW  

#### Ubuntu Build Instructions


1. Install necessary dependencies:  

    ```
    sudo apt-get install cmake
    sudo apt-get install freeglut3-dev
    sudo apt-get install libsdl2-dev
    sudo apt-get install libglew-dev
    ```

2. Clone this git repo and submodules:  

    ```
    git clone https://github.com/MarkoSterbentz/RealTimeLidar
    git submodule init
    git submodule update
    ```
3. Compile the project  
    - If building with an IDE like CLion, simply import the project and build.  
    - If using CMake:
  
        ```
        mkdir build
        cmake ..
        make
        make install
        ```

#### Running the Project
This section will be updated in the future.