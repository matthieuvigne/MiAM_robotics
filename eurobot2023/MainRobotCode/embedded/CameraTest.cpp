/// \file MainLoop.c
/// \brief This file implements the main function, as well as several other features.
///
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

// g++ src/CameraClient.cpp CameraTest.cpp ../../../vision/src/network/message.cpp ../../../vision/src/network/server_response.cpp ../../../vision/src/common/marker.cpp ../../../vision/src/common/logger.cpp ../../../vision/src/common/maths.cpp ../../../vision/src/network/socket.cpp ../../../vision/src/network/client_*.cpp -o cameraTest --std=c++11 `pkg-config --cflags --libs eigen3` -Iinclude/ -I../../../vision/include/ -lpthread

#include "CameraClient.h"

#include <iostream>
#include <thread>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>



int main(int argc, char **argv)
{
    CameraClient camera;

    std::thread camThread(&CameraClient::run, &camera);
    camThread.detach();

    while(true)
    {
        usleep(500000);
        std::cout << "excavation site:" << camera.getNumberOfMarkersInExcavationSite() << std::endl;
    }
    // while(!client.init("192.168.7.2", 30000))
    //     usleep(500000);
    // bool right = true;
    // client.run(&right);
    return 0;
}

