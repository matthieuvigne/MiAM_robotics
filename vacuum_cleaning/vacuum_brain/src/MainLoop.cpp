/// \file MainLoop.c
/// \brief Main function: code for inverted pendulum.
///
/// \author Matthieu Vigne
/// \copyright GNU GPLv3

#include "LoggerFields.h"
#include <miam_utils/raspberry_pi/RaspberryPi.h>
#include <miam_utils/trajectory/ThreeWheelsKinematics.hpp>
//~ #include "Utilities.h"

#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <cmath>

// Update loop frequency, in s.
//~ double const LOOP_PERIOD = 0.005;
double const LOOP_PERIOD = 0.010;

// Stop motor before exit.
void killCode(int x)
{
    //~ sendCommandToMotors(0.0, 0.0);
    exit(0);
}


int main(int argc, char **argv)
{
    // Init raspberry serial ports and GPIO.
    //~ RPi_enablePorts();
    // Configure CTRL+C signal.
    //~ signal(SIGINT, killCode);
    //~ signal(SIGTERM, killCode);
    
    // Init com with arduino
    
    //TODO
    //~ int port = uart_open("/dev/ttyACM0", B115200);
    //~ int port = uart_open("/dev/ttyACM1", B115200);
    //~ int port = uart_open("/dev/ttyACM2", B115200);
    int port = uart_open("/dev/ttyACM4", B115200);
    
    Logger logger = initLog();
    
    // Configure and start main loop.
    Metronome metronome(LOOP_PERIOD * 1e9);
    double currentTime = 0;
    double lastTime = 0;
    
    omni::BaseSpeed targetSpeed;
    omni::ThreeWheelsKinematics kinematics(0.10, 0.05);
    
    while (true)
    {
        // Wait for next tick.
        lastTime = currentTime;
        metronome.wait();
        currentTime = metronome.getElapsedTime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        
        targetSpeed = omni::BaseSpeed(0.5 * std::sin(currentTime),0.0, 0.0);
        omni::WheelSpeed targetWheelSpeed = kinematics.inverseKinematics(targetSpeed);
        std::cout << targetWheelSpeed.wheelSpeed_[0] << std::endl;
        
        int wheelspeed = int(targetWheelSpeed.wheelSpeed_[0] * 3600 / 2.0 / 3.14159) + 16384;
        std::cout << "here" << currentTime << std::endl;
        
        unsigned char message[9];
        message[0] = 0xFF;
        message[1] = 0xFF;
        message[2] = wheelspeed & 0xFF;
        message[3] = (wheelspeed >> 8) & 0xFF;
        
        wheelspeed = int(targetWheelSpeed.wheelSpeed_[1] * 3600 / 2.0 / 3.14159) + 16384;
        message[4] = wheelspeed & 0xFF;
        message[5] = (wheelspeed >> 8) & 0xFF;
        wheelspeed = int(targetWheelSpeed.wheelSpeed_[2] * 3600 / 2.0 / 3.14159) + 16384;
        message[6] = wheelspeed & 0xFF;
        message[7] = (wheelspeed >> 8) & 0xFF;
        message[8] = 0;
        for(int i = 2; i < 8; i++)
            message[8] += message[i];
        std::cout << "Writing..." << metronome.getElapsedTime() << std::endl;
        write(port, message, 9);
        std::cout << "Write done" << metronome.getElapsedTime() << std::endl;
    
        // Log
        logger.setData(LOGGER_TIME, currentTime);
        logger.writeLine();
    }
    return 0;
}

