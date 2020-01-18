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
double const LOOP_PERIOD = 0.005;

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
    signal(SIGINT, killCode);
    signal(SIGTERM, killCode);
    
    // Init com with arduino
    
    //TODO
    int port = uart_open("/dev/ttyACM0", B115200);
    
    Logger logger = initLog();
    
    // Configure and start main loop.
    Metronome metronome(LOOP_PERIOD * 1e9);
    double currentTime = 0;
    double lastTime = 0;
    
    ThreeWheelsKinematics::BaseSpeed targetSpeed;
    ThreeWheelsKinematics kinematics(0.10);
    
    while (true)
    {
        // Wait for next tick.
        lastTime = currentTime;
        metronome.wait();
        currentTime = metronome.getElapsedTime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        
        targetSpeed = ThreeWheelsKinematics::BaseSpeed(0.1 * std::sin(currentTime),0.0, 0.0);
        ThreeWheelsKinematics::WheelSpeed targetWheelSpeed = kinematics.inverseKinematics(targetSpeed);
        
        int wheelspeed = int((targetWheelSpeed.wheel1_ / 0.05) * 3600 / 2.0 / 3.14159) + 16384;
        
         
        unsigned char message[8];
        message[0] = 0xFF;
        message[1] = 0xFF;
        message[2] = wheelspeed & 0xFF;
        message[3] = (wheelspeed >> 8) & 0xFF;
        wheelspeed = int((targetWheelSpeed.wheel2_ / 0.05) * 3600 / 2.0 / 3.14159) + 16384;
        message[4] = wheelspeed & 0xFF;
        message[5] = (wheelspeed >> 8) & 0xFF;
        wheelspeed = int((targetWheelSpeed.wheel3_ / 0.05) * 3600 / 2.0 / 3.14159) + 16384;
        message[6] = wheelspeed & 0xFF;
        message[7] = (wheelspeed >> 8) & 0xFF;
        write(port, message, 8);
        for(int i = 0; i < 8; i++)
            std::cout << int(message[i]) << std::endl;
    
        // Log
        logger.setData(LOGGER_TIME, currentTime);
        logger.writeLine();
    }
    return 0;
}

