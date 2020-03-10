/// \file MainLoop.c
/// \brief Main function: code for inverted pendulum.
///
/// \author Matthieu Vigne
/// \copyright GNU GPLv3

#include "LoggerFields.h"
#include "ArduinoListener.h"
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

int const ENCODER_RESOLUTION = 3600; // Encoder resolution

int main(int argc, char **argv)
{
    // Init raspberry serial ports and GPIO.
    //~ RPi_enablePorts();
    
    // Create kinematics object representing the robot.
    omni::ThreeWheelsKinematics kinematics(0.10, 0.05);
    
    // Init log
    Logger logger = initLog();
    
    // Init communication with arduino
    ArduinoListener arduino(kinematics, ENCODER_RESOLUTION);
    bool isArduinoInit = arduino.initialize("/dev/ttyACM0");
    
    if (!isArduinoInit)
    {
        std::cout << "Failed to init communication with Arduion" << std::endl;
        exit(0);
    }
    
    // Configure and start main loop.
    Metronome metronome(LOOP_PERIOD * 1e9);
    double currentTime = 0;
    double lastTime = 0;
    omni::BaseSpeed targetSpeed, currentSpeed;
    omni::WheelSpeed targetWheelSpeed, currentWheelSpeed;
    while (true)
    {
        // Wait for next tick.
        lastTime = currentTime;
        metronome.wait();
        currentTime = metronome.getElapsedTime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        
        targetSpeed = omni::BaseSpeed(0.0, 0.2 * std::sin(currentTime), 0.0);
        
        arduino.setTarget(targetSpeed);
        currentSpeed = arduino.getCurrentSpeed();
        
        // Log
        logger.setData(LOGGER_TIME, currentTime);
        logger.setData(LOGGER_TARGET_VELOCITY_X, targetSpeed.vx_);
        logger.setData(LOGGER_TARGET_VELOCITY_Y, targetSpeed.vy_);
        logger.setData(LOGGER_TARGET_VELOCITY_OMEGA, targetSpeed.omega_);
        logger.setData(LOGGER_CURRENT_VELOCITY_X, currentSpeed.vx_);
        logger.setData(LOGGER_CURRENT_VELOCITY_Y, currentSpeed.vy_);
        logger.setData(LOGGER_CURRENT_VELOCITY_OMEGA, currentSpeed.omega_);
        
        targetWheelSpeed = kinematics.inverseKinematics(targetSpeed);
        currentWheelSpeed = kinematics.inverseKinematics(currentSpeed);
        
        logger.setData(LOGGER_TARGET_WHEEL_VELOCITY_1, targetWheelSpeed.w_[0]);
        logger.setData(LOGGER_TARGET_WHEEL_VELOCITY_2, targetWheelSpeed.w_[1]);
        logger.setData(LOGGER_TARGET_WHEEL_VELOCITY_3, targetWheelSpeed.w_[2]);
        logger.setData(LOGGER_CURRENT_WHEEL_VELOCITY_1, currentWheelSpeed.w_[0]);
        logger.setData(LOGGER_CURRENT_WHEEL_VELOCITY_2, currentWheelSpeed.w_[1]);
        logger.setData(LOGGER_CURRENT_WHEEL_VELOCITY_3, currentWheelSpeed.w_[2]);
        logger.writeLine();
    }
    return 0;
}

