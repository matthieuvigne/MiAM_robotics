/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "ArduinoListener.h"

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <mutex>
#include <thread>

#include <math.h>

//~ #include <cmath>

#include <iostream>

// Length of input message from Arduino
#define ARDUINO_MESSAGE_LENGTH 7

ArduinoListener::ArduinoListener(omni::ThreeWheelsKinematics const& kinematics, int const& encoderResolution) :
    kinematics_(kinematics),
    port_(-1),
    mutex_(),
    SI_TO_TICKS_(encoderResolution / 2.0 / M_PI),
    lastWriteTime_(0.0)
{
    // EmptySI_TO_TICKS
}


bool ArduinoListener::initialize(std::string const& portName)
{
    // If already initialize (i.e. a thread is already running), don't allow for a new configuration.
    //~ if (isInitialized_)
        //~ return false;

    port_ = uart_open(portName, B115200);

    if (port_ < 0)
    {
        // Error
        return false;
    }

    // Start the communication thread.
    std::thread thread(&ArduinoListener::communicationThread, this);
    thread.detach();

    return true;
}


void ArduinoListener::setTarget(omni::BaseSpeed const& targetBaseSpeed)
{
    mutex_.lock();
    targetWheelSpeed_ = kinematics_.inverseKinematics(targetBaseSpeed);
    // Force new write as soon as possible.
    lastWriteTime_ = -1.0;
    mutex_.unlock();
}


omni::BaseSpeed ArduinoListener::getCurrentSpeed()
{
    mutex_.lock();
    omni::WheelSpeed currentWheelSpeed = currentWheelSpeed_;
    mutex_.unlock();
    return kinematics_.forwardKinematics(currentWheelSpeed);
}


omni::ThreeWheelsKinematics *ArduinoListener::getKinematics()
{
    return &kinematics_;
}


double const TARGET_UPDATE_PERIOD = 0.004; // Time increment to send new target, in s.

void ArduinoListener::communicationThread()
{
    // Init
    // Metronome object: simple wrapper for getting current time.
    Metronome timer(0.1);
    lastWriteTime_ = 0.0;

    unsigned char arduinoMessage[ARDUINO_MESSAGE_LENGTH];
    unsigned char readData[ARDUINO_MESSAGE_LENGTH + 2];
    int positionInMessage = -1;
    bool lastWasFF = false;

    while(true)
    {
        double time = timer.getElapsedTime();

        // Send new target to arduino.
        if (time - lastWriteTime_ > TARGET_UPDATE_PERIOD)
        {
            // Send message to Arduino.
            unsigned char message[9];
            // Header
            message[0] = 0xFF;
            message[1] = 0xFF;
            for(int i = 0; i < 3; i++)
            {
                // Send target as 2s complement
                uint16_t wheelspeed = (1 << 14) + int16_t(targetWheelSpeed_.w_[i] * SI_TO_TICKS_);
                message[2 + 2 * i] = wheelspeed & 0xFF;
                message[3 + 2 * i] = (wheelspeed >> 8) & 0xFF;
            }
            // Checksum
            message[8] = 0;
            for(int i = 2; i < 8; i++)
                message[8] += message[i];
            // Send message
            write(port_, message, 9);
            lastWriteTime_ = time;
        }

        // Read message from Arduino
        int nBytesRead = read_timeout(port_, readData, ARDUINO_MESSAGE_LENGTH + 2, 1);

        // Process data from Arduino.
        for(int i = 0; i < nBytesRead; i++)
        {
            unsigned char newData = readData[i];
            // If it's a 0xFF, and if the previous byte was also a 0xFF, a new message starts.
            if(newData == 0xFF && lastWasFF)
                positionInMessage = 0;
            else
            {
                lastWasFF = newData == 0xFF;

                // If we are currently reading a message, add it to the buffer.
                if(positionInMessage > -1)
                {
                    arduinoMessage[positionInMessage] = newData;
                    positionInMessage ++;
                }
                // If the end of a message was reached, decode it.
                if(positionInMessage == ARDUINO_MESSAGE_LENGTH)
                {
                    // Reset status.
                    lastWasFF = false;
                    positionInMessage = -1;

                    // Verify checksum.
                    // Sum of previous bytes must be equal to the checksum.
                    uint8_t checksum = 0;
                    for(int i = 0; i < ARDUINO_MESSAGE_LENGTH - 1; i++)
                        checksum += arduinoMessage[i];
                    if(checksum != arduinoMessage[ARDUINO_MESSAGE_LENGTH - 1])
                    {
                        #ifdef DEBUG
                            std::cout << "[uCListener] Invalid checksum, refusing packet" << std::endl;
                        #endif
                    }
                    else
                    {
                        // Decode message.
                        // Get current encoder value.
                        mutex_.lock();
                        for(int i = 0; i < 3; i++)
                        {
                            // Decode data, stored as 2s complement.
                            int16_t wheelSpeedTicks = (1 << 15) - ((arduinoMessage[0 + 2 * i] << 8) + arduinoMessage[1 + 2 * i]);
                            currentWheelSpeed_.w_[i] = wheelSpeedTicks / SI_TO_TICKS_;
                        }
                        mutex_.unlock();
                    }
                }
            }


        }
    }
}

