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




ArduinoListener::ArduinoListener(omni::ThreeWheelsKinematics const& kinematics, int const& encoderResolution) :
    kinematics_(kinematics),
    port_(-1),
    mutex_(),
    SI_TO_TICKS(encoderResolution / 2.0 / M_PI)
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
    double lastWriteTime = 0.0;
    while(true)
    {
        double time = timer.getElapsedTime();
        if (time - lastWriteTime > TARGET_UPDATE_PERIOD)
        {
            // Send message to Arduino.
            unsigned char message[9];
            // Header
            message[0] = 0xFF;
            message[1] = 0xFF;
            for(int i = 0; i < 3; i++)
            {
                int wheelspeed = int(targetWheelSpeed_.wheelSpeed_[i] * SI_TO_TICKS) + 16384;
                message[2 + 2 * i] = wheelspeed & 0xFF;
                message[3 + 2 * i] = (wheelspeed >> 8) & 0xFF;
            }
            // Checksum
            message[8] = 0;
            for(int i = 2; i < 8; i++)
                message[8] += message[i];
            // Send message
            write(port_, message, 9);
            lastWriteTime = time;
            
            // Flush port...
        }
    }
}



//~ // Global variables: a uCData struct protected by a mutex.
//~ std::mutex uCMutex;
//~ uCData listenerData;

//~ // State in the message:
//~ bool lastWasFF = false; //If the last byte was a 0xFF byte.
//~ int positionInMessage = -1;    // Where we are currently writting in the buffer.
//~ unsigned char buffer[MESSAGE_LENGTH];

//~ // For encoder: last value.
//~ int16_t lastEncoderValue[2] = {0, 0};
//~ // First read: take current value as 0.
//~ bool isFirstRead = true;

//~ void uCListener_listenerThread(int const& port)
//~ {
    //~ // Init data structure.
    //~ listenerData.encoderValues[0] = 0.0;
    //~ listenerData.encoderValues[1] = 0.0;

    //~ while(true)
    //~ {
        //~ // Read a single byte from the serial port.
        //~ unsigned char lastData = 0;
        //~ int result = read(port, &lastData, 1);
        //~ if(result < 0)
            //~ printf("Error reading from microcontroller: %d\n", errno);
        //~ else
        //~ {
            //~ // If it's a 0xFF, and if the previous byte was also a 0xFF, a new message starts.
            //~ if(lastData == 0xFF && lastWasFF)
                //~ positionInMessage = 0;
            //~ else
            //~ {
                //~ lastWasFF = lastData == 0xFF;

                //~ // If we are currently reading a message, add it to the buffer.
                //~ if(positionInMessage > -1)
                //~ {
                    //~ buffer[positionInMessage] = lastData;
                    //~ positionInMessage ++;
                //~ }
                //~ // If the end of a message was reached, decode it.
                //~ if(positionInMessage == MESSAGE_LENGTH)
                //~ {
                    //~ // Reset status.
                    //~ lastWasFF = false;
                    //~ positionInMessage = -1;

                    //~ // Verify checksum.
                    //~ // Sum of previous bytes must be equal to the checksum.
                    //~ uint8_t checksum = 0;
                    //~ for(int i = 0; i < MESSAGE_LENGTH - 1; i++)
                        //~ checksum += buffer[i];
                    //~ if(checksum != buffer[MESSAGE_LENGTH - 1])
                    //~ {
                        //~ #ifdef DEBUG
                            //~ printf("[uCListener] Invalid checksum, refusing packet\n");
                        //~ #endif
                    //~ }
                    //~ else
                    //~ {
                        //~ // Decode message.
                        //~ // Get current encoder value.
                        //~ for(int i = 0; i < 2; i++)
                        //~ {
                            //~ int16_t encoderValue = (1 << 15) - ((buffer[0 + 2 * i] << 8) + buffer[1 + 2 * i]);

                            //~ if(isFirstRead)
                            //~ {
                                //~ isFirstRead = false || (i == 0);
                                //~ lastEncoderValue[i] = encoderValue;
                            //~ }
                            //~ // Determine the direction of the encoder from the shortest travel possible (i.e. we assume
                            //~ // that between two successive read, the encoder has turned by less than 16000 cournts, i.e. 2 turns).
                            //~ int32_t deltaEncoder = encoderValue - lastEncoderValue[i];
                            //~ while(deltaEncoder > (1 << 14) - 1)
                                //~ deltaEncoder -= 2 * (1 << 14);
                            //~ while(deltaEncoder < -(1 << 14))
                                //~ deltaEncoder += 2 * (1 << 14);
                            //~ lastEncoderValue[i] = encoderValue;

                            //~ double encoderIncrement = deltaEncoder * ENCODER_RESOLUTION;
                            //~ uCMutex.lock();
                            //~ listenerData.encoderValues[i] += encoderIncrement;
                            //~ uCMutex.unlock();
                        //~ }
                        //~ // Potentiometer value.
                        //~ uCMutex.lock();
                        //~ listenerData.potentiometerPosition = (buffer[4] << 8) + buffer[5];
                        //~ uCMutex.unlock();
                    //~ }
                //~ }
            //~ }
        //~ }
    //~ }
//~ }


//~ bool uCListener_start(std::string const& portName)
//~ {
    //~ // Start communication with the arduino.

    //~ // Open communication
    //~ int port = uart_open(portName, B1000000);
    //~ if(port < 0)
        //~ return false;

    //~ // Check that an Arduino with the MiAMSlave code is present.
    //~ unsigned char returnData[10];
    //~ returnData[9] = '\0';
    //~ int returnValue = read_timeout(port, returnData, 9, 2500);
    //~ if(returnValue < 9 || strcmp((char*)returnData, "MiAMSlave") != 0)
    //~ {
        //~ std::cout << "Didn't recieve correct message from Arduino slave: expected MiAMSlave, got \"" << returnData << "\"" << std::endl;
        //~ return false;
    //~ }

    //~ // Start the listening thread.
    //~ std::thread listenerThread(uCListener_listenerThread, port);
    //~ listenerThread.detach();
    //~ return true;
//~ }


//~ uCData uCListener_getData()
//~ {
    //~ uCMutex.lock();
    //~ uCData currentData = listenerData;
    //~ uCMutex.unlock();
    //~ return currentData;
//~ }
