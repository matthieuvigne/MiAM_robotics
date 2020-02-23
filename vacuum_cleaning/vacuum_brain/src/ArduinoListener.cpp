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

//~ #include <cmath>

#include <iostream>


double const LOOP_PERIOD = 0.010;



// Global variables: a uCData struct protected by a mutex.
std::mutex uCMutex;


ArduinoListener::ArduinoListener() :
    metronome_listener(LOOP_PERIOD * 1e9),
    metronome_writer(LOOP_PERIOD * 1e9),
    
    currentTime_listener(0),
    lastTime_listener(0),

    currentTime_writer(0),
    lastTime_writer(0)
    {
}

bool ArduinoListener::intialize(
    omni::ThreeWheelsKinematics const& kinematics, 
    std::string const& portName
    ) {
    
    kinematics_ = kinematics;
    int port = uart_open(portName, B115200);
    
    if (port < 0) {
        // Error
        return false;
    }
    
    port_ = port;
    
    // Start the listening thread.
    std::thread listenerThread(&ArduinoListener::uCListener_listenerThread, this);
    listenerThread.detach();
    
    // Start the writing thread.
    std::thread writingThread(&ArduinoListener::uCListener_writerThread, this);
    writingThread.detach();
    
    return true; 
}


void ArduinoListener::setTarget(omni::BaseSpeed const& targetBaseSpeed) {
    currentTarget_ = targetBaseSpeed;
}


omni::BaseSpeed ArduinoListener::getCurrentSpeed() {
    uCMutex.lock();
    omni::WheelSpeed currentWheelSpeed = currentWheelSpeed_;
    uCMutex.unlock();
    return kinematics_.forwardKinematics(currentWheelSpeed);
}

omni::ThreeWheelsKinematics *ArduinoListener::getKinematics() {
    return &kinematics_;
}


void ArduinoListener::uCListener_listenerThread() {
    // Init data structure.
    // TODO

    //~ while(true)
    //~ {
        //~ WheelSpeed currentWheelSpeed;
        
        //~ // Read wheel speed
        
        //~ // Save speed
        //~ uCMutex.lock();
        //~ currentWheelSpeed_ = currentWheelSpeed;
        //~ uCMutex.unlock();
    //~ }
}

void ArduinoListener::uCListener_writerThread() {
    
    while (true)
    {
        // Wait for next tick.
        lastTime_writer = currentTime_writer;
        metronome_writer.wait();
        currentTime_writer = metronome_writer.getElapsedTime();
        lastTime_writer = currentTime_writer;
        
        unsigned char message[9];
        
        // Conversion
        omni::WheelSpeed targetWheelSpeed = kinematics_.inverseKinematics(currentTarget_);
        
        // Write
        message[0] = 0xFF;
        message[1] = 0xFF;
        int wheelspeed = int(targetWheelSpeed.wheelSpeed_[0] * 3600 / 2.0 / 3.14159) + 16384;
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
        write(port_, message, 9);
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
