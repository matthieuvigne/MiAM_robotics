/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <miam_utils/raspberry_pi/RPiGPIO.h>
#include "ServoHandler.h"
#include "Parameters.h"
#include <unistd.h>

int const PUMP_PWM = 26;

static int const SERVO_SUCTION[3] = {SUCTION_RIGHT, SUCTION_CENTER, SUCTION_LEFT};
static int const SERVO_TUBE[3] = {VALVE_RIGHT, VALVE_CENTER, VALVE_LEFT};

////////////////////////////////////////////////////////////////
// Core functions
////////////////////////////////////////////////////////////////

ServoHandler::ServoHandler(MaestroDriver *maestro):
    isPumpOn_(false),
    maestro_(maestro)
{
}

bool ServoHandler::init(std::string const& portName)
{
    // Enable pump GPIO.
    #ifndef SIMULATION
    RPi_setupGPIO(PUMP_PWM, PI_GPIO_OUTPUT);
    #endif
    activatePump(false);

    maestro_->setSpeed(STATUE, 700);

    return maestro_->init(portName);
}

void ServoHandler::shutdownServos()
{
	for(int i = 0; i < 18; i++)
	{
		maestro_->setPosition(i, 0);
		usleep(80);
	}
}


////////////////////////////////////////////////////////////////
// Arm motion
////////////////////////////////////////////////////////////////
void ServoHandler::moveArm(bool const& rightArm, arm const& pose)
{
    if (rightArm)
    {
        switch (pose)
        {
            case arm::MEASURE: maestro_->setPosition(RIGHT_ARM, 1645); break;
            case arm::RAISE: maestro_->setPosition(RIGHT_ARM, 1500); break;
            case arm::FOLD: maestro_->setPosition(RIGHT_ARM, 700); break;
            default: break;
        }
    }
    else
    {
        switch (pose)
        {
            case arm::MEASURE: maestro_->setPosition(LEFT_ARM, 1100); break;
            case arm::RAISE: maestro_->setPosition(LEFT_ARM, 1300); break;
            case arm::FOLD: maestro_->setPosition(LEFT_ARM, 2050); break;
            default: break;
        }
    }
}


void ServoHandler::moveFinger(bool const& rightArm, finger const& pose)
{
    if (rightArm)
    {
        switch (pose)
        {
            case finger::PUSH: maestro_->setPosition(RIGHT_FINGER, 510); break;
            case finger::MEASURE: maestro_->setPosition(RIGHT_FINGER, 1900); break;
            case finger::FOLD: maestro_->setPosition(RIGHT_FINGER, 2490); break;
            default: break;
        }
    }
    else
    {
        switch (pose)
        {
            case finger::PUSH: maestro_->setPosition(LEFT_FINGER, 2490); break;
            case finger::MEASURE: maestro_->setPosition(LEFT_FINGER, 1100); break;
            case finger::FOLD: maestro_->setPosition(LEFT_FINGER, 510); break;
            default: break;
        }
    }
}

////////////////////////////////////////////////////////////////
// Suction-related functions
////////////////////////////////////////////////////////////////
void ServoHandler::openValve()
{
    maestro_->setPosition(VALVE, 0);
}

void ServoHandler::closeValve()
{
    maestro_->setPosition(VALVE, 7000);
}

void ServoHandler::openTube(int tubeNumber)
{
    if (tubeNumber < 0 || tubeNumber > 2)
        return;
    maestro_->setPosition(SERVO_TUBE[tubeNumber], 0);
}

void ServoHandler::closeTube(int tubeNumber)
{
    if (tubeNumber < 0 || tubeNumber > 2)
        return;
    maestro_->setPosition(SERVO_TUBE[tubeNumber], 7000);
}


void ServoHandler::activatePump(bool const& pumpOn)
{
    if (pumpOn)
    {
        #ifndef SIMULATION
        RPi_writeGPIO(PUMP_PWM, HIGH);
        #endif
        isPumpOn_ = true;
    }
    else
    {
        #ifndef SIMULATION
        RPi_writeGPIO(PUMP_PWM, LOW);
        #endif
        isPumpOn_ = false;
    }
}


void ServoHandler::moveSuction(int const& suctionNumber, suction const& position)
{
    if (suctionNumber == 0)
    {
        switch (position)
        {
            case suction::FOLD:        maestro_->setPosition(SERVO_SUCTION[0], 2500); break;
            case suction::DROP_SAMPLE:    maestro_->setPosition(SERVO_SUCTION[0], 1200); break;
            case suction::LOWER_SAMPLE:    maestro_->setPosition(SERVO_SUCTION[0], 1300); break;
            case suction::HORIZONTAL:  maestro_->setPosition(SERVO_SUCTION[0], 550); break;
            case suction::VERTICAL:  maestro_->setPosition(SERVO_SUCTION[0], 1600); break;
            default: break;
        }
    }
    else if (suctionNumber == 1)
    {
        switch (position)
        {
            case suction::FOLD:        maestro_->setPosition(SERVO_SUCTION[1], 2500); break;
            case suction::DROP_SAMPLE:    maestro_->setPosition(SERVO_SUCTION[1], 1500); break;
            case suction::LOWER_SAMPLE:    maestro_->setPosition(SERVO_SUCTION[1], 1575); break;
            case suction::HORIZONTAL:  maestro_->setPosition(SERVO_SUCTION[1], 1050); break;
            case suction::DROP_STATUE: maestro_->setSpeed(SERVO_SUCTION[1], 1100);
                                       usleep(5000);
                                       maestro_->setPosition(SERVO_SUCTION[1], 2100);
                                       usleep(500000);
                                       maestro_->setSpeed(SERVO_SUCTION[1], 0); break;
            case suction::VERTICAL:  maestro_->setPosition(SERVO_SUCTION[1], 1700); break;
            default: break;
        }
    }
    else
    {
        switch (position)
        {
            case suction::FOLD:        maestro_->setPosition(SERVO_SUCTION[2], 500); break;
            case suction::DROP_SAMPLE:    maestro_->setPosition(SERVO_SUCTION[2], 1800); break;
            case suction::LOWER_SAMPLE:    maestro_->setPosition(SERVO_SUCTION[2], 1700); break;
            case suction::HORIZONTAL:  maestro_->setPosition(SERVO_SUCTION[2], 2450); break;
            case suction::VERTICAL:  maestro_->setPosition(SERVO_SUCTION[2], 1425); break;
            default: break;
        }
    }
}


void ServoHandler::moveRail(int velocity)
{
    maestro_->setPosition(ELEVATOR, velocity);
}

////////////////////////////////////////////////////////////////
// Statue-related functions
////////////////////////////////////////////////////////////////
void ServoHandler::activateMagnet(bool const& magnetOn)
{
    if (magnetOn)
        maestro_->setPosition(MAGNET, 7000);
    else
        maestro_->setPosition(MAGNET, 0);
}

void ServoHandler::moveStatue(statue const& pose)
{
    switch(pose)
    {
        case statue::TRANSPORT: maestro_->setPosition(STATUE, 1975); break;
        case statue::CATCH: maestro_->setPosition(STATUE, 1780); break;
        case statue::DROP: maestro_->setPosition(STATUE, 1850); break;
        case statue::FOLD: maestro_->setPosition(STATUE, 900); break;
        default: break;
    }
}

void ServoHandler::moveClaw(claw const& pose)
{
    int const FOLD_POS[2] = {775, 2150};
    switch(pose)
    {
        case claw::FOLD: maestro_->setPosition(RIGHT_CLAW, FOLD_POS[0]);
                         maestro_->setPosition(LEFT_CLAW, FOLD_POS[1]); break;
        case claw::FRONT: maestro_->setPosition(RIGHT_CLAW, FOLD_POS[0] + 650);
                         maestro_->setPosition(LEFT_CLAW, FOLD_POS[1] - 650); break;
        case claw::SIDE: maestro_->setPosition(RIGHT_CLAW, FOLD_POS[0] + 1550);
                         maestro_->setPosition(LEFT_CLAW, FOLD_POS[1] - 1550); break;
        default: break;
    }
}
