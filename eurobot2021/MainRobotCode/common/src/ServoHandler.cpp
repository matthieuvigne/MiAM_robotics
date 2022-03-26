/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <miam_utils/raspberry_pi/RPiGPIO.h>
#include "ServoHandler.h"
#include "Parameters.h"
#include <unistd.h>

int const PUMP_PWM = 26;

static int const SERVO_SUCTION[3] = {SUCTION_RIGHT, SUCTION_CENTER, SUCTION_LEFT};

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
    turnOffPump();

    return maestro_->init(portName);
}


void ServoHandler::ouvrirlebrasdroitmilieu() {

     maestro_->setPosition(RIGHT_ARM, 1500);
    //maestro_->setPosition(SUCTION_RIGHT, 1500);
    //maestro_->setPosition(RIGHT_FINGER, 1500);
    // maestro_->setPosition(SUCTION_MILIEU, 1500);
    // maestro_->setPosition(SUCTION_LEFT, 1500);
    // maestro_->setPosition(LEFT_ARM, 1500);
}


void ServoHandler::ouvrirlebrasdroitbas() {

     maestro_->setPosition(RIGHT_ARM, 1980);
    //maestro_->setPosition(SUCTION_RIGHT, 1980);
    //maestro_->setPosition(RIGHT_FINGER, 1980);
    // maestro_->setPosition(SUCTION_MILIEU, 1980);
    // maestro_->setPosition(SUCTION_LEFT, 1980);
    // maestro_->setPosition(LEFT_ARM, 1020);
}


void ServoHandler::ouvrirlebrasdroithaut() {

     maestro_->setPosition(RIGHT_ARM, 1020);
    //maestro_->setPosition(SUCTION_RIGHT, 1020);
    //maestro_->setPosition(RIGHT_FINGER, 1020);
    // maestro_->setPosition(SUCTION_MILIEU, 1020);
    // maestro_->setPosition(SUCTION_LEFT, 1020);
    // maestro_->setPosition(LEFT_ARM, 1980);
}

void ServoHandler::ouvrirlebrasgauchemilieu() {

    // maestro_->setPosition(RIGHT_ARM, 1500);
    //maestro_->setPosition(SUCTION_RIGHT, 1500);
    //maestro_->setPosition(LEFT_FINGER, 1500);
    // maestro_->setPosition(SUCTION_MILIEU, 1500);
    // maestro_->setPosition(SUCTION_LEFT, 1500);
     maestro_->setPosition(LEFT_ARM, 1500);
}


void ServoHandler::ouvrirlebrasgauchebas() {

    // maestro_->setPosition(RIGHT_ARM, 1980);
    //maestro_->setPosition(SUCTION_RIGHT, 1980);
    //maestro_->setPosition(LEFT_FINGER, 1980);
    // maestro_->setPosition(SUCTION_MILIEU, 1980);
    // maestro_->setPosition(SUCTION_LEFT, 1980);
    maestro_->setPosition(LEFT_ARM, 1020);
}


void ServoHandler::ouvrirlebrasgauchehaut() {

    // maestro_->setPosition(RIGHT_ARM, 1020);
    //maestro_->setPosition(SUCTION_RIGHT, 1020);
    //maestro_->setPosition(LEFT_FINGER, 1020);
    // maestro_->setPosition(SUCTION_MILIEU, 1020);
    // maestro_->setPosition(SUCTION_LEFT, 1020);
     maestro_->setPosition(LEFT_ARM, 1980);
    
 }
 
  void ServoHandler::baisserledoigtdroit() {

    maestro_->setPosition(RIGHT_FINGER, 1980);
    //maestro_->setPosition(SUCTION_RIGHT, 1020);
    //maestro_->setPosition(LEFT_FINGER, 1020);
    // maestro_->setPosition(SUCTION_MILIEU, 1020);
    // maestro_->setPosition(SUCTION_LEFT, 1020);
    // maestro_->setPosition(LEFT_ARM, 1980);
    
 }
/* void ServoHandler::openValve() {

    maestro_->setPosition(VALVE, 0);
}


void ServoHandler::closeValve() {

    maestro_->setPosition(VALVE, 7000);
    maestro_->setPosition(VALVE, 7000);
} */

void ServoHandler::electroMagnetOn() {
    maestro_->setPosition(MAGNET, 1900);
}


void ServoHandler::electroMagnetOff() {
    maestro_->setPosition(MAGNET, 1500);
}


void ServoHandler::figurineArmLow() {
    maestro_->setPosition(STATUE, 900);
}


void ServoHandler::figurineArmMiddle() {
    maestro_->setPosition(STATUE, 1180);
}


void ServoHandler::figurineArmHigh() {
    maestro_->setPosition(STATUE, 1850);
}


void ServoHandler::figurineArmSpeedLow() {
    maestro_->setSpeed(STATUE, 500);
}


void ServoHandler::figurineArmSpeedHigh() {
    maestro_->setSpeed(STATUE, 1900);
}


/* void ServoHandler::openTube(int tubeNumber)
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
} */


void ServoHandler::shutdownServos()
{
	for(int i = 0; i < 18; i++)
	{
		maestro_->setPosition(i, 0);
		usleep(80);
	}
}

void ServoHandler::turnOnPump()
{
    #ifndef SIMULATION
    RPi_writeGPIO(PUMP_PWM, HIGH);
    #endif
    isPumpOn_ = true;
}

void ServoHandler::turnOffPump()
{
    #ifndef SIMULATION
    RPi_writeGPIO(PUMP_PWM, LOW);
    #endif
    isPumpOn_ = false;
}


void ServoHandler::moveSuction(bool high, bool moveMiddle)
{
    if(high)
    {
        maestro_->setPosition(SERVO_SUCTION[0], 1800);
        if (moveMiddle)
            maestro_->setPosition(SERVO_SUCTION[1], 1650);
        maestro_->setPosition(SERVO_SUCTION[2], 1800);
    }
    else
    {
        maestro_->setPosition(SERVO_SUCTION[0], 1100);
        if (moveMiddle)
            maestro_->setPosition(SERVO_SUCTION[1], 1100);
        maestro_->setPosition(SERVO_SUCTION[2], 1100);
    }
}



void ServoHandler::moveMiddle()
{
        maestro_->setPosition(SERVO_SUCTION[0], 1700);
            maestro_->setPosition(SERVO_SUCTION[1], 1700);
        maestro_->setPosition(SERVO_SUCTION[2], 1700);

}


void ServoHandler::moveSuctionForGoldDrop()
{
    maestro_->setPosition(SERVO_SUCTION[0], 1995);
    maestro_->setPosition(SERVO_SUCTION[2], 1995);
}

void ServoHandler::moveMiddleSuctionForDrop(bool drop)
{
    maestro_->setPosition(SERVO_SUCTION[0], 2200);
    if (drop)
        maestro_->setPosition(SERVO_SUCTION[1], 1725);
    else
        maestro_->setPosition(SERVO_SUCTION[1], 1800);
    maestro_->setPosition(SERVO_SUCTION[2], 2200);
}


void ServoHandler::moveRail(int velocity)
{
    maestro_->setPosition(ELEVATOR, velocity);
}


void ServoHandler::foldArms()
{
    maestro_->setPosition(5, 850);
    maestro_->setPosition(4, 2000);
}


void ServoHandler::unfoldArms(bool isPlayingRightSide)
{
    if (!isPlayingRightSide)
        maestro_->setPosition(5, 1585);
    else
        maestro_->setPosition(4, 1300);
}

void ServoHandler::raiseArms(bool isPlayingRightSide)
{
    if (!isPlayingRightSide)
        maestro_->setPosition(5, 1900);
    else
        maestro_->setPosition(4, 1000);
}

void ServoHandler::moveArmForDrop(bool isPlayingRightSide)
{
    if (!isPlayingRightSide)
        maestro_->setPosition(5, 1715);
    else
        maestro_->setPosition(4, 1170);
}
