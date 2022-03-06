/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <miam_utils/raspberry_pi/RPiGPIO.h>
#include "ServoHandler.h"
#include <unistd.h>

// Servo config: port defintion.
int const SERVO_SUCTION[3] = {0, 1, 2};    // Numbered from right to left.

int const SERVO_TAP = 12;

// Temporary.
/// \brief Servo position definition.
//~ int const SP_SUCTION_HIGH = {1500, 1500, 1500};
//~ int const SP_SUCTION_LOW = {1500, 1500, 1500};

//~ int const SP_TUBE_OPEN = {1500, 1500, 1500};
//~ int const SP_TUBE_CLOSE = {1500, 1500, 1500};

int const SP_TAP_OPEN  = 1000;
int const SP_TAP_CLOSE  = 1000;

int const PUMP_PWM = 26;

int const VALVE_RIGHT = 0;
int const VALVE_CENTER = 1;
int const VALVE_LEFT = 2;


int const ELECTROMAGNET_NUMBER = 4;
int const FIGURINE_ARM_NUMBER = 14;

typedef enum SERVOS{
    VENTOUSE_DROITE = 6,
    VENTOUSE_MILIEU = 7,
    VENTOUSE_GAUCHE = 8 ,
    //VALVE = 3,
    SERVO_VERTICAL_TRANSLATION = 9
};

//int const SERVO_TUBE[3] = {VENTOUSE_DROITE, VENTOUSE_MILIEU, VENTOUSE_GAUCHE};

int const RIGHT_ARM = 10;
int const LEFT_ARM = 12;

int const RIGHT_FINGER = 11;
int const LEFT_FINGER = 13;

ServoHandler::ServoHandler(MaestroDriver *maestro):
    maestro_(maestro)
{
}


void ServoHandler::ouvrirlesbrasdebugmilieu() {

    // maestro_->setPosition(RIGHT_ARM, 1500);
    maestro_->setPosition(VENTOUSE_DROITE, 1500);
    maestro_->setPosition(RIGHT_FINGER, 1500);
    // maestro_->setPosition(VENTOUSE_MILIEU, 1500);
    // maestro_->setPosition(VENTOUSE_GAUCHE, 1500);
    // maestro_->setPosition(LEFT_ARM, 1500);
}


void ServoHandler::ouvrirlesbrasdebugbas() {

    // maestro_->setPosition(RIGHT_ARM, 1980);
    maestro_->setPosition(VENTOUSE_DROITE, 1980);
    maestro_->setPosition(RIGHT_FINGER, 1980);
    // maestro_->setPosition(VENTOUSE_MILIEU, 1980);
    // maestro_->setPosition(VENTOUSE_GAUCHE, 1980);
    // maestro_->setPosition(LEFT_ARM, 1020);
}


void ServoHandler::ouvrirlesbrasdebughaut() {

    // maestro_->setPosition(RIGHT_ARM, 1020);
    maestro_->setPosition(VENTOUSE_DROITE, 1020);
    maestro_->setPosition(RIGHT_FINGER, 1020);
    // maestro_->setPosition(VENTOUSE_MILIEU, 1020);
    // maestro_->setPosition(VENTOUSE_GAUCHE, 1020);
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
    maestro_->setPosition(ELECTROMAGNET_NUMBER, 1900);
}


void ServoHandler::electroMagnetOff() {
    maestro_->setPosition(ELECTROMAGNET_NUMBER, 1500);
}


void ServoHandler::figurineArmLow() {
    maestro_->setPosition(FIGURINE_ARM_NUMBER, 900);
}


void ServoHandler::figurineArmMiddle() {
    maestro_->setPosition(FIGURINE_ARM_NUMBER, 1180);
}


void ServoHandler::figurineArmHigh() {
    maestro_->setPosition(FIGURINE_ARM_NUMBER, 1850);
}


void ServoHandler::figurineArmSpeedLow() {
    maestro_->setSpeed(FIGURINE_ARM_NUMBER, 500);
}


void ServoHandler::figurineArmSpeedHigh() {
    maestro_->setSpeed(FIGURINE_ARM_NUMBER, 1900);
}


bool ServoHandler::init(std::string const& portName)
{
    // Enable pump GPIO.
    RPi_setupGPIO(PUMP_PWM, PI_GPIO_OUTPUT);
    turnOffPump();

    return maestro_->init(portName);
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


void ServoHandler::tapOpen()
{
    maestro_->setPosition(SERVO_TAP, 1500);
}


void ServoHandler::tapClose()
{
    maestro_->setPosition(SERVO_TAP, 750);
}

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
    RPi_writeGPIO(PUMP_PWM, HIGH);
}

void ServoHandler::turnOffPump()
{
    RPi_writeGPIO(PUMP_PWM, LOW);
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
    maestro_->setPosition(SERVO_VERTICAL_TRANSLATION, velocity);
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