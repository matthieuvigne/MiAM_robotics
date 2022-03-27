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
    turnOffPump();

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

void ServoHandler::reglageouvrirlebrasdroithaut() {
	maestro_->setPosition(RIGHT_ARM, 700);
	//reglage mesure bras droit haut maestro_->setPosition(RIGHT_ARM, 700);
}

void ServoHandler::reglageouvrirlebrasdroitmilieu() {
	maestro_->setPosition(RIGHT_ARM, 1500);
	//reglage mesure bras droit mesure fouille avant action doigt maestro_->setPosition(RIGHT_ARM, 1500);
}
void ServoHandler::reglageouvrirlebrasdroitbas() {
	maestro_->setPosition(RIGHT_ARM, 1630);
	// reglage mesure bras droit bas maestro_->setPosition(RIGHT_ARM, 1640);
}

void ServoHandler::reglageouvrirlebrasgauchebas() {
	maestro_->setPosition(LEFT_ARM, 1200);
}
void ServoHandler::reglageouvrirlebrasgauchemilieu() {
	maestro_->setPosition(LEFT_ARM, 1500);
}
void ServoHandler::reglageouvrirlebrasgauchehaut() {
	maestro_->setPosition(LEFT_ARM, 2060);
}

void ServoHandler::reglageouvrirledoigtdroithaut() {
	maestro_->setPosition(RIGHT_FINGER, 1500);

}

void ServoHandler::reglageouvrirledoigtdroitbas() {
	maestro_->setPosition(RIGHT_FINGER, 1700);
	// reglage mesure doigt droit bas maestro_->setPosition(RIGHT_ARM, 1640);
}

void ServoHandler::reglageouvrirledoigtgauchehaut() {
	maestro_->setPosition(LEFT_FINGER, 1500);
}

void ServoHandler::reglageouvrirledoigtgauchebas() {
	maestro_->setPosition(LEFT_FINGER, 1500);
}

void ServoHandler::ouvrirlebrasmilieu(bool isPlayingRightSide, bool right)
{
	if (!isPlayingRightSide)
    {
		if (right)
            maestro_->setPosition(RIGHT_ARM, 1500);
     	else
            maestro_->setPosition(LEFT_ARM, 1500);
    }
    else
    {
        if (right)
            maestro_->setPosition(LEFT_ARM, 1500);
        else
            maestro_->setPosition(RIGHT_ARM, 1500);
    }
}

void ServoHandler::ouvrirlebrasbas(bool isPlayingRightSide, bool right)
{
	if (!isPlayingRightSide)
    {
		if (right)
            maestro_->setPosition(RIGHT_ARM, 1980);
     	else
            maestro_->setPosition(LEFT_ARM, 1980);
    }
    else
    {
        if (right)
            maestro_->setPosition(LEFT_ARM, 1980);
        else
            maestro_->setPosition(RIGHT_ARM, 1980);
    }
}

void ServoHandler::ouvrirlebrashaut(bool isPlayingRightSide, bool right)
{
	if (!isPlayingRightSide)
    {
		if (right)
            maestro_->setPosition(RIGHT_ARM, 1020);
     	else
            maestro_->setPosition(LEFT_ARM, 1020);
    }
    else
    {
        if (right)
            maestro_->setPosition(LEFT_ARM, 1020);
        else
            maestro_->setPosition(RIGHT_ARM, 1020);
    }
}

void ServoHandler::ouvrirledoigthaut(bool isPlayingRightSide, bool right)
{
	if (!isPlayingRightSide)
    {
		if (right)
            maestro_->setPosition(RIGHT_FINGER, 1980);
     	else
            maestro_->setPosition(LEFT_FINGER, 1980);
    }
    else
    {
        if (right)
            maestro_->setPosition(LEFT_FINGER, 1980);
        else
            maestro_->setPosition(RIGHT_FINGER, 1980);
    }
}

void ServoHandler::ouvrirledoigtbas(bool isPlayingRightSide, bool right)
{
    if (!isPlayingRightSide)
    {
        if (right)
            maestro_->setPosition(RIGHT_FINGER, 1020);
     	else
            maestro_->setPosition(LEFT_FINGER, 1020);
    }
    else
    {
        if (right)
            maestro_->setPosition(LEFT_FINGER, 1020);
        else
            maestro_->setPosition(RIGHT_FINGER, 1020);
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


void ServoHandler::moveSuction(bool high)
{
    if(high)
    {
        maestro_->setPosition(SERVO_SUCTION[0], 1550);
        maestro_->setPosition(SERVO_SUCTION[1], 1600);
        maestro_->setPosition(SERVO_SUCTION[2], 1150);
    }
    else
    {
        maestro_->setPosition(SERVO_SUCTION[0], 550);
        maestro_->setPosition(SERVO_SUCTION[1], 650);
        maestro_->setPosition(SERVO_SUCTION[2], 2150);
    }
}


void ServoHandler::initsectionmiddle()
{
        maestro_->setPosition(SERVO_SUCTION[1], 2500);
        //pour ventouse haute milieu 2500
}

void ServoHandler::transportfigurine()
{
        maestro_->setPosition(SERVO_SUCTION[1], 2300);
        usleep(0.5e6);
        maestro_->setPosition(SERVO_SUCTION[1], 2100);
        usleep(0.5e6);
        maestro_->setPosition(SERVO_SUCTION[1], 1900);
        usleep(0.5e6);
        maestro_->setPosition(SERVO_SUCTION[1], 1750);
        usleep(0.5e6);
        maestro_->setPosition(SERVO_SUCTION[1], 1600);
        //pour ventouse haute milieu 2500
}

void ServoHandler::moveRail(int velocity)
{
    maestro_->setPosition(ELEVATOR, velocity);
}

////////////////////////////////////////////////////////////////
// Statue-related functions
////////////////////////////////////////////////////////////////
void ServoHandler::electroMagnetOn()
{
    maestro_->setPosition(MAGNET, 7000);
}


void ServoHandler::electroMagnetOff()
{
    maestro_->setPosition(MAGNET, 0);
}



void ServoHandler::figurineArmTransport()
{
    maestro_->setPosition(STATUE, 1950);
}


void ServoHandler::figurineArmCatch()
{
    maestro_->setPosition(STATUE, 1750);
}


void ServoHandler::figurineArmLow()
{
    maestro_->setPosition(STATUE, 900);
}


void ServoHandler::figurineArmSpeedLow()
{
    maestro_->setSpeed(STATUE, 500);
}


void ServoHandler::figurineArmSpeedHigh()
{
    maestro_->setSpeed(STATUE, 1900);
}

