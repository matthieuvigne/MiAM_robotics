/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <unistd.h>
#include <math.h>
#include <thread>

#include <miam_utils/raspberry_pi/RaspberryPi.h>

#include "secondary_robot/Strategy.h"


using namespace miam::trajectory;
using miam::RobotPosition;


int const BRUSH_MOTOR = 12;
int const BRUSH_DIR = 16;
int const RESERVOIR_SERVO = 5;


// Rail
int const RAIL_SWITCH = 21;
#define RAIL_SERVO_ID 30
#define MIAM_RAIL_TOLERANCE 100 // in counts
#define RAIL_DOWN_VALUE -44000 // in counts


namespace secondary_robot {


void Strategy::set_brush_move(BrushDirection brushDirection)
{
    if (brushDirection == BrushDirection::OFF)
    {
        RPi_writeGPIO(BRUSH_MOTOR, false);
    }
    else if (brushDirection == BrushDirection::TOWARDS_BACK)
    {
        RPi_writeGPIO(BRUSH_MOTOR, true);
        RPi_writeGPIO(BRUSH_DIR, true);
    }
    else if (brushDirection == BrushDirection::TOWARDS_FRONT)
    {
        RPi_writeGPIO(BRUSH_MOTOR, true);
        RPi_writeGPIO(BRUSH_DIR, false);
    }
}

void Strategy::set_reservoir_tilt(ReservoirTilt reservoirTilt)
{
    if (reservoirTilt == ReservoirTilt::DOWN)
    {
        servo->setTargetPosition(RESERVOIR_SERVO, 2500);
    }
    else if (reservoirTilt == ReservoirTilt::HORIZONTAL)
    {
        servo->setTargetPosition(RESERVOIR_SERVO, 2100);
    }
    else if (reservoirTilt == ReservoirTilt::GRAB)
    {
        servo->setTargetPosition(RESERVOIR_SERVO, 1820);
    }
    else if (reservoirTilt == ReservoirTilt::UP)
    {
        servo->setTargetPosition(RESERVOIR_SERVO, 1630);
    }
}

void Strategy::grab_cherries()
{

    set_reservoir_tilt(ReservoirTilt::GRAB);
    set_brush_move(BrushDirection::TOWARDS_BACK);
    moveRail(rail::CHERRY_GRAB);

    // Move forward - slowly
    RobotPosition targetPosition = motionController->getCurrentPosition();
    miam::trajectory::TrajectoryConfig conf = motionController->robotParams_.getTrajConf();
    conf.maxWheelVelocity *= 0.2;
    TrajectoryVector traj = miam::trajectory::computeTrajectoryStraightLine(conf, targetPosition, 60);
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();
    // Wait a bit more
    robot->wait(0.5);
    // Stop and move back
    set_reservoir_tilt(ReservoirTilt::UP);
    robot->wait(0.1);
    set_brush_move(BrushDirection::OFF);
    moveRail(rail::IDLE);

    traj = miam::trajectory::computeTrajectoryStraightLine(motionController->robotParams_.getTrajConf(), targetPosition, -60);
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();
}

void Strategy::put_cherries_in_the_basket()
{
    // put rail in the right height
    set_reservoir_tilt(ReservoirTilt::HORIZONTAL);
    moveRail(rail::TOP);

    // go front
    // go_to_straight_line(motionController->getCurrentPosition() + RobotPosition(150, 0, 0));
    go_forward(100);

    // tilt and push cherries
    set_reservoir_tilt(ReservoirTilt::DOWN);
    for (int i = 0; i < 3; i++)
    {
        set_brush_move(BrushDirection::TOWARDS_FRONT);
        robot->wait(0.75);
        set_brush_move(BrushDirection::TOWARDS_BACK);
        robot->wait(0.75);
    }
    set_reservoir_tilt(ReservoirTilt::HORIZONTAL);
    set_brush_move(BrushDirection::OFF);

    go_forward(-100);
    moveRail(rail::IDLE);
}

void Strategy::moveRail(double const& targetPosition)
{
    int targetValue = static_cast<int>((1 - std::min(1.0, std::max(0.0, targetPosition))) * RAIL_DOWN_VALUE);

    if (currentRailMeasurements.currentPosition_ > targetValue)
        servo->setTargetVelocity(RAIL_SERVO_ID, -4095);
    else
        servo->setTargetVelocity(RAIL_SERVO_ID, 4095);

    int nIter = 0;
    while (std::abs(currentRailMeasurements.currentPosition_ - targetValue) > MIAM_RAIL_TOLERANCE && nIter < 12000)
    {
        updateRailHeight();
        usleep(20000);
        nIter++;
    }
    servo->setTargetVelocity(RAIL_SERVO_ID, 0);
}



void Strategy::calibrateRail()
{
    servo->setMode(RAIL_SERVO_ID, STS::Mode::VELOCITY);
    usleep(2000);
    // the switch is up
    servo->setTargetVelocity(RAIL_SERVO_ID, 4095);
    while (RPi_readGPIO(RAIL_SWITCH) == 1)
    {
        servo->setTargetVelocity(RAIL_SERVO_ID, 4095);
        usleep(20000);
    }
    servo->setTargetVelocity(RAIL_SERVO_ID, 0);
    usleep(2000);

    // Init
    currentRailMeasurements.currentPosition_ = 0;
    currentRailMeasurements.lastEncoderMeasurement_ = servo->getCurrentPosition(RAIL_SERVO_ID);
}


void Strategy::updateRailHeight()
{
    usleep(1000);
    int currentCount = servo->getCurrentPosition(RAIL_SERVO_ID);
    while (currentCount == 0)
        currentCount = servo->getCurrentPosition(RAIL_SERVO_ID);
    int delta = currentCount - currentRailMeasurements.lastEncoderMeasurement_;
    currentRailMeasurements.lastEncoderMeasurement_ = currentCount;
    while (delta > 2048)
        delta -= 4096;
    while (delta < -2048)
        delta += 4096;
    currentRailMeasurements.currentPosition_ += delta;
}

}
