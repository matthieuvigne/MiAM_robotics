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

#define RAIL_DOWN_VALUE -23400 // in counts

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
        servo->setTargetPosition(RESERVOIR_SERVO, 2450);
    }
    else if (reservoirTilt == ReservoirTilt::HORIZONTAL)
    {
        servo->setTargetPosition(RESERVOIR_SERVO, 2048);
    }
    else if (reservoirTilt == ReservoirTilt::GRAB)
    {
        servo->setTargetPosition(RESERVOIR_SERVO, 1825);
    }
    else if (reservoirTilt == ReservoirTilt::UP)
    {
        servo->setTargetPosition(RESERVOIR_SERVO, 1580);
    }
}

void Strategy::grab_cherries()
{

    set_reservoir_tilt(ReservoirTilt::GRAB);
    moveRail(rail::CHERRY_GRAB);
    // Wait for rail to start moving.
    robot->wait(0.1);
    waitForRail();
    set_brush_move(BrushDirection::TOWARDS_BACK);

    // Move forward - slowly
    RobotPosition targetPosition = motionController->getCurrentPosition();
    miam::trajectory::TrajectoryConfig conf = motionController->robotParams_.getTrajConf();
    conf.maxWheelVelocity *= 0.4;
    conf.maxWheelAcceleration *= 0.4;
    TrajectoryVector traj = miam::trajectory::computeTrajectoryStraightLine(conf, targetPosition, 60);
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();
    // Wait a bit more
    robot->wait(0.5);
    for (int i = 0; i < 2; i++)
    {
        set_brush_move(BrushDirection::TOWARDS_FRONT);
        robot->wait(0.6);
        set_brush_move(BrushDirection::TOWARDS_BACK);
        robot->wait(0.6);
    }
    set_brush_move(BrushDirection::TOWARDS_FRONT);
    // Stop and move back
    set_reservoir_tilt(ReservoirTilt::UP);
    robot->wait(0.1);
    set_brush_move(BrushDirection::OFF);
    moveRail(rail::NOMINAL);

    traj = miam::trajectory::computeTrajectoryStraightLine(motionController->robotParams_.getTrajConf(), targetPosition, -120);
    motionController->setTrajectoryToFollow(traj);
    motionController->waitForTrajectoryFinished();
}

void Strategy::put_cherries_in_the_basket()
{
    // put rail in the right height
    set_reservoir_tilt(ReservoirTilt::HORIZONTAL);
    moveRail(rail::TOP);
    robot->wait(0.1);
    waitForRail();

    // go front
    // very far
    go_forward(190);

    // tilt and push cherries
    set_reservoir_tilt(ReservoirTilt::DOWN);
    for (int i = 0; i < 2; i++)
    {
        set_brush_move(BrushDirection::TOWARDS_FRONT);
        robot->wait(0.6);
        set_brush_move(BrushDirection::TOWARDS_BACK);
        robot->wait(0.6);
    }
    set_reservoir_tilt(ReservoirTilt::HORIZONTAL);
    set_brush_move(BrushDirection::OFF);

    go_forward(-130);
    moveRail(rail::MIDDLE);
    robot->wait(2.0);
    set_reservoir_tilt(ReservoirTilt::UP);
    waitForRail();
}


void Strategy::waitForRail()
{
    while (railState_ != rail::state::IDLE)
        usleep(500);
}

void Strategy::moveRail(double const& targetPosition)
{
    targetRailValue_ = static_cast<int>((1 - std::max(0.0, targetPosition)) * RAIL_DOWN_VALUE);

    if (currentRailMeasurements.currentPosition_ > targetRailValue_)
    {
        servo->setTargetVelocity(RAIL_SERVO_ID, -4095);
        railState_ = rail::state::GOING_DOWN;
    }
    else
    {
        servo->setTargetVelocity(RAIL_SERVO_ID, 4095);
        railState_ = rail::state::GOING_UP;
    }
}

void Strategy::calibrateRail()
{
    servo->setMode(RAIL_SERVO_ID, STS::Mode::VELOCITY);
    railState_ = rail::state::CALIBRATING;
}

void Strategy::updateRailHeight()
{
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
    motionController->log("railPosition", currentRailMeasurements.currentPosition_);
}

}
