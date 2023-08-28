/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <unistd.h>
#include <math.h>
#include <thread>

#include <miam_utils/raspberry_pi/RaspberryPi.h>

#include "secondary_robot/Strategy.h"
#include "secondary_robot/CherryActions.h"


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

void Strategy::grab_cherries(RobotPosition startPosition)
{
    textlog << "[CherryActions] grab_cherries" << std::endl;
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

    RobotPosition distributorPosition = startPosition;
    distributorPosition.x += abs(startPosition.theta) > M_PI_2 ? -60 : 60;
    TrajectoryVector traj = miam::trajectory::computeTrajectoryStraightLineToPoint(conf, targetPosition, distributorPosition);

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
    textlog << "[CherryActions] put_cherries_in_the_basket" << std::endl;
    // put rail in the right height
    set_reservoir_tilt(ReservoirTilt::HORIZONTAL);
    moveRail(rail::TOP);
    robot->wait(0.1);
    waitForRail();

    // go front
    // very far
    // go_forward(230);


    RobotPosition position = motionController->getCurrentPosition();
    position.x = 180;
    if (!robot->isPlayingRightSide())
    {
        // left side: a little more to the left
        position.x = 150;
    }
    position.theta = M_PI_2;
    position.y = 3000 - 50;
    go_to_straight_line(position);

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
    robot->wait(0.1);
    set_reservoir_tilt(ReservoirTilt::UP);
    // waitForRail();

    putCherriesIntoBasket_ = true;
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
    int currentCount = 0;

    for (int i = 0; i < 5; i++)
    {
        // Retry up to 5 times.
        currentCount = servo->getCurrentPosition(RAIL_SERVO_ID);
        if (currentCount != 0)
            break;
    }
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


bool CherryAction::performAction()
{
    textlog << "[CherryAction] performAction" << std::endl;
    // RobotPosition currentPosition;
    secondary_robot::Strategy* strategy_secondary(
        dynamic_cast<secondary_robot::Strategy*>(strategy));
    textlog << "[CherryAction] casted ptr" << std::endl;

    // // refine the position
    // if ((strategy->motionController->getCurrentPosition() - end_position).norm() > 20)
    // {
    //     strategy_secondary->go_to_straight_line(end_position);
    // }
    // strategy_secondary->turn_around_point(end_position.theta);

    textlog << "[CherryAction] grab_cherries" << std::endl;
    RobotPosition start_position_with_offset = start_position;
    start_position_with_offset.x += abs(start_position_with_offset.theta) > M_PI_2 ? -50 : 50;
    strategy_secondary->grab_cherries(start_position_with_offset);

    // put in the basket
    strategy_secondary->moveRail(secondary_robot::rail::ANTICIPATING_TOP);
    TrajectoryVector traj;

    RobotPosition position = strategy_secondary->motionController->getCurrentPosition();
    position.x = 180;
    position.y = 3000 - 151 - 160;
    if (!strategy_secondary->robot->isPlayingRightSide())
    {
        // left side: a little more to the left
        position.x = 150;
    }
    position.theta = M_PI_2;

    textlog << "[CherryAction] planning path" << std::endl;
    traj = strategy_secondary->robot->getMotionController()->computeMPCTrajectory(
        position,
        strategy_secondary->robot->getMotionController()->getDetectedObstacles(), true);

    if (traj.empty())
    {
        return false;
    }

    strategy_secondary->robot->getMotionController()->setTrajectoryToFollow(traj);

    textlog << "[CherryAction] go to basket" << std::endl;
    if (strategy_secondary->robot->getMotionController()->waitForTrajectoryFinished())
    {
        textlog << "[CherryAction] put_cherries_in_the_basket" << std::endl;
        strategy_secondary->put_cherries_in_the_basket();
        return true;
    }
    return false;
}


bool PutCherriesInTheBasket::performAction()
{
    textlog << "[PutCherriesInTheBasket] performAction" << std::endl;
    // RobotPosition currentPosition;
    secondary_robot::Strategy* strategy_secondary(
        dynamic_cast<secondary_robot::Strategy*>(strategy));
    textlog << "[PutCherriesInTheBasket] casted ptr" << std::endl;

    textlog << "[CherryAction] put_cherries_in_the_basket" << std::endl;

    strategy_secondary->put_cherries_in_the_basket();
    return true;
}