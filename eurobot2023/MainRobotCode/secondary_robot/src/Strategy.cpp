/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <unistd.h>
#include <math.h>
#include <thread>

#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>
#include <miam_utils/raspberry_pi/RaspberryPi.h>

#include "secondary_robot/Strategy.h"

#include <common/MotionPlanner.h>

using namespace miam::trajectory;
using miam::RobotPosition;


#define USE_CAMERA 1

#define ENABLE_DYNAMIC_ACTION_CHOOSING 0 // use the dynamic action choosing feature

// #define SKIP_TO_GRABBING_SAMPLES 1
// #define SKIP_TO_PUSHING_SAMPLES 1
// #define SKIP_TO_GRABBING_SAMPLES_SIDE_DIST 1

// This function is responsible for trying to bring the robot back to base,
// at the end of the match.
bool MATCH_COMPLETED = false;

int const BRUSH_MOTOR = 12;
int const BRUSH_DIR = 16;


// Rail
int const RAIL_SWITCH = 21;

namespace secondary_robot {

Strategy::Strategy()
{

}


enum setupPhase
{
    FIRST_CALL,
    CALIBRATING_RAIL,
    MOVING_RAIL
};

bool Strategy::setup(RobotInterface *robot)
{
    static setupPhase phase = setupPhase::FIRST_CALL;
#ifdef SIMULATION
    // Always perform setup in simulation
    phase = setupPhase::FIRST_CALL;
#endif

    if (phase == setupPhase::FIRST_CALL)
    {
        phase = setupPhase::CALIBRATING_RAIL;
        this->robot = robot;
        this->servo = robot->getServos();
        this->motionController = robot->getMotionController();

        RPi_setupGPIO(RAIL_SWITCH, PI_GPIO_INPUT_PULLUP);

        // Start all servos in position mode, except for the rail.
        servo->setMode(0xFE, STS::Mode::POSITION);
        servo->setMode(RAIL_SERVO_ID, STS::Mode::VELOCITY);

        // Set initial position
        RobotPosition targetPosition;
        targetPosition.x = 715;
        targetPosition.y = 165;
        targetPosition.theta = 0;
        motionController->resetPosition(targetPosition, true, true, true);
        motionController->setAvoidanceMode(AvoidanceMode::AVOIDANCE_MPC);

        RPi_setupGPIO(BRUSH_MOTOR, PiGPIOMode::PI_GPIO_OUTPUT);
        RPi_writeGPIO(BRUSH_MOTOR, false);
        RPi_setupGPIO(BRUSH_DIR, PiGPIOMode::PI_GPIO_OUTPUT);
        RPi_writeGPIO(BRUSH_DIR, false);

        // connect socket
        int attempts = 0;
        while(attempts < 20)
        {
            try
            {
                std::cout << "Trying to connect" << std::endl;
                // args: addr, port, isUDP
                // address is broadcast address ; UDP = true
                sock_.connect("192.168.6.255", 37020, true);
                std::cout << "Connected!" << std::endl;
                usleep(50000);
                break;
            }
            catch(network::SocketException const&)
            {
                usleep(1e6);
                std::cout << "Failed to connect..." << std::endl;
            }
            attempts++;
        }

        calibrateRail();
    }
    else if (phase == setupPhase::CALIBRATING_RAIL)
    {
        if (railState_ == rail::IDLE)
        {
            moveRail(rail::NOMINAL);
            phase = setupPhase::MOVING_RAIL;
        }
    }
    else if (phase == setupPhase::MOVING_RAIL)
    {
        return railState_ == rail::IDLE;
    }
    return false;
}

void Strategy::periodicAction()
{
#ifdef SIMULATION
    railState_ = rail::state::IDLE;
    return;
#endif
    // Perform calibration if needed.
    if (railState_ == rail::state::CALIBRATING)
    {
        if (RPi_readGPIO(RAIL_SWITCH) == 0)
        {
            servo->setTargetVelocity(RAIL_SERVO_ID, 0);
            // Offset so that 1.0 corresponds to the top
            currentRailMeasurements.currentPosition_ = -500;
            currentRailMeasurements.lastEncoderMeasurement_ = servo->getCurrentPosition(RAIL_SERVO_ID);
            railState_ = rail::state::IDLE;
        }
        else
            servo->setTargetVelocity(RAIL_SERVO_ID, 4095);
    }
    else
    {
        updateRailHeight();
        if ((railState_ == rail::state::GOING_UP && currentRailMeasurements.currentPosition_ > targetRailValue_) ||
            (railState_ == rail::state::GOING_DOWN && currentRailMeasurements.currentPosition_ < targetRailValue_))
        {
            servo->setTargetVelocity(RAIL_SERVO_ID, 0);
            railState_ = rail::state::IDLE;
        }
    }
}

void Strategy::shutdown()
{
    RPi_writeGPIO(BRUSH_MOTOR, false);
}

void Strategy::match_impl()
{
    // Create required variables.
    RobotPosition targetPosition;
    TrajectoryVector traj;
    std::vector<RobotPosition> targetPositions;

    // create brain
    MotionPlanner* motionPlanner = motionController->motionPlanner_;

    RobotParameters const robotParameters = robot->getParameters();

    // Points of interest: points to go to grab the cherries (before the final move action)
    double const distributor_width = 30;
    double const cherryDistributorOffset = robotParameters.CHASSIS_FRONT  + 60 + distributor_width / 2;

    RobotPosition const cherryDistributorBottom(1000 - cherryDistributorOffset, 165, 0);
    RobotPosition const cherryDistributorTop(1000 - cherryDistributorOffset, 3000 - 165, 0);
    RobotPosition const cherryDistributorLeft(cherryDistributorOffset, 1500, 0);
    RobotPosition const cherryDistributorRight(2000 - cherryDistributorOffset, 1500, 0);


    // Start bottom, grab bottom and left cherries.

    // // Get the  bottom cheeries
    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    targetPositions.push_back(cherryDistributorBottom);
    go_to_rounded_corner(targetPositions);

    grab_cherries();

    // Go to the left cherries, grab them
    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    targetPositions.push_back(cherryDistributorLeft + RobotPosition(300, 0, 0));
    targetPositions.push_back(cherryDistributorLeft+ RobotPosition(20, 0, 0));
    go_to_rounded_corner(targetPositions);
    grab_cherries();

    // Put the cherries in the basket
    targetPositions.clear();
    RobotPosition position = motionController->getCurrentPosition();
    targetPositions.push_back(position);
    position.x = robotParameters.CHASSIS_WIDTH + 40.0;
    position.y = 2500;
    targetPositions.push_back(position);
    position.y = 3000 - robotParameters.CHASSIS_FRONT - 60;
    targetPositions.push_back(position);
    go_to_rounded_corner(targetPositions);

    put_cherries_in_the_basket();

    // Match end: go back to base
    goBackToBase();

    while (true) ;;

    std::cout << "Strategy thread ended" << robot->getMatchTime() << std::endl;
}

void Strategy::match()
{

    std::cout << "Strategy thread started." << std::endl;

    std::thread stratMain(&Strategy::match_impl, this);
    pthread_t handle = stratMain.native_handle();
    createdThreads_.push_back(handle);
    stratMain.detach();

    double const FALLBACK_TIME = 90.0;
    robot->wait(FALLBACK_TIME);
    if (!MATCH_COMPLETED)
        pthread_cancel(handle);
    createdThreads_.clear();
    robot->wait(0.05);
    if (!MATCH_COMPLETED)
    {
        std::cout << "Match almost done, auto-triggering fallback strategy" << std::endl;
        goBackToBase();
    }
}

void Strategy::goBackToBase()
{
    TrajectoryVector traj;
    RobotPosition endPosition;
    std::vector<RobotPosition> positions;

    RobotPosition position = motionController->getCurrentPosition();
    positions.push_back(position);
    position.x = 600;
    position.y = 400;
    positions.push_back(position);
    go_to_rounded_corner(positions);
}
}
