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
#define RAIL_SERVO_ID 30
#define MIAM_RAIL_TOLERANCE 100 // in counts
#define RAIL_DOWN_VALUE -44000 // in counts


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
        targetPosition.x = 725;
        targetPosition.y = 170;
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
            currentRailMeasurements.currentPosition_ = 0;
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

Action* Strategy::chooseNextAction(
    std::vector<Action>& actions,
    RobotPosition currentPosition,
    MotionPlanner& motionPlanner
)
{

    Action* bestAction = &actions.front();
    double bestLoss = 0;

    int i = 0;

    for (auto & a : actions)
    {

        if (a.isActivated_)
        {
            // compute loss
            double timeItTakesToGoThere = motionPlanner.computeMotionTime(robot->getParameters().getTrajConf(), currentPosition, a.startPosition_);
            double currentLoss = a.nPointsToGain_ /(a.timeItTakes_ + timeItTakesToGoThere);

            std::cout << "Calcul pour " << i << " : " << timeItTakesToGoThere << ", " << currentLoss << std::endl;

            if (bestLoss < currentLoss)
            {
                bestAction = &a;
                bestLoss = currentLoss;
                std::cout << "Chosen action " << i << std::endl;
            }
        }
        i++;
    }

    return bestAction;
}

void Strategy::match_impl()
{
    // Create required variables.
    RobotPosition targetPosition;
    TrajectoryVector traj;
    RobotPosition endPosition;
    std::vector<RobotPosition> positions;

    // Send start signal
    // std::cout << sock_.send("Match started") << std::endl;

    // create brain
    MotionPlanner* motionPlanner = motionController->motionPlanner_;

    double const robot_chassis_front = robot->getParameters().CHASSIS_FRONT;
    double const robot_chassis_back = robot->getParameters().CHASSIS_BACK;
    double const robot_half_chassis_width = robot->getParameters().CHASSIS_WIDTH/2.0;

    RobotPosition const left_of_cherry_distributor_bottom(700,200,0);
    RobotPosition const center_of_cherry_distributor_left(15,1500,0);
    RobotPosition const center_of_cherry_distributor_right(2000 - 15,1500,0);
    RobotPosition const center_of_cherry_distributor_bottom(1000,170,0);
    RobotPosition const center_of_cherry_distributor_top(1000,2830,0);

    double const distributor_width = 30;



#if ENABLE_DYNAMIC_ACTION_CHOOSING

    Action action1(100,1, RobotPosition(2000, 1700, 0));
    // Action action2(500, 1, RobotPosition(2200, 500, 0));
    // Action action3(15, 1, RobotPosition(1500, 200, 0));

    std::vector<Action> actionVector;
    actionVector.push_back(action1);
    // actionVector.push_back(action2);
    // actionVector.push_back(action3);



    while (!actionVector.empty())
    {

        std::cout << "Remaining actions : " << actionVector.size() << std::endl;
        for (auto v : actionVector)
        {
            std::cout << v << std::endl;
        }

        Action* nextAction = chooseNextAction(actionVector, motionController->getCurrentPosition(), motionPlanner);


        std::cout << "nextAction : " << *nextAction << std::endl;

        targetPosition = motionController->getCurrentPosition();
        traj = motionPlanner->computeTraj(robot->getParameters().getTrajConf(), targetPosition, nextAction->startPosition_);
        motionController->setTrajectoryToFollow(traj);
        bool moveSuccess = motionController->waitForTrajectoryFinished();

        if (!moveSuccess)
        {
            // attempt alternative route
            // assume object size is ~20 cm

            std::cout << "attempting alternative route" << std::endl;

            // attempt left
            RobotPosition left_point(motionController->getCurrentPosition());
            // 20 cm to the left
            left_point.x = left_point.x - 400 * sin(left_point.theta);
            left_point.y = left_point.y + 400 * cos(left_point.theta);

            if(left_point.x < table_dimensions::table_max_x and left_point.x > table_dimensions::table_min_x
                and left_point.y < table_dimensions::table_max_y and left_point.y > table_dimensions::table_min_y )
            {


                std::cout << "trying to go left" << std::endl;
                std::cout << "current position : " << motionController->getCurrentPosition() <<  std::endl;
                std::cout << "waypoint : " << left_point <<  std::endl;

                // // attempt following the trajectory
                // targetPosition = motionController->getCurrentPosition();
                // traj = computeTrajectoryStraightLineToPoint(targetPosition, left_point);
                // motionController->setTrajectoryToFollow(traj);

                targetPosition = motionController->getCurrentPosition();
                positions.clear();
                positions.push_back(targetPosition);
                positions.push_back(left_point);
                positions.push_back(nextAction->startPosition_);
                traj = computeTrajectoryRoundedCorner(robot->getParameters().getTrajConf(), positions, 200.0, 0.3);
                motionController->setTrajectoryToFollow(traj);

                if (motionController->waitForTrajectoryFinished())
                {
                    std::cout << "waypoint reached :" << motionController->getCurrentPosition() <<  std::endl;

                    // targetPosition = motionController->getCurrentPosition();
                    // traj = motionPlanner->computeTraj(targetPosition, nextAction->startPosition_);
                    // motionController->setTrajectoryToFollow(traj);
                    // moveSuccess = motionController->waitForTrajectoryFinished();
                } else
                {
                    std::cout << "waypoint NOT reached :" << motionController->getCurrentPosition() <<  std::endl;

                }


            }

            if (!moveSuccess)
            {
                // attempt alternative route
                // assume object size is ~20 cm

                std::cout << "attempting alternative route" << std::endl;

                // attempt right
                RobotPosition right_point(motionController->getCurrentPosition());
                // 20 cm to the right
                right_point.x = right_point.x + 400 * sin(right_point.theta);
                right_point.y = right_point.y - 400 * cos(right_point.theta);

                if(right_point.x < table_dimensions::table_max_x and right_point.x > table_dimensions::table_min_x
                    and right_point.y < table_dimensions::table_max_y and right_point.y > table_dimensions::table_min_y )
                {


                    std::cout << "trying to go right" << std::endl;
                    std::cout << "current position : " << motionController->getCurrentPosition() <<  std::endl;
                    std::cout << "waypoint : " << right_point <<  std::endl;

                    // // attempt following the trajectory
                    // targetPosition = motionController->getCurrentPosition();
                    // traj = computeTrajectoryStraightLineToPoint(targetPosition, right_point);
                    // motionController->setTrajectoryToFollow(traj);

                    targetPosition = motionController->getCurrentPosition();
                    positions.clear();
                    positions.push_back(targetPosition);
                    positions.push_back(right_point);
                    positions.push_back(nextAction->startPosition_);
                    traj = computeTrajectoryRoundedCorner(robot->getParameters().getTrajConf(), positions, 200.0, 0.3);
                    motionController->setTrajectoryToFollow(traj);

                    if (motionController->waitForTrajectoryFinished())
                    {
                        std::cout << "waypoint reached :" << motionController->getCurrentPosition() <<  std::endl;

                        // targetPosition = motionController->getCurrentPosition();
                        // traj = motionPlanner->computeTraj(targetPosition, nextAction->startPosition_);
                        // motionController->setTrajectoryToFollow(traj);
                        // moveSuccess = motionController->waitForTrajectoryFinished();
                    } else
                    {
                        std::cout << "waypoint NOT reached :" << motionController->getCurrentPosition() <<  std::endl;

                    }


                }
            }

        }


        if (moveSuccess)
        {
            nextAction->performAction(robot);
            actionVector.erase(std::find(actionVector.begin(), actionVector.end(), *nextAction));

            // If action successful, reactivate all actions
            for (auto & a : actionVector)
                nextAction->isActivated_ = true;
        }
        else
        {
            // penaliser l'action
            nextAction->isActivated_ = false;
        }



    }
#else

    // // grab cheries in front of robot
    // go_to_straight_line(left_of_cherry_distributor_bottom);

    // // set rail height and tilt
    // set_reservoir_tilt(ReservoirTilt::UP);
    // moveRail(rail::CHERRY_DISTRIBUTOR);
    // set_brush_move(BrushDirection::TOWARDS_BACK);

    // // go front
    // go_to_straight_line(motionController->getCurrentPosition() + RobotPosition(100, 0, 0));
    // // wait
    // robot->wait(3);
    // // go back
    // go_to_straight_line(motionController->getCurrentPosition() + RobotPosition(-100, 0, 0), true); // backwards


    // code de sophie

    // Option 1 -> Bottom, right & top cherries
    // -------------------------------

    std::vector<RobotPosition> targetPositions;

    // // Get the  bottom cheeries
    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    targetPositions.push_back(center_of_cherry_distributor_bottom - RobotPosition(robot_chassis_front  + 40 + distributor_width / 2,0,0));
    go_to_rounded_corner(targetPositions);

    grab_cherries();

    // Go to the top cherries, grab them
    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    targetPositions.push_back(RobotPosition(500, 3000 - 600,0));
    targetPositions.push_back(RobotPosition(300, 3000 - robot_chassis_front - 300,0));
    targetPositions.push_back(RobotPosition(300, 3000 - robot_chassis_front - 20,0));
    // targetPositions.push_back(RobotPosition(500, 3000 - 600,0));
    // targetPositions.push_back(center_of_cherry_distributor_top -RobotPosition(robot_chassis_front  + 300 + distributor_width, 150,0));
    // targetPositions.push_back(center_of_cherry_distributor_top -RobotPosition(robot_chassis_front  + 200 + distributor_width, 50,0));
    // targetPositions.push_back(center_of_cherry_distributor_top -RobotPosition(robot_chassis_front  + 40 + distributor_width, 0,0));
    go_to_rounded_corner(targetPositions);


    // grab_cherries();


    // RobotPosition startPos = center_of_cherry_distributor_top -RobotPosition(robot_chassis_front  + 40 + distributor_width, 0,0);
    // motionController->resetPosition(startPos, true, true, true);

    // Put the cherries in the basket

    // targetPositions.clear();
    // RobotPosition position = motionController->getCurrentPosition();
    // targetPositions.push_back(position);
    // position.y -= 200;
    // targetPositions.push_back(position);
    // position.x = 300;
    // targetPositions.push_back(position);
    // position.y += 100;
    // targetPositions.push_back(position);
    // go_to_rounded_corner(targetPositions);

    put_cherries_in_the_basket();

    while (true) ;;

    // Go to get right cherries then get the top cherries
    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    targetPositions.push_back(center_of_cherry_distributor_right - RobotPosition(robot_chassis_front + 700,0,0));
    targetPositions.push_back(center_of_cherry_distributor_right - RobotPosition(robot_chassis_front + 151 + distributor_width / 2,0,0));
    targetPositions.push_back(center_of_cherry_distributor_right - RobotPosition(robot_chassis_front + 150 + distributor_width / 2,0,0));
    go_to_rounded_corner(targetPositions);

    grab_cherries();


    // avoid the line
    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    targetPositions.push_back(RobotPosition(500, 3000-600,0));
    targetPositions.push_back(center_of_cherry_distributor_top -RobotPosition(robot_chassis_front  + 300 + distributor_width, 150,0));
    targetPositions.push_back(center_of_cherry_distributor_top -RobotPosition(robot_chassis_front  + 100 + distributor_width, 50,0));
    targetPositions.push_back(center_of_cherry_distributor_top -RobotPosition(robot_chassis_front  + 40 + distributor_width, 0,0));
    go_to_rounded_corner(targetPositions, true);
    grab_cherries();


    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    targetPositions.push_back(center_of_cherry_distributor_top -RobotPosition(robot_chassis_front + 151 + distributor_width / 2, 0,0));
    targetPositions.push_back(center_of_cherry_distributor_top -RobotPosition(robot_chassis_front + 150 + distributor_width / 2, 0,0));
    go_to_rounded_corner(targetPositions);

    grab_cherries();

    // Go to put the cherries in the basket
    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    targetPositions.push_back(RobotPosition(225,3000 - robot_chassis_front - 151, 0));
    targetPositions.push_back(RobotPosition(225,3000 - robot_chassis_front - 150,M_PI_2));
    go_to_rounded_corner(targetPositions);

    put_cherries_in_the_basket();

    // Go to the final zone
    go_to_straight_line(RobotPosition(725,200,0), true);
    // go_to_straight_line(RobotPosition(300,1300,0), true);



#endif

    std::cout << "Strategy thread ended" << robot->getMatchTime() << std::endl;
}

void Strategy::match()
{

    std::cout << "Strategy thread started." << std::endl;

    std::thread stratMain(&Strategy::match_impl, this);
    pthread_t handle = stratMain.native_handle();
    createdThreads_.push_back(handle);
    stratMain.detach();

    double const FALLBACK_TIME = 95.0;
    robot->wait(FALLBACK_TIME);
    if (!MATCH_COMPLETED)
        pthread_cancel(handle);
    createdThreads_.clear();
    robot->wait(0.05);
    // usleep(50000);
    if (!MATCH_COMPLETED)
    {
        std::cout << "Match almost done, auto-triggering fallback strategy" << std::endl;
    }
}
}
