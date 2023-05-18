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
#include <miam_utils/TextLogger.h>

#include "secondary_robot/Strategy.h"

#include <common/MotionPlanner.h>

using namespace miam::trajectory;
using miam::RobotPosition;



// This function is responsible for trying to bring the robot back to base,
// at the end of the match.
bool MATCH_COMPLETED = false;

int const BRUSH_MOTOR = 12;
int const BRUSH_DIR = 16;

// Rail
int const RAIL_SWITCH = 21;

namespace secondary_robot {

RobotPosition const START_POSITION(715, 165, 0);
RobotPosition const ALTERNATIVE_START_POSITION(715-500, 3000-165, 0);

double getTime()
{
    struct timespec currentTime;
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    return currentTime.tv_sec + static_cast<double>(currentTime.tv_nsec  / 1.0e9);
}


Strategy::Strategy()
{

}


enum setupPhase
{
    FIRST_CALL,
    CALIBRATING_RAIL,
    MOVING_RAIL,
    WAITING,
};

bool Strategy::setup(RobotInterface *robot)
{
    static setupPhase phase = setupPhase::FIRST_CALL;
    static double waitStart = 0;

    countedPointsForGoingBackToBase_ = false;

    // TODO can i be changed during setup with screen?
    startingTop_ = false;

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
        if (startingTop_)
        {
            motionController->resetPosition(ALTERNATIVE_START_POSITION, true, true, true);
        }
        else
        {
            motionController->resetPosition(START_POSITION, true, true, true);
        }
        motionController->setAvoidanceMode(AvoidanceMode::AVOIDANCE_OFF);

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
                textlog << "[Strategy (secondary_robot)] " << "Trying to connect" << std::endl;
                // args: addr, port, isUDP
                // address is broadcast address ; UDP = true
                sock_.connect("192.168.6.255", 37020, true);
                textlog << "[Strategy (secondary_robot)] " << "Connected!" << std::endl;
                usleep(50000);
                break;
            }
            catch(network::SocketException const&)
            {
                usleep(1e6);
                textlog << "[Strategy (secondary_robot)] " << "Failed to connect..." << std::endl;
            }
            attempts++;
        }
        set_reservoir_tilt(ReservoirTilt::UP);
        calibrateRail();
    }
    else if (phase == setupPhase::CALIBRATING_RAIL)
    {
        if (railState_ == rail::IDLE)
        {
            waitStart = getTime();
            phase = setupPhase::WAITING;
        }
    }
    else if (phase == setupPhase::WAITING)
    {
        if (getTime() - waitStart > 0.3)
        {
            phase = setupPhase::MOVING_RAIL;
            moveRail(rail::NOMINAL);
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
        static int phase = 0;
        // Hit first time, going fast
        if (phase == 0)
        {
            if (RPi_readGPIO(RAIL_SWITCH) == 0)
            {
                servo->setTargetVelocity(RAIL_SERVO_ID, -4096);
                phase = 1;
            }
            else
                servo->setTargetVelocity(RAIL_SERVO_ID, 4095);
        }
        else if (phase == 1)
        {
            // Hit slowly
            if (RPi_readGPIO(RAIL_SWITCH) == 1)
            {
                servo->setTargetVelocity(RAIL_SERVO_ID, 800);
                phase = 2;
            }
        }
        else
        {
            if (RPi_readGPIO(RAIL_SWITCH) == 0)
            {
                servo->setTargetVelocity(RAIL_SERVO_ID, 0);
                currentRailMeasurements.currentPosition_ = 0;
                currentRailMeasurements.lastEncoderMeasurement_ = servo->getCurrentPosition(RAIL_SERVO_ID);
                railState_ = rail::state::IDLE;
            }
        }
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
        else if (railState_ != rail::state::IDLE)
        {
            int targetVelocity = static_cast<int>(1.5 * (targetRailValue_ - currentRailMeasurements.currentPosition_));
            if (targetVelocity > 4096)
                targetVelocity = 4096;
            if (targetVelocity < -4096)
                targetVelocity = -4096;
            if (targetVelocity > 0 && targetVelocity < 700)
                targetVelocity = 700;
            if (targetVelocity < 0 && targetVelocity > -500)
                targetVelocity = -500;
            // Special case: top has a mechanical bound, wait until we touch it.
            if (currentRailMeasurements.currentPosition_ > 1500 && targetVelocity > 700)
                targetVelocity = 700;
            int speed = servo->getCurrentSpeed(RAIL_SERVO_ID);
            if (targetRailValue_ >  0 && currentRailMeasurements.currentPosition_ > 0 && speed > 0 && speed < 500)
            {
                servo->setTargetVelocity(RAIL_SERVO_ID, 0);
                railState_ = rail::state::IDLE;
                // Reset position to bound
                currentRailMeasurements.currentPosition_ = 2600;
            }
            else
            {
                servo->setTargetVelocity(RAIL_SERVO_ID, targetVelocity);
            }
        }
    }
}

void Strategy::shutdown()
{
    RPi_writeGPIO(BRUSH_MOTOR, false);
}



void Strategy::startSequenceTop()
{
    RobotParameters const robotParameters = robot->getParameters();
    // Points of interest: points to go to grab the cherries (before the final move action)
    double const distributor_width = 30;
    double const cherryDistributorOffset = robotParameters.CHASSIS_FRONT  + 60 + distributor_width / 2;
    RobotPosition const cherryDistributorBottom(1000 - cherryDistributorOffset, 165, 0);
    RobotPosition const cherryDistributorTop(1000 - cherryDistributorOffset, 3000 - 165, 0);
    RobotPosition const cherryDistributorLeft(cherryDistributorOffset, 1500, 0);
    RobotPosition const cherryDistributorRight(2000 - cherryDistributorOffset, 1500, 0);

    RobotPosition targetPosition;
    TrajectoryVector traj;
    std::vector<RobotPosition> targetPositions;

    // Get the top cheeries
    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    targetPositions.push_back(cherryDistributorTop);
    go_to_rounded_corner(targetPositions);

    if ((motionController->getCurrentPosition() - cherryDistributorTop).norm() < 100)
    {
        grab_cherries();
    }

    // Put the cherries in the basket
    moveRail(rail::TOP);

    // cible
    RobotPosition position = motionController->getCurrentPosition();
    position.x = 180; //robotParameters.CHASSIS_WIDTH + 90.0;
    position.y = 3000 - robotParameters.CHASSIS_FRONT - 160;
    if (!robot->isPlayingRightSide())
    {
        // left side: a little more to the left
        position.x = 150;
    }
    position.theta = M_PI_2;


    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    position.y = 3000 - robotParameters.CHASSIS_FRONT - 170;
    targetPositions.push_back(position);
    position.y = 3000 - robotParameters.CHASSIS_FRONT - 160;
    targetPositions.push_back(position);
    go_to_rounded_corner(targetPositions);


    if ((motionController->getCurrentPosition() - position).norm() < 100)
    {
        put_cherries_in_the_basket();
        // Estimate: 18 cherries in basket
        robot->updateScore(18);
    }


    // Faire une action : aller de 3 vers 1
    PushingCakesAction action3to1 = PushCakes3to1();

    if (performSecondaryRobotAction(&action3to1))
        textlog << "[Strategy (secondary robot)] startSequenceTop: PushCakes3to1 succeeded" << std::endl;
    

    

}

void Strategy::startSequenceBottom()
{    
    RobotParameters const robotParameters = robot->getParameters();
    // Points of interest: points to go to grab the cherries (before the final move action)
    double const distributor_width = 30;
    double const cherryDistributorOffset = robotParameters.CHASSIS_FRONT  + 60 + distributor_width / 2;
    RobotPosition const cherryDistributorBottom(1000 - cherryDistributorOffset, 165, 0);
    RobotPosition const cherryDistributorTop(1000 - cherryDistributorOffset, 3000 - 165, 0);
    RobotPosition const cherryDistributorLeft(cherryDistributorOffset, 1500, 0);
    RobotPosition const cherryDistributorRight(2000 - cherryDistributorOffset, 1500, 0);

    RobotPosition targetPosition;
    TrajectoryVector traj;
    std::vector<RobotPosition> targetPositions;

    // Get the  bottom cheeries
    targetPositions.clear();
    targetPositions.push_back(motionController->getCurrentPosition());
    targetPositions.push_back(cherryDistributorBottom);
    go_to_rounded_corner(targetPositions);

    if ((motionController->getCurrentPosition() - cherryDistributorBottom).norm() < 100)
    {
        grab_cherries();
    }

    // // Go to the left cherries, grab them
    // targetPositions.clear();
    // targetPositions.push_back(motionController->getCurrentPosition());
    // targetPositions.push_back(cherryDistributorLeft + RobotPosition(300, 0, 0));
    // targetPositions.push_back(cherryDistributorLeft);
    // go_to_rounded_corner(targetPositions);

    // if ((motionController->getCurrentPosition() - cherryDistributorLeft).norm() < 100)
    // {
    //     grab_cherries();
    // }

    // Put the cherries in the basket

    moveRail(rail::TOP);

    // targetPositions.clear();
    // RobotPosition position = motionController->getCurrentPosition();
    // targetPositions.push_back(position);
    // position.x = robotParameters.CHASSIS_WIDTH + 90.0;
    // position.y = 2500;
    // targetPositions.push_back(position);
    // position.y = 3000 - robotParameters.CHASSIS_FRONT - 60;
    // targetPositions.push_back(position);
    // go_to_rounded_corner(targetPositions);


    // cible
    RobotPosition position = motionController->getCurrentPosition();
    position.x = 180; //robotParameters.CHASSIS_WIDTH + 90.0;
    position.y = 3000 - robotParameters.CHASSIS_FRONT - 160;
    if (!robot->isPlayingRightSide())
    {
        // left side: a little more to the left
        position.x = 150;
    }
    position.theta = M_PI_2;

    // Ajouter un obstacle fictif au niveau des gateaux
    // x 230, y 2300, radius 200
    RobotPosition obstacle;
    obstacle.x = 230;
    obstacle.y = 2300;
    robot->getMotionController()->addPersistentObstacle(std::make_tuple(obstacle, 300));

    traj = robot->getMotionController()->computeMPCTrajectory(position, robot->getMotionController()->getDetectedObstacles());
    robot->getMotionController()->setTrajectoryToFollow(traj);
    robot->getMotionController()->waitForTrajectoryFinished();

    // retirer l'obstacle
    robot->getMotionController()->popBackPersistentObstacles();


    if ((motionController->getCurrentPosition() - position).norm() < 100)
    {
        put_cherries_in_the_basket();
        // Estimate: 18 cherries in basket
        robot->updateScore(18);
    }



}



void Strategy::match_impl()
{
    // Send start message
    if (sock_.send("Match started"))
        textlog << "[Strategy (secondary_robot)] " << "Start message sent" << std::endl;
    else
        textlog << "[Strategy (secondary_robot)] " << "Start message error! not sent" << std::endl;

    robot->updateScore(5);


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

    // Create required variables.
    RobotPosition targetPosition;
    TrajectoryVector traj;
    std::vector<RobotPosition> targetPositions;

    if (startingTop_)
    {
        motionController->resetPosition(ALTERNATIVE_START_POSITION, true, true, true);
        startSequenceTop();
    }
    else
    {
        motionController->resetPosition(START_POSITION, true, true, true);
        startSequenceBottom();
    }


    // go_forward(-50);
    // traj.clear();
    // traj.push_back(std::shared_ptr<Trajectory>(new PointTurn(robot->getParameters().getTrajConf(), robot->getMotionController()->getCurrentPosition(), -M_PI_2)));
    // robot->getMotionController()->setTrajectoryToFollow(traj);
    // robot->getMotionController()->waitForTrajectoryFinished();

    // Roam around the terrain
    // moveRail(rail::CHERRY_GRAB);
    // robot->wait(0.1);
    // waitForRail();
    // set_reservoir_tilt(ReservoirTilt::DOWN);

    std::vector<std::shared_ptr<SecondaryRobotAction > > actions;

    // if starting top the cakes have already been taken
    if (!startingTop_)
    {
        std::shared_ptr<SecondaryRobotAction > pushCakes1to5(new PushCakes1to5());
        actions.push_back(pushCakes1to5);
    }

    std::shared_ptr<SecondaryRobotAction > pushCakes7to5(new PushCakes7to5());
    actions.push_back(pushCakes7to5);
    std::shared_ptr<SecondaryRobotAction > pushCakes3to4(new PushCakes3to4());
    actions.push_back(pushCakes3to4);
    std::shared_ptr<SecondaryRobotAction > pushCakes6to4(new PushCakes6to4());
    actions.push_back(pushCakes6to4);
    std::shared_ptr<SecondaryRobotAction > pushCakes7to5ButOnlyPartial(new PushCakes7to5ButOnlyPartial());
    actions.push_back(pushCakes7to5ButOnlyPartial);
    std::shared_ptr<SecondaryRobotAction > pushCakes6to5(new PushCakes6to5());
    actions.push_back(pushCakes6to5);
    std::shared_ptr<SecondaryRobotAction > actionCherryLeft(new GrabCherriesLeft());
    actions.push_back(actionCherryLeft);
    // std::shared_ptr<SecondaryRobotAction > actionCherryRight(new GrabCherriesRight());
    // actions.push_back(actionCherryRight);
    

    int number_of_unsuccessful_iters = 0;

    while (actions.size() > 0)
    {
        
        textlog << "[Strategy (secondary_robot)] " << "Left distributor was " << (motionController->wasLeftDistributorVisited() ? "!" : "NOT") << " visited" << std::endl;
        textlog << "[Strategy (secondary_robot)] " << "Right distributor was " << (motionController->wasRightDistributorVisited() ? "!" : "NOT") << " visited" << std::endl;

        // which action is :
        // * so that no robot is close
        // * and the closer to the current position

        int action_index = -1;
        double minDistanceToStartPoint = 10000;
        RobotPosition currentPosition = robot->getMotionController()->getCurrentPosition();
        for (int i = 0; i < actions.size(); i++)
        {

            SecondaryRobotAction* action = actions.at(i).get();

            if (!action->activated)
            {
                textlog << "[Strategy (secondary_robot)] " << "action " << i << " was deactivated" << std::endl;
                continue;
            }

            double minDistanceFromObstacle = 10000;
            double minDistanceFromObstacleEnd = 10000;
            double distanceToStartPoint = (action->start_position - currentPosition).norm();

            for (auto obstacle : robot->getMotionController()->getDetectedObstacles(false))
            {

                double tmpMin = (std::get<0>(obstacle) - action->start_position).norm() - std::get<1>(obstacle);
                double tmpMinEnd = (std::get<0>(obstacle) - action->end_position).norm() - std::get<1>(obstacle);

                // distance to center of obstacle minus size of the obstacle
                minDistanceFromObstacle = std::min(
                    minDistanceFromObstacle,
                    tmpMin);
                // distance to center of obstacle minus size of the obstacle
                minDistanceFromObstacleEnd = std::min(
                    minDistanceFromObstacleEnd,
                    tmpMinEnd);
            }

            // 150 = radius of the robot
            if (minDistanceFromObstacle > 150 & minDistanceFromObstacleEnd > 70 &
                distanceToStartPoint < minDistanceToStartPoint)
            {
                action_index = i;
                minDistanceToStartPoint = distanceToStartPoint;
            }


            textlog << "[Strategy (secondary_robot)] " << "action " << i << " start point " << action->start_position << std::endl;
            textlog << "[Strategy (secondary_robot)] " << "   minDistanceFromObstacle " << minDistanceFromObstacle << " minDistanceFromObstacleEnd " << minDistanceFromObstacleEnd << " distanceToStartPoint " << distanceToStartPoint << std::endl;



        }

        textlog << "[Strategy (secondary_robot)] " << "Chosen action: " << action_index << std::endl;
        if (action_index < 0)
        {
            textlog << "[Strategy (secondary_robot)] " << "Reactivating all actions" << std::endl;
            for (int i = 0; i < actions.size() ; i++)
            {
                actions.at(i)->activated = true;
            }
        }

        robot->wait(1);

        if (action_index >= 0)
        {
            number_of_unsuccessful_iters = 0;

            textlog << "[Strategy (secondary_robot)] " << "####### Performing pushing action: " << action_index << " (" << actions.size() << " remaining)" << std::endl;

            SecondaryRobotAction* action = actions.at(action_index).get();

            if (performSecondaryRobotAction(action))
            {
                textlog << "[Strategy (secondary_robot)] " << "Action was successful: removed" << std::endl;
                actions.erase( actions.begin() + action_index);
            }
            else
            {
                textlog << "[Strategy (secondary_robot)] " << "Action was not successful: deactivated" << std::endl;
                action->activated = false;
            }
        }
        else
        {
            number_of_unsuccessful_iters++;
        }

        if (number_of_unsuccessful_iters > 10)
        {
            textlog << "[Strategy (secondary_robot)] " << "Removing all actions" << std::endl;
            actions.clear();
        }
    }

    // Match end: go back to base
    goBackToBase();
    MATCH_COMPLETED = true;

    while (true) ;;

    textlog << "[Strategy (secondary_robot)] " << "Strategy thread ended" << robot->getMatchTime() << std::endl;
}

void Strategy::match()
{

    textlog << "[Strategy (secondary_robot)] " << "Strategy thread started." << std::endl;

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
        textlog << "[Strategy (secondary_robot)] " << "Match almost done, auto-triggering fallback strategy" << std::endl;
        goBackToBase();
    }
    else
    {
        textlog << "[Strategy (secondary_robot)] " << "Match almost done, strategy already complete" << std::endl;
    }
}

bool Strategy::checkIfBackToBase()
{
    // if the robot is already in the end zone, just flush trajectories
    RobotPosition currentPosition = motionController->getCurrentPosition();
    if (currentPosition.x > 440.0 && currentPosition.x < 1000.0 && currentPosition.y < 480.0)
    {
        textlog << "[Strategy (secondary robot)] checkIfBackToBase: robot is in zone, flushing all trajs" << std::endl;
        TrajectoryVector emptyTraj;
        std::shared_ptr<Trajectory > pt(new PointTurn(robot->getParameters().getTrajConf(), currentPosition, currentPosition.theta));
        emptyTraj.push_back(pt);
        motionController->setTrajectoryToFollow(emptyTraj);

        if (!countedPointsForGoingBackToBase_)
        {
            textlog << "[Strategy (secondary robot)] checkIfBackToBase: counted points for going back to base" << std::endl;
            countedPointsForGoingBackToBase_ = true;
            robot->updateScore(15);    
        }

        return true;
    }
    textlog << "[Strategy (secondary robot)] checkIfBackToBase: robot not yet in zone " << std::endl;
    return false;
}

void Strategy::goBackToBase()
{
    // count points and stop
    if (checkIfBackToBase())
    {
        return;
    }

    // set a low avoidance zone for end
    RobotPosition lowAvoidanceZonePosition;
    lowAvoidanceZonePosition.x = 750;
    lowAvoidanceZonePosition.y = 230;
    robot->getMotionController()->setLowAvoidanceZone(lowAvoidanceZonePosition, 800);

    TrajectoryVector traj;
    RobotPosition endPosition;
    std::vector<RobotPosition> positions;

    RobotPosition position = motionController->getCurrentPosition();
    position.x = 680;
    position.y = 500;
    position.theta = -M_PI_2;

    std::vector<Obstacle > detectedObstacles = robot->getMotionController()->getDetectedObstacles();

    std::cout << "See if obstacles to remove" << std::endl;
    // remove obstacles which are too close to the position (since we know
    // there is an obstacle)
    bool detectedAnObstacleToRemove = false;
    bool wasThereAnObstacleInEndZone = false;

    do
    {
        detectedAnObstacleToRemove = false;
        for (int i = 0; i < detectedObstacles.size(); i++)
        {
            Obstacle obstacle = detectedObstacles.at(i);
            if ((std::get<0>(obstacle) - position).norm() < 500 )
            {
                std::cout << "Removing an obstacle" << std::endl;
                detectedObstacles.erase( detectedObstacles.begin() + i);
                detectedAnObstacleToRemove = true;
                wasThereAnObstacleInEndZone = true;
                break;
            }
        }
    } while (detectedAnObstacleToRemove);

    std::cout << "detectedObstacles.size() " << detectedObstacles.size() << std::endl;

    if (wasThereAnObstacleInEndZone)
        std::cout << "There was an obstacle in end zone" << std::endl;

    double angleBeforeFinalStraightLine = (wasThereAnObstacleInEndZone ? M_PI_2 : -M_PI_2);
    position.theta = angleBeforeFinalStraightLine;

    if ((motionController->getCurrentPosition() - position).norm() > 100)
    {
        traj = robot->getMotionController()->computeMPCTrajectory(
            position,
            detectedObstacles);

        if (traj.getDuration() > 0)
        {
            robot->getMotionController()->setTrajectoryToFollow(traj);
            robot->getMotionController()->waitForTrajectoryFinished();
        }
        else
        {
            go_forward(-100);
            go_to_straight_line(position);
        }
    }
    else
    {
        go_to_straight_line(position);
    }

    position.x = 680;
    position.y = 100;
    position.theta = angleBeforeFinalStraightLine;

    TrajectoryConfig trajconf = robot->getParameters().getTrajConf();
    trajconf.maxWheelVelocity = 200;

    textlog << "[Strategy (secondary robot)] goBackToBase: Performing a final straight line" << std::endl;

    traj = miam::trajectory::computeTrajectoryStraightLineToPoint(
        trajconf,
        motionController->getCurrentPosition(), // start
        position, // end
        0.0, // no velocity at end point
        wasThereAnObstacleInEndZone // backward if there is an obstacle
    );

    for (auto subtraj : traj)
    {
        subtraj->setAvoidanceEnabled(false);
    }

    robot->getMotionController()->setTrajectoryToFollow(traj);
    robot->getMotionController()->waitForTrajectoryFinished();

    // count points and stop
    checkIfBackToBase();
    
}

bool Strategy::performSecondaryRobotAction(SecondaryRobotAction* action)
{
    TrajectoryVector traj;

    // go to start taking obstacles into account
    for (auto obstacle : action->obstacles_on_the_road)
    {
        robot->getMotionController()->addPersistentObstacle(obstacle);
    }

    traj = robot->getMotionController()->computeMPCTrajectory(action->start_position, robot->getMotionController()->getDetectedObstacles());

    // remove obstacles on the road
    for (auto obstacle : action->obstacles_on_the_road)
    {
        robot->getMotionController()->popBackPersistentObstacles();
    }

    if (traj.getDuration() == 0.0)
    {
        textlog << "[Strategy (secondary_robot)] " << "Motion planning to action failed!" << std::endl;
        return false;
    }
    robot->getMotionController()->setTrajectoryToFollow(traj);

    // go to start point and perform action
    if (robot->getMotionController()->waitForTrajectoryFinished() && 
            action->performAction(this))
    {
        // action was successful

        // update score
        int score = action->getScore();
        textlog << "[Strategy (secondary_robot)] " << "update score: " << score << std::endl;
        robot->updateScore(score);

        // add obstacle in the end
        for (auto obstacle : action->obstacles_in_the_end)
        {
            robot->getMotionController()->addPersistentObstacle(obstacle);
        }
        
        return true;
    }
    else
    // action was not successful
    {
        return false;
    }
}

}
