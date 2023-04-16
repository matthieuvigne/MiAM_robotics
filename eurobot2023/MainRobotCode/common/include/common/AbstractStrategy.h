#ifndef ABSTRACT_STRATEGY_H
#define ABSTRACT_STRATEGY_H

#include <network/camera_client.hpp>
#include "common/RobotInterface.h"

class AbstractStrategy
{
    public:
        // Constructor
        AbstractStrategy() {}

        // Called before the start of the match, to setup the robot.
        virtual void setup(RobotInterface *robot) = 0;

        // Code executed when shutting down the robot
        virtual void shutdown() = 0;

        // The actual match code, which runs in its own thread.
        virtual void match() = 0;

        network::CameraClient camera_;
        std::vector<pthread_t> createdThreads_;

        RobotInterface *robot;
        MotionController *motionController;

        bool go_to_straight_line(RobotPosition targetPosition, bool backward = false);
        bool go_forward(double distance);
        bool go_to_rounded_corner(std::vector<RobotPosition> targetPositions, bool backwards = false);
};

#endif