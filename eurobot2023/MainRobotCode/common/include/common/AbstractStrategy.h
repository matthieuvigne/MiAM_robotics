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

        // The actual match code, which runs in its own thread.
        virtual void match() = 0;

        network::CameraClient camera_;
        std::vector<pthread_t> createdThreads_;
};

#endif