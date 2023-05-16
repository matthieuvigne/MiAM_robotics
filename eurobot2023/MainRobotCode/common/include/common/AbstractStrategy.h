#ifndef ABSTRACT_STRATEGY_H
#define ABSTRACT_STRATEGY_H

#include "common/RobotInterface.h"

class AbstractStrategy
{
    public:
        // Constructor
        AbstractStrategy() {}

        /// @brief Called before the start of the match, to setup the robot.
        /// @details This function can return false to be called again, and must
        ///          return true when done.
        virtual bool setup(RobotInterface *robot) = 0;

        // Code executed when shutting down the robot
        virtual void shutdown() = 0;

        // The actual match code, which runs in its own thread.
        virtual void match() = 0;

        /// @brief Function called at every timestep of the low-level loop.
        /// @details This function is meant to implement NON-BLOCKING
        ///          actions at the low-level loop period. The typical usage
        ///          is servo control.
        virtual void periodicAction() = 0;

        // network::CameraClient camera_;
        std::vector<pthread_t> createdThreads_;

        RobotInterface *robot;
        MotionController *motionController;

        bool go_to_straight_line(RobotPosition targetPosition,
          double factor = 1.0, bool backward = false);
        bool turn_around_point(double angle_rad, double factor = 1.0);
        bool go_forward(double distance);
        bool go_to_rounded_corner(std::vector<RobotPosition> targetPositions, bool backwards = false);

        /// @brief  Perform a square in the clockwise direction - used to calibrate the robot
        /// @param squareDimenstion Square size, in mm.
        void testSquare(bool clockwise = false, double const& squareDimenstion = 500);
};

#endif
