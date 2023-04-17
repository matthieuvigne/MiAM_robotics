/// \file Strategy.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef SECONDARY_ROBOT_STRATEGY_H
#define SECONDARY_ROBOT_STRATEGY_H

#include "common/RobotInterface.h"
#include "common/ServoHandler.h"
#include "common/AbstractAction.h"
#include "common/AbstractStrategy.h"
#include "common/MotionPlanning.h"

namespace secondary_robot
{

    enum ReservoirTilt
    {
        UP,
        DOWN
    };

    enum BrushDirection
    {
        OFF,
        TOWARDS_FRONT,
        TOWARDS_BACK
    };

    enum RailHeight // from 0 to 1000
    {
        BOTTOM = 0,
        TOP = 1000,
        CHERRY_DISTRIBUTOR = 200,
        CHERRY_BASKET = 800,
        MIDDLE = 500
    };

    class Strategy : public AbstractStrategy
    {
    public:
        // Constructor
        Strategy();

        // Called before the start of the match, to setup the robot.
        void setup(RobotInterface *robot);

        // Code executed when shutting down the robot
        void shutdown() override;

        // The actual match code, which runs in its own thread.
        void match();

    private:
        void match_impl(); /// Actual implementation of the match code.

        STSServoDriver *servo;

        Action *chooseNextAction(
            std::vector<Action> &actions,
            RobotPosition currentPosition,
            MotionPlanning motionPlanner);

        miam::PID PIDRail_;

        /// @brief Blocking function for moving the rail to a specified height
        /// @param railHeight the height of the rail, between 0 and 1000
        void move_rail(RailHeight railHeight);

        /// @brief Set the brush to move either direction or switch it off
        /// @param brushDirection the desired movement or off
        void set_brush_move(BrushDirection brushDirection);

        /// @brief Sets the reservoir to be tilted up or down
        /// @param reservoirTilt up or down
        void set_reservoir_tilt(ReservoirTilt reservoirTilt);

        /// @brief Execute the sequence to grab cherries: set rail position, brush and reservoir tilt, go forward 150mm, wait, then back 150mm
        void grab_cherries();

        /// @brief Execute the sequence to put cherries in the basket: set rail position, go forward 150mm, tilt and start motor, then de-tilt, stop and go backwards
        void put_cherries_in_the_basket();
    };
}

#endif
