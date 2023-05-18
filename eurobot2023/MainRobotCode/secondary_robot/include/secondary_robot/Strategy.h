/// \file Strategy.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef SECONDARY_ROBOT_STRATEGY_H
#define SECONDARY_ROBOT_STRATEGY_H

#include "common/RobotInterface.h"
#include "common/AbstractAction.h"
#include "common/AbstractStrategy.h"
#include "common/MotionPlanner.h"
#include "miam_utils/network/client_socket.hpp"
#include "miam_utils/network/socket_exception.hpp"
#include "secondary_robot/PushingCakesAction.h"
#include "secondary_robot/CherryActions.h"

namespace secondary_robot
{

    enum ReservoirTilt
    {
        UP,
        GRAB,
        HORIZONTAL,
        DOWN
    };

    enum BrushDirection
    {
        OFF,
        TOWARDS_FRONT,
        TOWARDS_BACK
    };

    namespace rail
    {
        double const TOP = 1.20;
        // double const TOP = 1.14;
        double const NOMINAL = 0.15;
        double const CHERRY_GRAB = 0.0;
        double const MIDDLE = 0.5;
        enum state
        {
            CALIBRATING,
            IDLE,
            GOING_UP,
            GOING_DOWN
        };
    }
    #define RAIL_SERVO_ID 30

    struct RailMeasurements
    {
        int lastEncoderMeasurement_;
        int currentPosition_;
    };


    class Strategy : public AbstractStrategy
    {
    public:
        // Constructor
        Strategy();

        // Called before the start of the match, to setup the robot.
        bool setup(RobotInterface *robot);

        // Code executed when shutting down the robot
        void shutdown() override;

        // The actual match code, which runs in its own thread.
        void match() override;

        void periodicAction() override;

        // socket to send start signal
        network::ClientSocket sock_;

        /// @brief Execute the sequence to grab cherries: set rail position, brush and reservoir tilt, go forward 150mm, wait, then back 150mm
        void grab_cherries();

        /// @brief Execute the sequence to put cherries in the basket: set rail position, go forward 150mm, tilt and start motor, then de-tilt, stop and go backwards
        void put_cherries_in_the_basket();

        /// @brief Blocking function for moving the rail to a specified height
        /// @param railHeight Normalized rail height, between 0 and 1
        void moveRail(double const& railHeight);


    private:
        void match_impl(); /// Actual implementation of the match code.

        STSServoDriver *servo;

        Action *chooseNextAction(
            std::vector<Action> &actions,
            RobotPosition currentPosition,
            MotionPlanner &motionPlanner);


        /// @brief Set the brush to move either direction or switch it off
        /// @param brushDirection the desired movement or off
        void set_brush_move(BrushDirection brushDirection);

        /// @brief Sets the reservoir to be tilted up or down
        /// @param reservoirTilt up or down
        void set_reservoir_tilt(ReservoirTilt reservoirTilt);


        void startSequenceTop();
        void startSequenceBottom();

        bool performSecondaryRobotAction(SecondaryRobotAction* action);

        void calibrateRail();
        void updateRailHeight();
        RailMeasurements currentRailMeasurements;

        int targetRailValue_;
        rail::state railState_ = rail::state::IDLE;
        /// @brief  Wait for the rail to stop moving.
        void waitForRail();

        // Go back to base (at the end of the match)
        void goBackToBase();
        bool checkIfBackToBase();
        bool countedPointsForGoingBackToBase_;

        bool startingTop_;
    };
}

#endif
