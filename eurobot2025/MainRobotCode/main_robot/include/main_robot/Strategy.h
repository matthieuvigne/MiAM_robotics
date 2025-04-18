/// \file Strategy.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef MAIN_ROBOT_STRATEGY_H
#define MAIN_ROBOT_STRATEGY_H

#include "common/RobotInterface.h"
#include "common/AbstractStrategy.h"
#include "common/MotionPlanner.h"

#include "main_robot/AbstractAction.h"
#include "main_robot/ServoManager.h"

#include <mutex>
#include <array>

namespace main_robot
{
class Strategy : public AbstractStrategy
{
    public:

        // Constructor
        Strategy(bool const& interactive = false);

        // Called before the start of the match, to setup the robot.
        bool setup(RobotInterface *robot) override;

        // Code executed when shutting down the robot
        void shutdown() override;

        // The actual match code, which runs in its own thread.
        void match() override;

    private:
        void match_impl(); /// Actual implementation of the match code.

        void goBackToBase();

        // Perform a given action, return true if action was successful.
        bool performAction(std::shared_ptr<AbstractAction> action, bool & actionShouldBeRemoved);

        std::vector<std::shared_ptr<AbstractAction>> actions_;  ///< All actions that can be performed.

        ServoManager servoManager_;
        bool isSetupFirstInstance_ = true;

        bool interactive_; ///< Actions are choosen by the user
  };
}



#endif
