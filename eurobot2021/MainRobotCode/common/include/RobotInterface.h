/// \file RobotInterface.h
/// \brief Interface for 2021 robot - for use on the robot and in simulation
/// \author MiAM Robotique, Matthieu Vigne
/// \author Rodolphe Dubois
/// \author Quentin Chan-Wai-Nam
/// \copyright GNU GPLv3

#ifndef ROBOT_INTERFACE_H
     #define ROBOT_INTERFACE_H

    #include <miam_utils/AbstractRobot.h>
    #include <unistd.h>
    #include "Parameters.h"

    class RobotInterface : public AbstractRobot
    {
        public:
            /// \brief Set a new target to the rail.
            /// \param position Relative rail position, form 0 (down) to 1 (up).
            virtual void moveRail(double const& position) = 0;

            // Sleep a specified number of seconds.
            virtual void wait(double const& waitTimeS)
            {
                usleep(static_cast<int>(1e6 * waitTimeS));
            }

            virtual bool isPlayingRightSide()
            {
                return false;
            }

            virtual ExcavationSquareColor getExcavationReadings(bool readRightSide)
            {
                return ExcavationSquareColor::RED;
            }
    };
 #endif
