/// \file GameState.h
/// \brief A class to represent the state of the table, and of the robot

#ifndef GAME_STATE_H
    #define GAME_STATE_H

    #include <gtkmm.h>
    #include "common/Types.h"

    #include "miam_utils/Map.h"

    #define NUMBER_OF_COLLECT_ZONES 8
    #define NUMBER_OF_PANTRY_ZONES 10

    // Center of the zones where the columns are on the table.
    extern const miam::RobotPosition COLLECT_ZONE_COORDS[NUMBER_OF_COLLECT_ZONES];
    extern const miam::RobotPosition PANTRY_ZONE_COORDS[NUMBER_OF_PANTRY_ZONES];

    /// Zones are numbered from top to bottom, left to right.
    class GameState
    {
        public:
            GameState();
            bool isCollectZoneFull[NUMBER_OF_COLLECT_ZONES];
            bool isPantryZoneUsed[NUMBER_OF_PANTRY_ZONES];

            bool isRobotFull = false;
            bool isClawFull = false;

            bool arePAMIMoving_ = false;

            /// @brief Draw current state as overlay on the table.
            /// @param cr A cairo::context, correctly scaled and offsetted.
            void draw(Cairo::RefPtr<Cairo::Context> const& cr, miam::RobotPosition const& robotPosition, bool const& isPlayingRightSide);

            /// @brief Generate map for obstacle avoidance
            Map generateMap();

            /// \brief Try to guess the actions of the other robot from the obstacle list
            /// \param[in] obstacles List of obstacles
            /// \param[in] time Current match time
            /// \param[in] logger Debugging logger
            void detectOtherRobotAction(std::vector<Obstacle> const& obstacles, double time, Logger *logger);
        private:
            double timeOtherRobotEnteredZone_[9] = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
    };
 #endif
