/// \file GameState.h
/// \brief A class to represent the state of the table, and of the robot

#ifndef GAME_STATE_H
    #define GAME_STATE_H

    #include <gtkmm.h>
    #include "common/Types.h"

    // Center of the zones where the plants are on the table.
    extern const miam::RobotPosition COLLECT_ZONE_COORDS[10];

    /// Zones are numbered from top to bottom, left to right.
    class GameState
    {
        public:
            bool isCollectZoneFull[10] = {true, true, true, true, true, true, true, true, true, true};

            bool isFrontClawFull = false;
            bool isBackClawFull = false;

            /// @brief Draw current state as overlay on the table.
            /// @param cr A cairo::context, correctly scaled and offsetted.
            void draw(Cairo::RefPtr<Cairo::Context> const& cr, miam::RobotPosition const& robotPosition);
    };
 #endif
