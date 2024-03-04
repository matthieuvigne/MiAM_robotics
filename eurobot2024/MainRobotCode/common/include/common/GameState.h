/// \file GameState.h
/// \brief A class to represent the state of the table, and of the robot

#ifndef GAME_STATE_H
    #define GAME_STATE_H

    #include <gtkmm.h>
    #include "common/Types.h"

    // Center of the zones where the plants are on the table.
    extern const miam::RobotPosition PLANT_COLLECT_COORD[6];
    extern const miam::RobotPosition PLANT_DROP_DISPLAY_COORD[6];

    enum class ClawContent{
        EMPTY,
        UNKNOWN_PLANT,
        WHITE_PLANT,
        PURPLE_PLANT
    };

    class GameState
    {
        public:
            /// @brief Number of plants for each zone on the table
            ///        Zones are numbered from top to bottom, left to right.
            int nPlantsPerZone[6] = {6, 6, 6, 6, 6, 6};
            /// @brief Number of plants in each collected zone.
            ///        1-3: standard zones ; 4-6: jardiniere
            int nPlantsCollected[6] = {0, 0, 0, 0, 0, 0};
            int nPotsInPile[4] = {6, 6, 6, 6};

            /// Content of the robot's claws
            ClawContent robotClawContent[6] = {ClawContent::EMPTY,
                                               ClawContent::EMPTY,
                                               ClawContent::EMPTY,
                                               ClawContent::EMPTY,
                                               ClawContent::EMPTY,
                                               ClawContent::EMPTY};

            bool isClawAvailable(bool const isFront) const
            {
                int i = 0;
                for (int j = 0; j < 3; j++)
                    if (robotClawContent[(isFront ? 0 : 3) + j] == ClawContent::EMPTY)
                        i++;
                return i >= 2;
            }


            int nPlantsInRobot()
            {
                int nPlants = 0;
                for (int j = 0; j < 6; j++)
                    if (robotClawContent[j] != ClawContent::EMPTY)
                        nPlants++;
                return nPlants;
            }

            bool isFrontClawMostFull()
            {
                int nFront = 0, nBack = 0;
                for (int j = 0; j < 3; j++)
                    if (robotClawContent[j] != ClawContent::EMPTY)
                        nFront++;
                for (int j = 0; j < 3; j++)
                    if (robotClawContent[3 + j] != ClawContent::EMPTY)
                        nBack++;
                return nFront >= nBack;
            }

            int clearClawContent(bool const& isFront)
            {
                int nPlants = 0;
                for (int j = 0; j < 3; j++)
                {
                    int idx = j + (isFront ? 3 : 0);
                    if (robotClawContent[idx] != ClawContent::EMPTY)
                        nPlants++;
                    robotClawContent[idx] = ClawContent::EMPTY;
                }
                return nPlants;
            }

            /// @brief Draw current state as overlay on the table.
            /// @param cr A cairo::context, correctly scaled and offsetted.
            void draw(Cairo::RefPtr<Cairo::Context> const& cr, miam::RobotPosition const& robotPosition);
    };
 #endif
