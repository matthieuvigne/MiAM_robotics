/// \file trajectory/PathPlanner.h
/// \brief Use a-star library to plan a suitable path around obstacles
/// \author MiAM Robotique, Quentin Chan Wai Nam
/// \copyright GNU GPLv3
#ifndef MIAM_TRAJECTORY_PATHPLANNER
#define MIAM_TRAJECTORY_PATHPLANNER
    #include "miam_utils/Logger.h"
    #include "miam_utils/Map.h"
    #include "miam_utils/trajectory/RobotPosition.h"

    namespace miam{
        namespace trajectory{

            /// @brief Plan geometric path from start to end position
            ///
            /// @note This function retuns a geometric path of waypoints,
            ///       without any time parametrization: it is a path, not a trajectory.
            /// @param map map on which to plan motion
            /// @param start start robotposition
            /// @param end end robotposition
            /// @param logger Logger for printing debug info
            /// @return empty vector if no path found, else path in robotpositions
            std::vector<RobotPosition> planPath(
                Map& map,
                RobotPosition const& start,
                RobotPosition const& end,
                Logger *logger);
        }
    }

#endif
