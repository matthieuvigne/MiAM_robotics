/// \file trajectory/PathPlanner.h
/// \brief Use a-star library to plan a suitable path around obstacles
/// \author MiAM Robotique, Quentin Chan Wai Nam
/// \copyright GNU GPLv3
#ifndef MIAM_TRAJECTORY_PATHPLANNER
#define MIAM_TRAJECTORY_PATHPLANNER
    #include "miam_utils/trajectory/Trajectory.h"
    #include "RobotPosition.h"
    #include <vector>
    #include "AStar.hpp"

    namespace miam{
        namespace trajectory{

            struct PathPlannerConfig
            {
                int astar_resolution_mm = 100;
                int astar_grid_size_x = 20;
                int astar_grid_size_y = 30;
            };

            /// @brief The path planning grid depicts the map. Each tile is a region of the map.
            /// Each tile (i, j) is centered on (i + 0.5) * resolution, (j + 0.5) * resolution.
            class PathPlanner
            {
                public:
                    PathPlanner(PathPlannerConfig const& config);

                    void addCollision(RobotPosition const& position, double radius);
                    void clearCollisions();
                    void printMap();

                    std::vector<RobotPosition> planPath(
                        RobotPosition const& start, RobotPosition const& end);

                    AStar::Vec2i robotPositionToVec2i(RobotPosition position);
                    RobotPosition vec2iToRobotPosition(AStar::Vec2i vec2i);

                protected:
                    PathPlannerConfig config_;
                    AStar::Generator generator_;
            };
        }
    }

#endif