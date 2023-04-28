/// \file trajectory/PathPlanner.h
/// \brief Use a-star library to plan a suitable path around obstacles
/// \author MiAM Robotique, Quentin Chan Wai Nam
/// \copyright GNU GPLv3
#ifndef MIAM_TRAJECTORY_PATHPLANNER
#define MIAM_TRAJECTORY_PATHPLANNER
    #include "miam_utils/trajectory/Trajectory.h"
    #include "RobotPosition.h"
    #include <vector>
    #include <AStar.hpp>

    namespace miam{
        namespace trajectory{

            struct PathPlannerConfig
            {
                int astar_resolution_mm = 50;
                int astar_grid_size_x = 40;
                int astar_grid_size_y = 60;
            };

            /// @brief The path planning grid depicts the map. Each tile is a region of the map.
            /// Each tile (i, j) is centered on (i + 0.5) * resolution, (j + 0.5) * resolution.
            class PathPlanner
            {
                public:
                    PathPlanner(PathPlannerConfig const& config);

                    /// @brief Adds a collision to the a-star map
                    /// @param position position in robotposition
                    /// @param radius radius of the collision in mm
                    void addCollision(RobotPosition const& position, double radius);
                    
                    /// @brief Resets the collisions to only table border and cherry
                    /// distributors
                    void resetCollisions();
                    
                    /// @brief Prints a-star map
                    void printMap();
                    
                    /// @brief Prints a-star map along with a path
                    /// @param path path in robotposition
                    void printMap(
                        std::vector<RobotPosition> path, 
                        RobotPosition currentPosition, 
                        RobotPosition targetPosition
                    );

                    /// @brief Plans a path from a start to and end given collisions
                    /// @param start start robotposition
                    /// @param end end robotposition
                    /// @return empty vector if no path found, else path in robotpositions
                    std::vector<RobotPosition> planPath(
                        RobotPosition const& start, RobotPosition const& end);

                    /// @brief Converts robot position to a-star grid position
                    /// @param position position in robotposition
                    /// @return position in a-star grid
                    AStar::Vec2i robotPositionToVec2i(RobotPosition position);


                    /// @brief Converts a-star grid position to robot position
                    /// @param vec2i position in a-star grid
                    /// @return position in robotposition
                    RobotPosition vec2iToRobotPosition(AStar::Vec2i vec2i);

                protected:
                    PathPlannerConfig config_;
                    AStar::Generator generator_;

                    void addCollisionsNoDuplicate(AStar::Vec2i collision);
            };
        }
    }

#endif