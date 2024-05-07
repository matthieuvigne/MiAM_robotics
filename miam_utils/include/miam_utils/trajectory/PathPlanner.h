/// \file trajectory/PathPlanner.h
/// \brief Use a-star library to plan a suitable path around obstacles
/// \author MiAM Robotique, Quentin Chan Wai Nam
/// \copyright GNU GPLv3
#ifndef MIAM_TRAJECTORY_PATHPLANNER
#define MIAM_TRAJECTORY_PATHPLANNER
    #include "miam_utils/trajectory/Trajectory.h"
    #include "miam_utils/Logger.h"
    #include "RobotPosition.h"
    #include <vector>
    #include "miam_utils/AStar.hpp"

    namespace miam{
        namespace trajectory{

            struct PathPlannerConfig
            {
                int astar_resolution_mm = 50;
                Eigen::MatrixXi map = Eigen::MatrixXi::Zero(1, 1); // Map containing default obstacles
            };

            /// @brief The path planning grid depicts the map. Each tile is a region of the map.
            /// Each tile (i, j) is centered on (i + 0.5) * resolution, (j + 0.5) * resolution.
            class PathPlanner
            {
                public:
                    PathPlanner(PathPlannerConfig const& config, Logger *logger);

                    /// @brief Adds a collision to the a-star map
                    /// @param position position in robotposition
                    /// @param radius radius of the collision in mm
                    void addCollision(RobotPosition const& position, double radius);

                    /// @brief Resets the collisions to only table border and cherry
                    /// distributors
                    void resetCollisions();

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

                    /// @brief Return the closest position in the grid which is not in an obstacle.
                    /// @param desiredPosition Desired position
                    /// @return Closest available cell
                    RobotPosition getNearestAvailablePosition(RobotPosition const& desiredPosition);
                private:
                    PathPlannerConfig config_;
                    AStar::Generator generator_;
                    Logger* logger_;
            };
        }
    }

#endif
