/// \author MiAM Robotique, Quentin Chan Wai Nam
/// \copyright GNU GPLv3
#include "miam_utils/trajectory/PathPlanner.h"
#include <iostream>
#include <cmath>
#include <algorithm>

using AStar::Vec2i;

namespace miam{
    namespace trajectory{
        PathPlanner::PathPlanner(PathPlannerConfig const& config, Logger *logger):
            config_(config),
            logger_(logger)
        {
            generator_.setWorldSize({config_.astar_grid_size_x, config_.astar_grid_size_y});
            generator_.setHeuristic(AStar::Heuristic::euclidean);
            generator_.setDiagonalMovement(true);

            resetCollisions();
        }

        void PathPlanner::printMap()
        {
            AStar::Vec2i worldSize = generator_.getWorldSize();

            *logger_ << "[PathPlanner] World size: " << worldSize.x << ", " << worldSize.y << std::endl;
            // *logger_ << "[PathPlanner] Collisions: " << std::endl;
            // for (auto collision : generator_.getCollisions())
            // {
            //     *logger_ << "[PathPlanner] " <<collision.x << ", " << collision.y << std::endl;
            // }

            std::vector<AStar::Vec2i > collisions = generator_.getCollisions();

            for (int j = 0; j < generator_.getWorldSize().y; j++)
            {
                for (int i = 0; i < generator_.getWorldSize().x; i++)
                {
                    AStar::Vec2i target({i, j});
                    if (std::find(
                        collisions.begin(),
                        collisions.end(),
                        target) != collisions.end())
                        {
                            *logger_ << "X ";
                        }
                        else
                        {
                            *logger_ << "  ";
                        }
                }
                *logger_ << std::endl;
            }
        }

        void PathPlanner::printMap(
            std::vector<RobotPosition> path,
            RobotPosition currentPosition,
            RobotPosition targetPosition)
        {
            AStar::Vec2i worldSize = generator_.getWorldSize();
            std::vector<AStar::Vec2i > pathInVec2i;
            for (auto& position : path)
            {
                pathInVec2i.push_back(robotPositionToVec2i(position));
            }
            // *logger_ << "[PathPlanner] Path: " << std::endl;
            // for (auto& position : pathInVec2i)
            // {
            //     *logger_ << "[PathPlanner] " <<position.x << " " << position.y << std::endl;
            // }


            *logger_ << "[PathPlanner] World size: " << worldSize.x << ", " << worldSize.y << std::endl;
            // *logger_ << "[PathPlanner] Collisions: " << std::endl;
            // for (auto collision : generator_.getCollisions())
            // {
            //     *logger_ << "[PathPlanner] " <<collision.x << ", " << collision.y << std::endl;
            // }

            std::vector<AStar::Vec2i > collisions = generator_.getCollisions();

            Vec2i current_position = robotPositionToVec2i(currentPosition);
            Vec2i target_position = robotPositionToVec2i(targetPosition);
            *logger_ << "[PathPlanner] Current position RobotPosition: " << currentPosition << std::endl;
            *logger_ << "[PathPlanner] Current position i, j: " << current_position.x << " " << current_position.y << std::endl;
            *logger_ << "[PathPlanner] Target position RobotPosition: " << targetPosition << std::endl;
            *logger_ << "[PathPlanner] Target position i, j: " << target_position.x << " " << target_position.y << std::endl;

            for (int j = generator_.getWorldSize().y - 1; j >= 0; j--)
            {
                for (int i = 0; i < generator_.getWorldSize().x; i++)
                {
                    AStar::Vec2i target({i, j});
                    if (target == current_position)
                    {
                        *logger_ << "S";
                    }
                    else if (target == target_position)
                    {
                        *logger_ << "E";
                    }
                    else if (std::find(
                        collisions.begin(),
                        collisions.end(),
                        target) != collisions.end())
                    {
                        *logger_ << "X";
                    }
                    else if (std::find(
                        pathInVec2i.begin(),
                        pathInVec2i.end(),
                        target) != pathInVec2i.end())
                    {
                        *logger_ << "o";
                    }
                    else
                    {
                        *logger_ << " ";
                    }
                }
                *logger_ << "" <<std::endl;
            }
        }

        void PathPlanner::addCollisionsNoDuplicate(AStar::Vec2i collision)
        {
            AStar::CoordinateList existingCollisions = generator_.getCollisions();
            if (!(std::find(existingCollisions.begin(), existingCollisions.end(), collision) != existingCollisions.end()))
            {
                generator_.addCollision(collision);
            }
        }

        void PathPlanner::addCollision(RobotPosition const& position, double radius)
        {
            // number of grid units to be taken into account
            int grid_distance = ceil(radius / config_.astar_resolution_mm);
            AStar::Vec2i center_position = robotPositionToVec2i(position);

            // in a big square around the center position, draw obstacle
            for (int i = std::max(0, center_position.x - grid_distance);
                i < std::min(config_.astar_grid_size_x, center_position.x + grid_distance + 1);
                i++)
            {
                for (int j = std::max(0, center_position.y - grid_distance);
                    j < std::min(config_.astar_grid_size_y, center_position.y + grid_distance + 1);
                    j++)
                {
                    RobotPosition obsPosition = vec2iToRobotPosition({i, j});
                    if ((position - obsPosition).norm() < radius)
                    {
                        addCollisionsNoDuplicate({i, j});
                    }
                }
            }
        }

        void PathPlanner::resetCollisions()
        {
            generator_.clearCollisions();

            // obstacles table
            // bordure exterieure
            int forbidden_border_size_grid = config_.forbidden_border_size_mm / config_.astar_resolution_mm;

            for (int i = 0; i < config_.astar_grid_size_x; i++)
            {
                for (int k = 0; k <= forbidden_border_size_grid; k++)
                {
                    generator_.addCollision({i, k});
                    generator_.addCollision({i, config_.astar_grid_size_y-1-k});

                }
            }
            for (int j = 0; j < config_.astar_grid_size_y; j++)
            {
                for (int k = 0; k <= forbidden_border_size_grid; k++)
                {
                    generator_.addCollision({k, j});
                    generator_.addCollision({config_.astar_grid_size_x-1-k, j});
                }
            }
        }

        std::vector<RobotPosition> PathPlanner::planPath(RobotPosition const& start, RobotPosition const& end)
        {

            std::vector<RobotPosition> positions;
            RobotPosition targetPosition;

            AStar::Vec2i startPoint = robotPositionToVec2i(start);
            AStar::Vec2i endPoint = robotPositionToVec2i(end);

            *logger_ << "[PathPlanner] Planning from: " << startPoint.x << " " << startPoint.y << std::endl;
            *logger_ << "[PathPlanner] Planning to  : " << endPoint.x << " " << endPoint.y << std::endl;

            // remove start position from obstacles
            generator_.removeCollision(startPoint);

            if (generator_.detectCollision(endPoint))
            {
                *logger_ << "[PathPlanner] PathPlanning failed: end point is in obstacle!" << std::endl;
                return positions;
            }

            *logger_ << "[PathPlanner] Generate path ... \n";
            auto path = generator_.findPath(
                startPoint,
                endPoint
            );


            bool checkPath = true;
            for (unsigned int i = 0; i < path.size() - 1; i++)
            {
                AStar::Vec2i p1 = path.at(i);
                AStar::Vec2i p2 = path.at(i+1);
                if (abs(p1.x - p2.x) > 1 || abs(p1.y - p2.y) > 1)
                {
                    checkPath = false;
                    break;
                }
            }


            if (checkPath && robotPositionToVec2i(start) == path.back() && robotPositionToVec2i(end) == path.front())
            {
                for (auto coordinate = path.rbegin(); coordinate != path.rend(); ++coordinate)
                {
                    targetPosition = vec2iToRobotPosition(*coordinate);
                    positions.push_back(targetPosition);
                }

                // remplacer extremites par start et end
                positions.front() = start;
                positions.back() = end;

                // Give to each point the angle corresponding to the line going to thext point.
                for (unsigned int i = 1; i < positions.size(); i++)
                {
                    double const dx = positions.at(i).x - positions.at(i-1).x;
                    double const dy = positions.at(i).y - positions.at(i-1).y;
                    positions.at(i).theta = std::atan2(dy, dx);
                }
                if (positions.size() > 1)
                    positions.at(0).theta = positions.at(1).theta;
            }
            else
            {
                *logger_ << "[PathPlanner] Path planning failed" << std::endl;
            }

            return positions;
        }

        RobotPosition PathPlanner::vec2iToRobotPosition(AStar::Vec2i astarWaypoint) {
            return RobotPosition(
                astarWaypoint.x * config_.astar_resolution_mm + config_.astar_resolution_mm / 2,
                astarWaypoint.y * config_.astar_resolution_mm + config_.astar_resolution_mm / 2,
                0
            );
        }

        AStar::Vec2i PathPlanner::robotPositionToVec2i(RobotPosition position) {

            // tail i, j is nearest to ((i + 0.5) * resolution, (j + 0.5) * resolution)
            int i = round(position.x / config_.astar_resolution_mm - 0.5);
            int j = round(position.y / config_.astar_resolution_mm - 0.5);
            return {i, j};
        }
    }
}
