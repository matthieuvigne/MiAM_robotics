/// \author MiAM Robotique, Quentin Chan Wai Nam
/// \copyright GNU GPLv3
#include "miam_utils/trajectory/PathPlanner.h"
#include <iostream>
#include <cmath>
#include <algorithm>

using AStar::Vec2i;

namespace miam{
    namespace trajectory{
        PathPlanner::PathPlanner(PathPlannerConfig const& config):
            config_(config)
        {
            generator_.setWorldSize({config_.astar_grid_size_x, config_.astar_grid_size_y});
            generator_.setHeuristic(AStar::Heuristic::euclidean);
            generator_.setDiagonalMovement(true);

            resetCollisions();
        }

        void PathPlanner::printMap() 
        {
            AStar::Vec2i worldSize = generator_.getWorldSize();
            
            std::cout << "World size: " << worldSize.x << ", " << worldSize.y << std::endl;
            // std::cout << "Collisions: " << std::endl;
            // for (auto collision : generator_.getCollisions())
            // {
            //     std::cout << collision.x << ", " << collision.y << std::endl;
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
                            std::cout << "X ";
                        }
                        else
                        {
                            std::cout << "  ";
                        }
                }
                std::cout << std::endl;
            }
        }

        void PathPlanner::printMap(std::vector<RobotPosition> path, RobotPosition currentPosition)
        {
            AStar::Vec2i worldSize = generator_.getWorldSize();
            std::vector<AStar::Vec2i > pathInVec2i;
            for (auto& position : path)
            {
                pathInVec2i.push_back(robotPositionToVec2i(position));
            }
            // std::cout << "Path: " << std::endl;
            // for (auto& position : pathInVec2i)
            // {
            //     std::cout << position.x << " " << position.y << std::endl;
            // }

            
            std::cout << "World size: " << worldSize.x << ", " << worldSize.y << std::endl;
            // std::cout << "Collisions: " << std::endl;
            // for (auto collision : generator_.getCollisions())
            // {
            //     std::cout << collision.x << ", " << collision.y << std::endl;
            // }

            std::vector<AStar::Vec2i > collisions = generator_.getCollisions();

            for (int j = generator_.getWorldSize().y - 1; j >= 0; j--)
            {
                for (int i = 0; i < generator_.getWorldSize().x; i++)
                {
                    AStar::Vec2i target({i, j});
                    if (target == robotPositionToVec2i(currentPosition))
                    {
                        std::cout << "O ";
                    }
                    else if (std::find(
                        collisions.begin(),
                        collisions.end(),
                        target) != collisions.end())
                    {
                        std::cout << "X ";
                    }
                    else if (std::find(
                        pathInVec2i.begin(),
                        pathInVec2i.end(),
                        target) != pathInVec2i.end())
                    {
                        std::cout << "o ";
                    }
                    else
                    {
                        std::cout << "  ";
                    }
                }
                std::cout << std::endl;
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
            for (int i = 0; i < config_.astar_grid_size_x; i++) 
            {
                generator_.addCollision({i, 0});
                generator_.addCollision({i, config_.astar_grid_size_y-1});
            }
            for (int j = 0; j < config_.astar_grid_size_y; j++) 
            {
                generator_.addCollision({0, j});
                generator_.addCollision({config_.astar_grid_size_x-1, j});
            }

            // distributeurs de cerises
            for (int j = 0; j < 4; j++) 
            {
                generator_.addCollision({8, j});
                generator_.addCollision({9, j});
                generator_.addCollision({10, j});
                generator_.addCollision({11, j});
            }
            for (int j = config_.astar_grid_size_y-1-4; j < config_.astar_grid_size_y; j++) 
            {
                generator_.addCollision({8, j});
                generator_.addCollision({9, j});
                generator_.addCollision({10, j});
                generator_.addCollision({11, j});
            }
        }

        std::vector<RobotPosition> PathPlanner::planPath(RobotPosition const& start, RobotPosition const& end)
        {
            std::cout << "Generate path ... \n";
            auto path = generator_.findPath(
                robotPositionToVec2i(start),
                robotPositionToVec2i(end)
            );

            std::vector<RobotPosition> positions;
            RobotPosition targetPosition;

            bool checkPath = true;
            for (int i = 0; i < path.size() - 1; i++)
            {
                AStar::Vec2i p1 = path.at(i);
                AStar::Vec2i p2 = path.at(i+1);
                if (abs(p1.x - p2.x) > 1 | abs(p1.y - p2.y) > 1)
                {
                    checkPath = false;
                    break;
                }
            }

            // remove start position from obstacles
            generator_.removeCollision(robotPositionToVec2i(start));

            if (checkPath & robotPositionToVec2i(start) == path.back() & robotPositionToVec2i(end) == path.front())
            {
                for (auto coordinate = path.rbegin(); coordinate != path.rend(); ++coordinate) 
                {
                    targetPosition = vec2iToRobotPosition(*coordinate);
                    positions.push_back(targetPosition);
                }

                // remplacer extremites par start et end
                positions.front() = start;
                positions.back() = end;

                // smooth out angles
                for (int i = 1; i < positions.size(); i++)
                {
                    double theta;

                    // compute angle
                    double dx = positions.at(i).x - positions.at(i-1).x;
                    double dy = positions.at(i).y - positions.at(i-1).y;
                    
                    if (dx == 0)
                    {
                        theta = (dy > 0 ? 1 : -1) * M_PI_2;
                    }
                    else
                    {
                        theta = atan(dy/dx);
                    }
                    positions.at(i-1).theta = theta;
                    positions.at(i).theta = theta;
                }
            } 
            else
            {
                std::cout << "Path planning failed" << std::endl;
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
