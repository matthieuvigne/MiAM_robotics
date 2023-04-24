/// \author MiAM Robotique, Quentin Chan Wai Nam
/// \copyright GNU GPLv3
#include "miam_utils/trajectory/PathPlanner.h"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace miam{
    namespace trajectory{
        PathPlanner::PathPlanner(PathPlannerConfig const& config):
            config_(config)
        {
            generator_.setWorldSize({config_.astar_grid_size_x, config_.astar_grid_size_y});
            generator_.setHeuristic(AStar::Heuristic::euclidean);
            generator_.setDiagonalMovement(true);

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
                    generator_.addCollision({9, j});
                    generator_.addCollision({10, j});
                }
                for (int j = config_.astar_grid_size_y-1-4; j < config_.astar_grid_size_y; j++) 
                {
                    generator_.addCollision({9, j});
                    generator_.addCollision({10, j});
                }
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

        void PathPlanner::printMap(std::vector<RobotPosition> path)
        {
            AStar::Vec2i worldSize = generator_.getWorldSize();
            std::vector<AStar::Vec2i > pathInVec2i;
            for (auto& position : path)
            {
                pathInVec2i.push_back(robotPositionToVec2i(position));
            }
            std::cout << "Path: " << std::endl;
            for (auto& position : pathInVec2i)
            {
                std::cout << position.x << " " << position.y << std::endl;
            }

            
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
                    if (std::find(
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

        void PathPlanner::addCollision(RobotPosition const& position, double radius)
        {
            // TODO take radius into account
            generator_.addCollision(robotPositionToVec2i(position));
        }
                
        void PathPlanner::clearCollisions()
        {
            generator_.clearCollisions();
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
            for(auto coordinate = path.rbegin(); coordinate != path.rend(); ++coordinate) 
            {
                targetPosition = vec2iToRobotPosition(*coordinate);
                positions.push_back(targetPosition);
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
