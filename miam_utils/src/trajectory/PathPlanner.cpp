/// \author MiAM Robotique, Quentin Chan Wai Nam
/// \copyright GNU GPLv3
#include "miam_utils/trajectory/PathPlanner.h"
#include <iostream>
#include <cmath>
#include <algorithm>

using AStar::Vec2i;


namespace miam::trajectory{

    // Utility functions: point coordinate conversion
    RobotPosition vec2iToRobotPosition(AStar::Vec2i const& astarWaypoint, double const& resolution)
    {
        return RobotPosition(
            astarWaypoint.x * resolution + resolution / 2,
            astarWaypoint.y * resolution + resolution / 2,
            0
        );
    }


    AStar::Vec2i robotPositionToVec2i(RobotPosition const& position, double const& resolution)
    {
        // tail i, j is nearest to ((i + 0.5) * resolution, (j + 0.5) * resolution)
        int i = round(position.x / resolution - 0.5);
        int j = round(position.y / resolution - 0.5);
        return {i, j};
    }


    PathPlanner::PathPlanner(PathPlannerConfig const& config, Logger *logger):
        config_(config),
        logger_(logger)
    {
        generator_.setWorldSize({static_cast<int>(config_.map.rows()), static_cast<int>(config_.map.cols())});
        generator_.setHeuristic(AStar::Heuristic::euclidean);
        generator_.setDiagonalMovement(true);

        resetCollisions();
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
            pathInVec2i.push_back(robotPositionToVec2i(position, config_.astar_resolution_mm));
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

        // std::vector<AStar::Vec2i > collisions = generator_.getCollisions();

        Vec2i current_position = robotPositionToVec2i(currentPosition, config_.astar_resolution_mm);
        Vec2i target_position = robotPositionToVec2i(targetPosition, config_.astar_resolution_mm);
        *logger_ << "[PathPlanner] Current position RobotPosition: " << currentPosition << std::endl;
        *logger_ << "[PathPlanner] Current position i, j: " << current_position.x << " " << current_position.y << std::endl;
        *logger_ << "[PathPlanner] Target position RobotPosition: " << targetPosition << std::endl;
        *logger_ << "[PathPlanner] Target position i, j: " << target_position.x << " " << target_position.y << std::endl;

        int const xSize = generator_.getWorldSize().x;
        int const ySize = generator_.getWorldSize().y;
        char mapBuffer[ySize][xSize];
        for (int j = 0; j < ySize; j++)
        {
            for (int i = 0; i < xSize; i++)
            {
                if (generator_.obstacleMap_(i, j) == 1)
                    mapBuffer[j][i] = 'X';
                else
                    mapBuffer[j][i] = ' ';
            }
        }
        for (auto const& coord : pathInVec2i)
            mapBuffer[coord.y][coord.x] = 'o';
        mapBuffer[current_position.y][current_position.x] = 'S';
        mapBuffer[target_position.y][target_position.x] = 'E';

        // Print buffer, reverse order
        for (int j = ySize - 1; j >= 0; j--)
            *logger_ << std::string(&mapBuffer[j][0], xSize) << std::endl;

    }

    void PathPlanner::addCollision(RobotPosition const& position, double radius)
    {
        // number of grid units to be taken into account
        int grid_distance = ceil(radius / config_.astar_resolution_mm);
        AStar::Vec2i center_position = robotPositionToVec2i(position, config_.astar_resolution_mm);

        // in a big square around the center position, draw obstacle
        for (int i = std::max(0, center_position.x - grid_distance);
            i < std::min(static_cast<int>(config_.map.rows()), center_position.x + grid_distance + 1);
            i++)
        {
            for (int j = std::max(0, center_position.y - grid_distance);
                j < std::min(static_cast<int>(config_.map.cols()), center_position.y + grid_distance + 1);
                j++)
            {
                RobotPosition obsPosition = vec2iToRobotPosition({i, j}, config_.astar_resolution_mm);
                if ((position - obsPosition).norm() < radius)
                {
                    generator_.addCollision({i, j});
                }
            }
        }
    }

    void PathPlanner::resetCollisions()
    {
        generator_.obstacleMap_ = config_.map;
    }

    std::vector<RobotPosition> PathPlanner::planPath(RobotPosition const& start, RobotPosition const& end)
    {

        std::vector<RobotPosition> positions;
        RobotPosition targetPosition;

        AStar::Vec2i startPoint = robotPositionToVec2i(start, config_.astar_resolution_mm);
        AStar::Vec2i endPoint = robotPositionToVec2i(end, config_.astar_resolution_mm);

        *logger_ << "[PathPlanner] Planning from: " << startPoint.x << " " << startPoint.y << std::endl;
        *logger_ << "[PathPlanner] Planning to  : " << endPoint.x << " " << endPoint.y << std::endl;

        // remove start position from obstacles
        generator_.removeCollision(startPoint);

        if (generator_.detectCollision(endPoint))
        {
            *logger_ << "[PathPlanner] PathPlanning failed: end point is in obstacle!" << std::endl;
            return positions;
        }

        *logger_ << "[PathPlanner] Generate path ... " << std::endl;
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


        if (checkPath && robotPositionToVec2i(start, config_.astar_resolution_mm) == path.back() && robotPositionToVec2i(end, config_.astar_resolution_mm) == path.front())
        {
            double const GRID_DIAGONAL = config_.astar_resolution_mm * 1.4142;

            positions.push_back(start);
            for (auto coordinate = path.rbegin(); coordinate != path.rend(); ++coordinate)
            {
                targetPosition = vec2iToRobotPosition(*coordinate, config_.astar_resolution_mm);
                // Don't add points that are less than one grid coordinate from start / end.
                // This is done to avoid artifacts due to the start / end being off-grid.
                if ((targetPosition - start).norm() > 1.5 * GRID_DIAGONAL &&
                    (targetPosition - end).norm() > 1.5 * GRID_DIAGONAL)
                    positions.push_back(targetPosition);
            }
            positions.push_back(end);

            // Give to each point the angle corresponding to the line going to the next point.
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

    RobotPosition PathPlanner::getNearestAvailablePosition(RobotPosition const& desiredPosition)
    {
        AStar::Vec2i startPoint = robotPositionToVec2i(desiredPosition, config_.astar_resolution_mm);

        if (generator_.obstacleMap_(startPoint.x, startPoint.y) == 0)
            return desiredPosition;

        // Greedy algorithm: compute all distances
        double minDistance = 1e6;
        AStar::Vec2i candidate = startPoint;
        AStar::Vec2i worldSize = generator_.getWorldSize();
        for (int x = 0; x < worldSize.x; x++)
            for (int y = 0; y < worldSize.x; y++)
            {
                if (generator_.detectCollision({x, y}))
                    continue;
                // Distance squared: since sqrt is monotonous, no need to take square root.
                double const distance = (x - startPoint.x) * (x - startPoint.x) + (y - startPoint.y) * (y - startPoint.y);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    candidate.x = x;
                    candidate.y = y;
                }
            }

        return vec2iToRobotPosition(candidate, config_.astar_resolution_mm);
    }
}
