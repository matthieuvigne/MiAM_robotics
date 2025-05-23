/// \author MiAM Robotique, Quentin Chan Wai Nam
/// \copyright GNU GPLv3
#include "miam_utils/trajectory/PathPlanner.h"
#include "miam_utils/AStar.hpp"

namespace miam::trajectory{
    std::vector<RobotPosition> planPath(Map& map, RobotPosition const& start, RobotPosition const& end)
    {
        AStar::Generator generator;
        generator.setHeuristic(AStar::Heuristic::euclidean);
        generator.setDiagonalMovement(true);

        std::vector<RobotPosition> positions;
        RobotPosition targetPosition;

        MapCoord startPoint = map.posToCoord(start);
        MapCoord endPoint = map.posToCoord(end);

        // remove start position from obstacles
        map(startPoint.x, startPoint.y) = 0;

        // Abort if end point is in collision
        if (map.detectCollision(endPoint))
            return positions;

        auto path = generator.findPath(
            map,
            startPoint,
            endPoint
        );


        bool checkPath = true;
        for (unsigned int i = 0; i < path.size() - 1; i++)
        {
            MapCoord p1 = path.at(i);
            MapCoord p2 = path.at(i+1);
            if (abs(p1.x - p2.x) > 1 || abs(p1.y - p2.y) > 1)
            {
                checkPath = false;
                break;
            }
        }


        if (checkPath && map.posToCoord(start) == path.back() && map.posToCoord(end) == path.front())
        {
            double const GRID_DIAGONAL = map.getGridSize() * 1.4142;

            positions.push_back(start);
            for (auto coordinate = path.rbegin(); coordinate != path.rend(); ++coordinate)
            {
                targetPosition = map.coordToPos(*coordinate);
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

        return positions;
    }
}
