#include <algorithm>
#include "miam_utils/Map.h"


Map::Map() :
    Eigen::MatrixXi(),
    gridSize_(10.0)
{
}

Map::Map(Eigen::MatrixXi mat, double const& gridSize) :
    Eigen::MatrixXi(mat),
    gridSize_(gridSize)
{
}


MapCoord Map::posToCoord(RobotPosition const& pos, bool unclamp) const
{
    // tail i, j is nearest to ((i + 0.5) * resolution, (j + 0.5) * resolution)
    int i = round(pos.x / gridSize_ - 0.5);
    int j = round(pos.y / gridSize_ - 0.5);
    if (unclamp)
        return MapCoord(i, j);
    return MapCoord(std::clamp(i, 0, static_cast<int>(rows())), std::clamp(j, 0, static_cast<int>(cols())));
}

bool Map::detectCollision(int const& x, int const& y) const
{
    if (x < 0 || x >= rows() || y < 0 || y >= cols())
        return true;
    return coeff(x, y) == 1;
}

RobotPosition Map::coordToPos(MapCoord const& pos) const
{
    return RobotPosition((pos.x + 0.5) * gridSize_, (pos.y + 0.5) * gridSize_, 0.0);
}

bool Map::detectCollision(MapCoord const& c) const
{
    return detectCollision(c.x, c.y);
}

bool Map::detectCollision(RobotPosition const& pos) const
{
    return detectCollision(posToCoord(pos, true));
}

RobotPosition Map::getNearestAvailablePosition(RobotPosition const& desiredPosition)
{
    MapCoord startPoint = posToCoord(desiredPosition);

    if (coeff(startPoint.x, startPoint.y) == 0)
        return desiredPosition;

    // Greedy algorithm: compute all distances
    double minDistance = 1e6;
    MapCoord candidate = startPoint;
    for (int x = 0; x < rows(); x++)
        for (int y = 0; y < cols(); y++)
        {
            if (detectCollision(x, y))
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

    return coordToPos(candidate);
}

void Map::print(Logger *logger,
                std::vector<RobotPosition> path,
                RobotPosition currentPosition,
                RobotPosition targetPosition) const
{
    std::vector<MapCoord > pathInMapCoord;
    for (auto& position : path)
    {
        pathInMapCoord.push_back(posToCoord(position));
    }
    *logger << "[PathPlanner] World size: " << rows() << ", " << cols() << std::endl;
    MapCoord current_position = posToCoord(currentPosition);
    MapCoord target_position = posToCoord(targetPosition);
    *logger << "[PathPlanner] Current position RobotPosition: " << currentPosition << std::endl;
    *logger << "[PathPlanner] Current position i, j: " << current_position.x << " " << current_position.y << std::endl;
    *logger << "[PathPlanner] Target position RobotPosition: " << targetPosition << std::endl;
    *logger << "[PathPlanner] Target position i, j: " << target_position.x << " " << target_position.y << std::endl;

    int const xSize = rows();
    int const ySize = cols();
    char mapBuffer[ySize][xSize];
    for (int j = 0; j < ySize; j++)
    {
        for (int i = 0; i < xSize; i++)
        {
            if (coeff(i, j) == 1)
                mapBuffer[j][i] = 'X';
            else
                mapBuffer[j][i] = ' ';
        }
    }
    for (auto const& coord : pathInMapCoord)
        mapBuffer[coord.y][coord.x] = 'o';
    mapBuffer[current_position.y][current_position.x] = 'S';
    mapBuffer[target_position.y][target_position.x] = 'E';

    // Print buffer, reverse order
    for (int j = ySize - 1; j >= 0; j--)
        *logger << std::string(&mapBuffer[j][0], xSize) << std::endl;
}


void Map::addCollisionCircle(RobotPosition const& position, double radius)
{
    // number of grid units to be taken into account
    int grid_distance = ceil(radius / gridSize_);
    MapCoord center_position = posToCoord(position);

    // in a big square around the center position, draw obstacle
    for (int i = std::max(0, center_position.x - grid_distance);
        i < std::min(static_cast<int>(rows()), center_position.x + grid_distance + 1);
        i++)
    {
        for (int j = std::max(0, center_position.y - grid_distance);
            j < std::min(static_cast<int>(cols()), center_position.y + grid_distance + 1);
            j++)
        {
            RobotPosition obsPosition = coordToPos({i, j});
            if ((position - obsPosition).norm() < radius)
            {
                (*this)(i, j) = 1;
            }
        }
    }
}
