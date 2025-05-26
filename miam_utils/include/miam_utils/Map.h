#ifndef MAP_H
#define MAP_H

#include <eigen3/Eigen/Core>
#include <algorithm>

#include "miam_utils/trajectory/RobotPosition.h"
#include "miam_utils/Logger.h"

using miam::RobotPosition;

struct MapCoord
{
    int x, y;

    MapCoord() : x(0), y(0) {}
    MapCoord(int i, int j) : x(i), y(j) {}

    bool operator ==(MapCoord const& other)
    {
        return x == other.x && y == other.y;
    }
};

inline MapCoord operator+(MapCoord a, MapCoord b)
{
    return MapCoord(a.x + b.x, a.y + b.y);
}

class Map: public Eigen::MatrixXi
{
    public:
        Map();
        Map(Eigen::MatrixXi mat, double const& gridSize);

        MapCoord posToCoord(RobotPosition const& pos, bool unclamp=false) const;
        RobotPosition coordToPos(MapCoord const& pos) const;

        bool detectCollision(int const& x, int const& y) const;
        bool detectCollision(MapCoord const& c) const;
        bool detectCollision(RobotPosition const& pos) const;

        /// @brief Adds a collision to the a-star map
        /// @param position position in robotposition
        /// @param radius radius of the collision in mm
        void addCollisionCircle(RobotPosition const& position, double radius);

        /// @brief Adds a collision square to the a-star map
        /// @param position position in robotposition
        /// @param halfDiameter half of the diameter of the square of the collision in mm
        void addCollisionSquare(RobotPosition const& position, double halfDiameter);

        /// @brief Return the closest position in the grid which is not in an obstacle.
        /// @param desiredPosition Desired position
        /// @return Closest available cell*
        RobotPosition getNearestAvailablePosition(RobotPosition const& desiredPosition);

        void print(Logger *logger,
                   std::vector<RobotPosition> path,
                   RobotPosition currentPosition,
                   RobotPosition targetPosition) const;

        double getGridSize() const {return gridSize_;};
    private:
        double gridSize_ = 10.0; // Grid size, in mm.
};

#endif