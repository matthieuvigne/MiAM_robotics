#include <iostream>
#include "AStar/AStar.hpp"

#include <miam_utils/trajectory/Trajectory.h>
#include <miam_utils/trajectory/ArcCircle.h>
#include <miam_utils/trajectory/StraightLine.h>
#include <miam_utils/trajectory/PointTurn.h>
#include <miam_utils/trajectory/Utilities.h>

#include<iostream>
#include<fstream>

#include "gnuplot-iostream.h"

#include "mpc_solver_lib/mpc_solver.hpp"

using namespace miam;
using namespace std;

#define MIAM_ASTAR_RESOLUTION_MM 100
#define MIAM_ASTAR_GRID_SIZE_X 20
#define MIAM_ASTAR_GRID_SIZE_Y 30

RobotPosition astarWaypointToRobotPosition(AStar::Vec2i astarWaypoint) {
    return RobotPosition(
        astarWaypoint.x * MIAM_ASTAR_RESOLUTION_MM + MIAM_ASTAR_RESOLUTION_MM / 2, 
        astarWaypoint.y * MIAM_ASTAR_RESOLUTION_MM + MIAM_ASTAR_RESOLUTION_MM / 2,
        0
        );
};

int main()  
{
    AStar::Generator generator;

    generator.setWorldSize({MIAM_ASTAR_GRID_SIZE_X, MIAM_ASTAR_GRID_SIZE_Y});
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    // obstacles table
    // bordure exterieure
    for (int i = 0; i < MIAM_ASTAR_GRID_SIZE_X; i++) 
    {
        generator.addCollision({i, 0});
        generator.addCollision({i, MIAM_ASTAR_GRID_SIZE_Y-1});
    }
    for (int j = 0; j < MIAM_ASTAR_GRID_SIZE_Y; j++) 
    {
        generator.addCollision({0, j});
        generator.addCollision({MIAM_ASTAR_GRID_SIZE_Y-1, j});
    }

    // distributeurs de cerises
    for (int j = 0; j < 4; j++) 
    {
        generator.addCollision({9, j});
        generator.addCollision({10, j});
    }
    for (int j = MIAM_ASTAR_GRID_SIZE_Y-1-4; j < MIAM_ASTAR_GRID_SIZE_Y; j++) 
    {
        generator.addCollision({9, j});
        generator.addCollision({10, j});
    }

    // add other obstacles here
    // TODO

    std::cout << "Generate path ... \n";
    auto path = generator.findPath({2, 3}, {17, 2});

    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }

    // convertir en robotposition
    std::vector<RobotPosition> positions;
    RobotPosition targetPosition;

    for(auto coordinate = path.rbegin(); coordinate != path.rend(); ++coordinate) 
    {
        targetPosition = astarWaypointToRobotPosition(*coordinate);
        cout << targetPosition << endl;
        positions.push_back(targetPosition);
    }

    std::vector<TrajectoryPoint> solvedTraj = solveTrajectoryFromWaypoints(positions);

    for (auto& tp : solvedTraj) {
        std::cout << "solved: " <<  
            tp.position.x << "\t" << 
            tp.position.y << "\t" << 
            tp.position.theta << "\t" << 
            tp.linearVelocity << "\t" << 
            tp.angularVelocity << std::endl;
    }
}
