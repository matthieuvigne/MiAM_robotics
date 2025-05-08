/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.

    Code originally from https://github.com/daancode/a-star, with custom modifications.
*/
#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>
#include <eigen3/Eigen/Core>

#include "miam_utils/Map.h"

namespace AStar
{
    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(MapCoord, MapCoord)>;
    using CoordinateList = std::vector<MapCoord>;

    struct Node
    {
        uint G, H;
        MapCoord coordinates;
        Node *parent;

        Node(MapCoord coord_, Node *parent_ = nullptr);
        uint getScore();
    };

    using NodeSet = std::vector<Node*>;

    class Generator
    {
    public:
        Generator();
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList findPath(Map const& obstacleMap, MapCoord source_, MapCoord target_) const;

    private:
        Node* findNodeOnList(NodeSet& nodes_, MapCoord coordinates_) const;
        void releaseNodes(NodeSet& nodes_) const;

        HeuristicFunction heuristic;
        CoordinateList direction;
        uint directions;
    };

    class Heuristic
    {
        static MapCoord getDelta(MapCoord source_, MapCoord target_);

    public:
        static uint manhattan(MapCoord source_, MapCoord target_);
        static uint euclidean(MapCoord source_, MapCoord target_);
        static uint octagonal(MapCoord source_, MapCoord target_);
    };
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
