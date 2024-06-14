// Code originally from https://github.com/daancode/a-star, with custom modifications.

#include "miam_utils/AStar.hpp"
#include <algorithm>
#include <math.h>

using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

double AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setWorldSize({0, 0});
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
    obstacleMap_ = Eigen::MatrixXi::Zero(worldSize.x, worldSize.y);
    parentCoordinateX_ = Eigen::MatrixXi::Constant(worldSize.x, worldSize.y, -1);
    parentCoordinateY_ = Eigen::MatrixXi::Constant(worldSize.x, worldSize.y, -1);
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(Vec2i coordinates_)
{
    int const x = std::min(worldSize.x - 1, std::max(0, coordinates_.x));
    int const y = std::min(worldSize.y - 1, std::max(0, coordinates_.y));
    obstacleMap_(x, y) = 1;
}

AStar::Vec2i AStar::Generator::getWorldSize()
{
    return worldSize;
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    int const x = std::min(worldSize.x - 1, std::max(0, coordinates_.x));
    int const y = std::min(worldSize.y - 1, std::max(0, coordinates_.y));
    obstacleMap_(x, y) = 0;
}

void AStar::Generator::clearCollisions()
{
    obstacleMap_.setZero();
}

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    openSet.reserve(100);
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));

    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        current = *current_it;

        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        }

        if (current->coordinates == target_) {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        for (uint i = 0; i < directions; ++i) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            double totalCost = current->G + ((i < 4) ? 1 : 1.01 * std::sqrt(2));

            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            }

            if (current->parent != nullptr &&
                hasLineOfSight(newCoordinates, current->parent->coordinates))
            {
                double c = std::sqrt((newCoordinates.x - current->parent->coordinates.x) * (newCoordinates.x - current->parent->coordinates.x) + (newCoordinates.y - current->parent->coordinates.y) * (newCoordinates.y - current->parent->coordinates.y));

                double parentCost = 0.9999999 * current->parent->G + c;
                if (parentCost < successor->G)
                {
                    successor->parent = current->parent;
                    successor->G = parentCost;
                }
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y)
        return true;
    return obstacleMap_(coordinates_.x, coordinates_.y) == 1;
}

bool AStar::Generator::hasLineOfSight(Vec2i const& start, Vec2i const&  end)
{
    int const dx = std::abs(end.x - start.x);
    int const dy = -std::abs(end.y - start.y);

    int const sx = (end.x > start.x ? 1 : -1);
    int const sy = (end.y > start.y ? 1 : -1);

    Vec2i current(start);
    int e = dx + dy;
    while (true)
    {
        if (detectCollision(current))
            return false;
        if (current.x == end.x && current.y == end.y)
            return true;

        int e2 = 2 * e;
        if (e2 >= dy)
        {
            if (current.x == end.x)
                return true;
            e += dy;
            current.x += sx;
        }
        if (e2 <= dx)
        {
            if (current.y == end.y)
                return true;
            e += dx;
            current.y += sy;
        }
    }
    return true;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
