/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/trajectory/Utilities.h"
#include "miam_utils/trajectory/StraightLine.h"
#include "miam_utils/trajectory/PointTurn.h"
#include "miam_utils/trajectory/ArcCircle.h"

#include <iostream>
#include <cmath>
#include <limits>

namespace miam{
    namespace trajectory{

        TrajectoryVector TrajectoryVector::operator+(const TrajectoryVector b) const
        {
            TrajectoryVector v = *this;
            v.insert(v.end(), b.begin(), b.end());
            return v;
        }

        double TrajectoryVector::getDuration() const
        {
            double duration = 0;
            for (auto traj : *this)
            {
                duration += traj->getDuration();
            }
            return duration;
        }

        TrajectoryPoint TrajectoryVector::getCurrentPoint(double const& currentTime) const
        {
            if (MIAM_DEBUG_TRAJECTORY_TYPE)
                std::cout << "TrajectoryVector::getCurrentPoint " << currentTime << std::endl;

            if (this->empty())
                return TrajectoryPoint();

            double sum_time = 0;
            for (long unsigned int i = 0; i < this->size(); i++) {
                double traj_time = this->at(i)->getDuration();
                if (sum_time + traj_time > currentTime) {
                    return this->at(i)->getCurrentPoint(currentTime - sum_time);
                }
                sum_time += traj_time;
            }
            return this->back()->getEndPoint();
        };


        TrajectoryPoint TrajectoryVector::getEndPoint() const
        {
            if (this->empty())
                return TrajectoryPoint();
            return this->back()->getEndPoint();
        }


        RobotPosition computeCircleCenter(RobotPosition const& startingPosition, double const& radius, rotationside const& side)
        {
            RobotPosition circleCenter;
            double centerAngle = startingPosition.theta + M_PI / 2.0;
            if(side == rotationside::RIGHT)
                centerAngle += M_PI;
            circleCenter.x = startingPosition.x + std::abs(radius) * std::cos(centerAngle);
            circleCenter.y = startingPosition.y + std::abs(radius) * std::sin(centerAngle);
            // Absolue angle of segment [center, startPoint]
            circleCenter.theta = centerAngle - M_PI;
            return circleCenter;
        }


        double computeShortestAngle(RobotPosition startPoint, RobotPosition endPoint)
        {
            // Compute line angle.
            if (std::abs(endPoint.y - startPoint.y) < std::numeric_limits<double>::epsilon() && std::numeric_limits<double>::epsilon() < 0)
            {
                return 0;
            }

            double angle = std::atan2(endPoint.y - startPoint.y, endPoint.x - startPoint.x);
            // Return value in ]-pi, pi] of the original angle.
            return moduloTwoPi(angle - startPoint.theta);
        }


        double distance(RobotPosition const& first, RobotPosition const& second)
        {
            return (first - second).norm();
        }

        double moduloTwoPi(double angle)
        {
            angle = std::fmod(angle, 2 * M_PI);
            if (angle <= -M_PI)
                angle += 2 * M_PI;
            if (angle > M_PI)
                angle -= 2 * M_PI;
            return angle;
        }

        TrajectoryVector computeTrajectoryStraightLineToPoint(TrajectoryConfig const& config,
                                                              RobotPosition const& startPosition,
                                                              RobotPosition const& endPosition,
                                                              double const& endVelocity,
                                                              flags const& trajectoryFlags)
        {
            TrajectoryVector vector;
            std::shared_ptr<StraightLine> line(new StraightLine(config, startPosition, endPosition, 0.0, endVelocity, trajectoryFlags & flags::BACKWARD));

            // Get angle from straight line as rotation target.
            vector.push_back(std::shared_ptr<Trajectory>(new PointTurn(config, startPosition, line->getAngle())));
            vector.push_back(std::shared_ptr<Trajectory>(line));
            if (!(trajectoryFlags & flags::IGNORE_END_ANGLE))
            {
                vector.push_back(std::shared_ptr<Trajectory>(new PointTurn(config, vector.getEndPoint().position, endPosition.theta)));
            }
            return vector;
        }

        TrajectoryVector computeTrajectoryRoundedCorner(TrajectoryConfig const& config,
                                                        std::vector<RobotPosition> const& positions,
                                                        double radius,
                                                        double transitionVelocityFactor,
                                                        flags const& trajectoryFlags)
        {
            TrajectoryVector trajectories;
            if(positions.size() < 2)
                return trajectories;
            // Compute the transition angular velocity.
            double factor = transitionVelocityFactor;
            if(factor < 0.0)
                factor = 0.0;
            else if(factor > 1.0)
                factor = 1.0;

            double transitionLinearVelocity = factor * config.maxWheelVelocity;
            double transitionAngularVelocity = transitionLinearVelocity / (std::abs(radius) + config.robotWheelSpacing);

            // Compute first rotation to be aligned with second point.
            TrajectoryVector straightLine = computeTrajectoryStraightLineToPoint(
                config,
                positions.at(0),
                positions.at(1),
                transitionLinearVelocity,
                static_cast<flags>(trajectoryFlags & flags::BACKWARD));

            trajectories.push_back(straightLine[0]);

            // For each remaining pair of points, computed the line and rounded corner to go there.
            for(uint i = 1; i < positions.size() - 1; i++)
            {
                RobotPosition const startPoint = trajectories.back()->getEndPoint().position;
                RobotPosition const roundedCornerPoint = positions.at(i);
                RobotPosition const endPoint = positions.at(i + 1);

                // Find position of the center of the circle.
                // Get vectors going from the corner to both points.
                RobotPosition firstVector = startPoint - roundedCornerPoint;
                double const firstNorm = firstVector.norm();
                firstVector.normalize();
                RobotPosition secondVector = endPoint - roundedCornerPoint;
                double const secondNorm = secondVector.norm();
                secondVector.normalize();

                // Find angle between both vectors.
                double const angle = std::acos(firstVector.dot(secondVector));

                // If points are aligned, just skip it.
                if (M_PI - angle < 1e-3)
                    continue;

                // Get distance from roundedCornerPoint to center along each vector, reducing the radius if it is too large.
                double const coefficient = 1 / std::tan(angle / 2.0);
                double const circleRadius = std::min(radius, std::min(firstNorm, secondNorm) / coefficient * 0.99);

                // Compute point where the circle intersects both trajectories: get the first point and the
                // angle.
                RobotPosition circleIntersection = roundedCornerPoint + circleRadius * coefficient * firstVector;
                // Put back right angle.
                circleIntersection.theta = startPoint.theta;

                // Compute direction of the circle, based on cross-product.
                rotationside side = rotationside::LEFT;
                if(firstVector.cross(secondVector) > 0.0)
                    side = rotationside::RIGHT;
                if (trajectoryFlags & flags::BACKWARD)
                {
                    if (side == rotationside::RIGHT)
                        side = rotationside::LEFT;
                    else
                        side = rotationside::RIGHT;
                }

                // Compute end angle: minus the angle of secondVector.
                double endAngle = std::atan2(secondVector.y, secondVector.x);
                if(side == rotationside::LEFT)
                    endAngle -= M_PI_2;
                else
                    endAngle += M_PI_2;

                if (trajectoryFlags & flags::BACKWARD)
                    endAngle += M_PI;

                // Compute trajectory.
                std::shared_ptr<StraightLine> line(new StraightLine(config, startPoint, circleIntersection, (i == 1 ? 0.0 : transitionLinearVelocity), transitionLinearVelocity, trajectoryFlags & flags::BACKWARD));
                trajectories.push_back(line);
                std::shared_ptr<ArcCircle> circle(new ArcCircle(config, circleIntersection, circleRadius, side, endAngle, transitionAngularVelocity, transitionAngularVelocity, trajectoryFlags & flags::BACKWARD));
                trajectories.push_back(circle);
            }
            // Append final straight line.
            RobotPosition currentPoint = trajectories.back()->getEndPoint().position;
            RobotPosition endPoint = positions.back();
            std::shared_ptr<StraightLine> line(new StraightLine(config, currentPoint, endPoint, (positions.size() == 2 ? 0 : transitionLinearVelocity), 0.0, trajectoryFlags & flags::BACKWARD));
            trajectories.push_back(line);

            // Add final rotation if needed
            if (!(trajectoryFlags & flags::IGNORE_END_ANGLE))
            {
                trajectories.push_back(std::shared_ptr<Trajectory>(new PointTurn(config, trajectories.getEndPoint().position, endPoint.theta)));
            }

            return trajectories;
        }

        TrajectoryVector computeTrajectoryStraightLine(TrajectoryConfig const& config, RobotPosition & position, double const& distance)
        {
            RobotPosition endPosition = position + RobotPosition(distance, 0.0, 0.0).rotate(position.theta);
            endPosition.theta = position.theta;

            TrajectoryVector vector;
            std::shared_ptr<StraightLine> line(new StraightLine(config, position, endPosition, 0.0, 0.0, (distance < 0)));

            vector.push_back(std::shared_ptr<Trajectory>(line));
            position = endPosition;
            return vector;
        }
    }
}
