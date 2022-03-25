/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/trajectory/ArcCircle.h"
#include "miam_utils/trajectory/Utilities.h"

#include <iostream>
#include <cmath>

namespace miam{
    namespace trajectory{
        ArcCircle::ArcCircle(RobotPosition const& startPoint,
                             double const& radius,
                             rotationside const& side,
                             double const& endAngle,
                             double const& startVelocity,
                             double const& endVelocity,
                             bool const& backward,
                             double maxVelocity,
                             double maxAcceleration):
             movingBackward_(1.0),
             radius_(std::abs(radius)),
             side_(side),
             endAngle_(endAngle),
             endVelocity_(endVelocity),
             maxVelocity_(maxVelocity),
             maxAcceleration_(maxAcceleration)
        {
            if (backward)
                movingBackward_ = -1.0;
            make(startPoint, startVelocity);
        }

        TrajectoryPoint ArcCircle::getCurrentPoint(double const& currentTime)
        {
            TrajectoryPoint output;
            output.position = circleCenter_;
            TrapezoidState state = trapezoid_.getState(currentTime);

            // Compute point along circle.
            output.position.x += radius_ * std::cos(circleCenter_.theta + motionSign_ * state.position);
            output.position.y += radius_ * std::sin(circleCenter_.theta + motionSign_ * state.position);
            output.position.theta = circleCenter_.theta + motionSign_ * M_PI_2 + motionSign_ * state.position;
            if(movingBackward_ == -1.0)
                output.position.theta -= M_PI  *motionSign_;
            // Compute linear and angular velocity
            output.linearVelocity = movingBackward_ * state.velocity * radius_;
            output.angularVelocity = motionSign_ * state.velocity;

            return output;
        }

        void ArcCircle::make(RobotPosition const& startPoint, double const& startVelocity)
        {
             motionSign_ = 1.0;

            // Compute position of circle center.
            // Get angle of robot wheel axis, swich sign based on direction.
            circleCenter_ = computeCircleCenter(startPoint, radius_, side_);

            // Compute angle of travel along the circle.
            double travelAngle = moduloTwoPi(endAngle_ - circleCenter_.theta);
            // Give it the correct sign, based on desired direction and rotation side.
            if(side_ == rotationside::RIGHT)
                motionSign_ = -1.0;
            motionSign_ = motionSign_ * movingBackward_;

            // Compute angle with right sign, modulo 2 Pi
            if(motionSign_ < 0)
            {
                if(travelAngle > 0)
                    travelAngle -=2 * M_PI;
            }
            else
            {
                if(travelAngle < 0)
                    travelAngle +=2 * M_PI;
            }

            // Compute trapezoid.
            double maxAngularVelocity = maxVelocity_ / (radius_ + config::robotWheelSpacing);
            double maxAngularAcceleration = maxAcceleration_ / (radius_ + config::robotWheelSpacing);
            trapezoid_ = Trapezoid(travelAngle, startVelocity, endVelocity_, maxAngularVelocity, maxAngularAcceleration);

            duration_ = trapezoid_.getDuration();
        }


        void ArcCircle::replanify(double const& replanificationTime)
        {
            RobotPosition startPoint = getCurrentPoint(replanificationTime).position;
            make(startPoint, 0.0);
        }
    }
}
