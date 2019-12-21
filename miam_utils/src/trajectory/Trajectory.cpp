/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/trajectory/Trajectory.h"

#include <cmath>

namespace miam{
    namespace trajectory{

        // Default trajectory config.
        namespace config
        {
            double maxWheelVelocity = 300.0;
            double maxWheelAcceleration = 300.0;
            double robotWheelSpacing = 100.0;
        }


        void setTrajectoryGenerationConfig(double const& maxWheelVelocity,
                                           double const& maxWheelAcceleration,
                                           double const& robotWheelSpacing)
        {
            config::maxWheelVelocity = std::abs(maxWheelVelocity);
            config::maxWheelAcceleration = std::abs(maxWheelAcceleration);
            config::robotWheelSpacing = std::abs(robotWheelSpacing);
        }

        Trajectory::Trajectory()
        {
            duration_ = 0.0;
        }

        double Trajectory::getDuration()
        {
            return duration_;
        }

        TrajectoryPoint Trajectory::getEndPoint()
        {
            return getCurrentPoint(getDuration());
        }
    }
}
