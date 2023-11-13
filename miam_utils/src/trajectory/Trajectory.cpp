/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/trajectory/Trajectory.h"

#include <cmath>

namespace miam{
    namespace trajectory{

        TrajectoryPoint::TrajectoryPoint():
            position(),
            linearVelocity(0.0),
            angularVelocity(0.0)
            {}

        Trajectory::Trajectory(TrajectoryConfig const& config):
            isAvoidanceTrajectory_(false),
            needReplanning_(false),
            duration_(0.0),
            config_(config),
            avoidanceEnabled_(true)
        {
        }

        double Trajectory::getDuration()
        {
            return duration_;
        }

        TrajectoryPoint Trajectory::getEndPoint()
        {
            return getCurrentPoint(getDuration());
        }

        std::ostream& operator<<(std::ostream& os, const TrajectoryPoint& p)
        {
            os << p.position;
            os << " v: " << p.linearVelocity;
            os << " w: " << p.angularVelocity;
            return os;
        }
    }
}
