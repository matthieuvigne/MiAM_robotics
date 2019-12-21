/// \file trajectory/SampledTrajectory.h
/// \brief A trajectory defined by a set of TrajectoryPoints & linear interpolation between points.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MIAM_TRAJECTORY_SAMPLED_TRAJECTORY
#define MIAM_TRAJECTORY_SAMPLED_TRAJECTORY

    #include "MiAMEurobot/trajectory/Trajectory.h"
    #include <vector>

    namespace miam{
        namespace trajectory{
            class SampledTrajectory: public Trajectory
            {
                public:
                    /// \brief Constructor.
                    ///
                    /// \param[in] sampled_trajectory Vector of trajectory waypoints. Suppose they are equally sampled in time.
                    /// \param[in] start_time Time at the first point of the trajectory.
                    /// \param[in] end_time Time at the last point of the trajectory.
                    SampledTrajectory(
                        std::vector<TrajectoryPoint > sampledTrajectory,
                        double duration
                        );

                    TrajectoryPoint getCurrentPoint(double const& currentTime);

                private:
                    std::vector<TrajectoryPoint > sampledTrajectory_; ///< Vector of trajectory waypoints.
            };
        }
    }
#endif
