/// \file trajectory/Trajectory.h
/// \brief Definition of a trajectory, and associated elements.
///
/// \details What we call a trajectory is an object along wich we can servo the robot. It thus groups together the
///          notion of path (i.e. curve on the table, including robot angle), and of time parametrization (in order
///          to allow trajectory tracking in itself).
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MIAM_TRAJECTORY_TRAJECTORY
#define MIAM_TRAJECTORY_TRAJECTORY

    #include "RobotPosition.h"

    namespace miam{
        namespace trajectory{

            // Config variables, used for trajectory generation.
            namespace config
            {
                extern double maxWheelVelocity;     ///< Maximum wheel velocity along a trajectory, mm/s.
                extern double maxWheelAcceleration; ///< Maximum wheel acceleration of a trajectory, mm/s2.
                extern double robotWheelSpacing;     ///< Distance from wheel to robot center, in mm. Used to compute velocity of external wheel while along a curve.
            }

            /// \brief Set the velocity and dimension value of the config parameters.
            /// \details The config parameters are used as default parameters for trajectory generation.
            ///          This function enables chaning these parameters.
            ///
            /// \param[in] maxWheelVelocity Maximum wheel velocity, mm/s.
            /// \param[in] maxWheelAcceleration Maximum wheel acceleration, mm/s2.
            /// \param[in] robotWheelSpacing Distance from wheel to robot center, in mm.
            void setTrajectoryGenerationConfig(double const& maxWheelVelocity,
                                               double const& maxWheelAcceleration,
                                               double const& robotWheelSpacing);

            /// \brief A trajectory point, containing everything for servoing along this trajectory.
            struct TrajectoryPoint{
                RobotPosition position; ///< Trajectory point in the table.
                double linearVelocity; ///< Linear velocity along the trajectory, at the current point.
                double angularVelocity; ///< Angular velocity along the trajectory, at the current point.

                /// \brief Default constructor.
                TrajectoryPoint():
                position(),
                linearVelocity(0.0),
                angularVelocity(0.0)
                {}
            };

            class Trajectory
            {
                public:
                    Trajectory();

                    /// \brief Get trajectory point at current time.
                    ///
                    /// \param[in] currentTime Time relative to trajectory start, in seconds.
                    /// \return The current trajectory point.
                    virtual TrajectoryPoint getCurrentPoint(double const& currentTime) = 0;

                    /// \brief Get trajectory duration, in seconds.
                    /// \return Trajectory duration.
                    double getDuration();

                    /// \brief Get final point of the trajectory.
                    /// \return Position at duration_
                    TrajectoryPoint getEndPoint();

                    /// \brief Replanify the trajectory from the given time.
                    ///
                    /// \details Given a time t, replanify the trajectory to have a new trajectory
                    ///          starting at getCurrentPoint(t) with zero velocity at time t=0.
                    /// \param[in] replanificationTime Time at which we replanify, in seconds.
                    virtual void replanify(double const& replanificationTime) = 0;

                protected:
                    double duration_; ///< Trajectory duration.
            };
        }
    }
#endif
