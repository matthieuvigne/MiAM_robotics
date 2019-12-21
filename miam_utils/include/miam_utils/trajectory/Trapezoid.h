/// \file trajectory/Trapezoid.h
/// \brief Define a generic velocity trapezoid, use to parametrize other curves.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MIAM_TRAJECTORY_TRAPEZOID
#define MIAM_TRAJECTORY_TRAPEZOID
    #include "MiAMEurobot/trajectory/Trajectory.h"

    namespace miam{
        namespace trajectory{
            /// \brief State along the trapezoid (position and velocity).
            struct TrapezoidState{
                double position; ///< Current position in trapezoid.
                double velocity; ///< Current velocity in trapezoid.

                /// \brief Default constructor.
                TrapezoidState():
                    position(0.0),
                    velocity(0.0)
                {}
            };


            /// \brief Define a velocity trapezoid path go to to a specif point.
            /// \details Given a distance, start and end velocity, performs a velocity trapezoid for going from
            ///          0 to the given distance. This is done with constraints on max velocity and acceleration:
            ///          in the case where the trajectory is unfeasible (too much difference between start and end
            ///          velocities), the best attemp (i.e. constant max acceleration) is made.
            class Trapezoid
            {
                public:
                    /// \brief Default constructor.
                    Trapezoid();

                    /// \brief Constructor.
                    ///
                    /// \param[in] distance Trapezoid length. Only absolute value is taken into account.
                    /// \param[in] startVelocity Start velocity, cropped to [-maxVelocity, maxVelocity].
                    /// \param[in] endVelocity End velocity, cropped to [0, maxVelocity].
                    /// \param[in] maxVelocity Max velocity along the trapezoid. Only absolute value is taken into account.
                    /// \param[in] maxAcceleration Max acceleration along the trapezoid. Only absolute value is taken into account.
                    Trapezoid(double const& distance,
                              double const& startVelocity,
                              double const& endVelocity,
                              double maxVelocity = config::maxWheelVelocity,
                              double maxAcceleration = config::maxWheelAcceleration);

                    /// \brief Get point along the trapezoid curve, at a given time.
                    ///
                    /// \param[in] currentTime Time, in s, since trapezoid following started.
                    /// \return State along the trapezoid.
                    TrapezoidState getState(double const& currentTime);

                    /// \brief Get trapezoid duration, in seconds.
                    /// \return Trapezoid duration.
                    double getDuration();

                private:
                    double duration_;     ///< Trapezoid duration.
                    double maxVelocity_;     ///< Maximum velocity.
                    double maxAcceleration_; ///< Maximum acceleration.
                    double startVelocity_; ///< Velocity at which the trajectory starts.
                    double length_; ///< Trapezoid length.
                    double timeToStopAccelerating_; ///< When to stop accelerating.
                    double timeToStartDecelerating_; ///< When to start decelerating.
                    double accelerationDistance_;    ///< Distance traveled during acceleration phase.
            };
        }
    }
#endif
