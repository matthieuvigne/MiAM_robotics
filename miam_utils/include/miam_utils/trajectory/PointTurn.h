/// \file trajectory/PointTurn.h
/// \brief In-place rotation of the robot.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MIAM_TRAJECTORY_POINT_TURN
#define MIAM_TRAJECTORY_POINT_TURN

    #include "miam_utils/trajectory/Trajectory.h"
    #include "miam_utils/trajectory/Trapezoid.h"

    namespace miam{
        namespace trajectory{
            class PointTurn: public Trajectory
            {
                public:
                    /// \brief Constructor.
                    /// \details Note: only zero-velocity transitions is supported, due to the singular configuration
                    ///          of this trajectory.
                    ///
                    /// \param[in] config Trajectory configuration (wheel spacing, max speed and acceleration)
                    /// \param[in] startPoint Trajectory starting point.
                    /// \param[in] endAngle Ending angle - it will be taken modulo 2 pi.
                    /// \param[in] maxVelocity Max wheel velocity. Only absolute value is taken into account.
                    /// \param[in] maxAcceleration Max acceleration. Only absolute value is taken into account.
                    PointTurn(TrajectoryConfig const& config,
                              RobotPosition const& startPoint,
                              double const& endAngle);

                    TrajectoryPoint getCurrentPoint(double const& currentTime);

                    void replanify(double const& replanificationTime);
                private:
                    void make(RobotPosition const& startPoint); ///< Build (or rebuild) the trajectory.

                    RobotPosition startPoint_; ///< Point where the trajectory started.
                    int motionSign_; ///< 1 or -1, to indicate direction of motion (trapezoid is always positive).
                    Trapezoid trapezoid_; ///< Velocity trapezoid.

                    double endAngle_;     ///< End angle.
            };
        }
    }
#endif
