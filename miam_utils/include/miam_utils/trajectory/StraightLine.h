/// \file trajectory/StraightLine.h
/// \brief A trajectory going in a straight line from two points, along a velocity trapezoid.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MIAM_TRAJECTORY_STRAIGHT_LINE
#define MIAM_TRAJECTORY_STRAIGHT_LINE

    #include "MiAMEurobot/trajectory/Trajectory.h"
    #include "MiAMEurobot/trajectory/Trapezoid.h"

    namespace miam{
        namespace trajectory{
            class StraightLine: public Trajectory
            {
                public:
                    /// \brief Constructor.
                    /// \details Robot angle along straight line is the angle of the line, computed to be the closest
                    ///          (modulto 2 pi) to the startPoint angle.
                    ///
                    /// \param[in] startPoint Strating point.
                    /// \param[in] endPoint Ending point, only x and y are taken into account.
                    /// \param[in] startVelocity Start velocity.
                    /// \param[in] endVelocity Desired end velocity.
                    /// \param[in] backward If robot should move backward along the straight line.
                    /// \param[in] maxVelocity Max velocity. Only absolute value is taken into account.
                    /// \param[in] maxAcceleration Max acceleration. Only absolute value is taken into account.
                    StraightLine(RobotPosition const& startPoint,
                                 RobotPosition const& endPoint,
                                 double const& startVelocity = 0.0,
                                 double const& endVelocity = 0.0,
                                 bool const& backward = false,
                                 double maxVelocity=config::maxWheelVelocity,
                                 double maxAcceleration=config::maxWheelAcceleration);

                    TrajectoryPoint getCurrentPoint(double const& currentTime);

                    void replanify(double const& replanificationTime);

                    /// \brief Return line angle.
                    ///
                    /// \return Line angle
                    double getAngle();
                private:
                    void make(RobotPosition const& startPoint, double const& startVelocity); ///< Build (or rebuild) the trajectory.

                    RobotPosition startPoint_; ///< Point where the trajectory started.
                    int motionSign_; ///< 1 or -1, to indicate direction of motion.
                    Trapezoid trapezoid_; ///< Velocity trapezoid.

                    RobotPosition endPoint_; ///< End position.
                    double endVelocity_; ///< End velocity.
                    bool backward_;     ///< True if going backward.
                    double maxVelocity_; ///< Maximum velocity.
                    double maxAcceleration_; ///< Maximum acceleration.
            };
        }
    }
#endif
