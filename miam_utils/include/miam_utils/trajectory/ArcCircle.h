/// \file trajectory/ArcCircle.h
/// \brief A circular arc trajectory.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MIAM_TRAJECTORY_ARC_CIRCLE
#define MIAM_TRAJECTORY_ARC_CIRCLE

    #include "MiAMEurobot/trajectory/Trajectory.h"
    #include "MiAMEurobot/trajectory/Trapezoid.h"

    namespace miam{
        namespace trajectory{
            enum rotationside{
                RIGHT,
                LEFT
            };

            class ArcCircle: public Trajectory
            {
                public:
                    /// \brief Constructor.
                    /// \details Specify the trajectory, giving robot starting position, circle radius, rotation
                    ///          side (used to define if the circle is on the right or on the left of the robot),
                    ///          and final angle along the arc. The final angle is the angle between -pi and
                    ///          pi of the end point along the circle, and not the robot angle.
                    ///
                    /// \param[in] startPoint Strating point.
                    /// \param[in] radius Circle radius.
                    /// \param[in] side  On which side the trajectory is generated.
                    /// \param[in] endAngle Position of endpoint along the circle.
                    /// \param[in] startVelocity Start velocity.
                    /// \param[in] endVelocity Desired end velocity.
                    /// \param[in] backward If robot should move backward along the trajectory.
                    /// \param[in] maxVelocity Max velocity. Only absolute value is taken into account.
                    /// \param[in] maxAcceleration Max acceleration. Only absolute value is taken into account.
                    ArcCircle(RobotPosition const& startPoint,
                             double const& radius,
                             rotationside const& side,
                             double const& endAngle,
                             double const& startVelocity=0.0,
                             double const& endVelocity=0.0,
                             bool const& backward = false,
                             double maxVelocity=config::maxWheelVelocity,
                             double maxAcceleration=config::maxWheelAcceleration);

                    TrajectoryPoint getCurrentPoint(double const& currentTime);

                    void replanify(double const& replanificationTime);

                private:
                    void make(RobotPosition const& startPoint, double const& startVelocity); ///< Build (or rebuild) the trajectory.

                    int movingBackward_; ///< 1 or -1, to indicate if robot is moving forward or backward.
                    int motionSign_; ///< 1 or -1, to indicate if rotation is direct or indirect.
                    double radius_; ///< Circle radius.
                    RobotPosition circleCenter_; ///< Center of the circle.
                    Trapezoid trapezoid_; ///< Velocity trapezoid.

                    rotationside side_; ///< Rotation direction.
                    double endAngle_; ///< End angle.
                    double endVelocity_; ///< End velocity.
                    double maxVelocity_; ///< Maximum velocity.
                    double maxAcceleration_; ///< Maximum acceleration.
            };
        }
    }
#endif
