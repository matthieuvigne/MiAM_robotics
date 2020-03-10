/// \file trajectory/ThreeWheelsKinematics.hpp
/// \brief Implements the kinematics of an omnidirectional robot built with three omniwheels.
#ifndef MIAM_THREE_WHEELS_KINEMATICS_HPP
#define MIAM_THREE_WHEELS_KINEMATICS_HPP

#include "miam_utils/trajectory/RobotPosition.h"

namespace omni
{
    /// \brief Spatial velocity of the robot center (linear velocity in m/s, angular velocity in rad/s).
    struct BaseSpeed
    {
        BaseSpeed()
        : vx_(0.),
          vy_(0.),
          omega_(0.)
        {}

        BaseSpeed(
          double const& vx,
          double const& vy,
          double const& omega)
        : vx_(vx),
          vy_(vy),
          omega_(omega)
        {}

        double vx_;  ///< Linear velocity, m/s.
        double vy_;  ///< Linear velocity, m/s.
        double omega_;   ///< Angular velocity, rad/s.
    };

    /// \brief Angular velocity of each wheel, in rad/s.
    struct WheelSpeed
    {
        WheelSpeed()
        : w_{0, 0, 0}
        {}

        WheelSpeed(
          double const& wheel1,
          double const& wheel2,
          double const& wheel3)
        : w_{wheel1, wheel2, wheel3}
        {}

        double w_[3];  ///< Wheel velocity, rad/s
    };
    
    /// \brief Kinematics of the omnidirectional base.
    /// 
    /// \details This class handles conversion of velocities between the motor space (wheel angular
    ///          velocity) and the base space (cartesian linear velocity plus angular velocity).
    class ThreeWheelsKinematics 
    {
        public:
            /// \brief Constructor.
            ///
            /// \param[in] robotRadius Distance between a wheel and the center of the robot.
            /// \param[in] wheelRadius Wheel radius.
            ThreeWheelsKinematics(double const& robotRadius, double const& wheelRadius);

        public:
            /// \brief Forward kinematic model
            /// \details Convert from wheel speed to base speed.
            ///
            /// \param[in] wheelSpeed Input wheel speed.
            /// \return Corresponding base speed.
            BaseSpeed forwardKinematics(WheelSpeed const& wheelSpeed) const;

            /// \brief Inverse kinematic model
            /// \details Convert from base speed to wheel speed.
            ///
            /// \param[in] baseSpeed Input base speed.
            /// \return Corresponding wheel speed.
            WheelSpeed inverseKinematics(BaseSpeed const& baseSpeed) const;

        private:
            double robotRadius_; ///< Radius of the robot: distance from center of the robot to the wheel, in m.
            double wheelRadius_; ///< Radius of the wheel, in m.

        }; // class ThreeWheelsKinematics
    }
#endif // MIAM_THREE_WHEELS_KINEMATICS_HPP
