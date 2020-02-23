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
            : angular_velocity_(0.),
              linear_velocity_x_(0.),
              linear_velocity_y_(0.)
            {}

            BaseSpeed(
              double const& angular_velocity,
              double const& linear_velocity_x,
              double const& linear_velocity_y)
            : angular_velocity_(angular_velocity),
              linear_velocity_x_(linear_velocity_x),
              linear_velocity_y_(linear_velocity_y)
            {}

            double angular_velocity_;   ///< Angular velocity, rad/s.
            double linear_velocity_x_;  ///< Linear velocity, m/s.
            double linear_velocity_y_;  ///< Linear velocity, m/s.
        };

        /// \brief Angular velocity of each wheel, in rad/s.
        struct WheelSpeed
        {
            WheelSpeed()
            : wheelSpeed_{0, 0, 0}
            {}

            WheelSpeed(
              double const& wheel1,
              double const& wheel2,
              double const& wheel3)
            : wheelSpeed_{wheel1, wheel2, wheel3}
            {}

            double wheelSpeed_[3];  ///< Wheel velocity, rad/s
        };
        
        /// \brief Kinematics of the omnidirectional base.
        class ThreeWheelsKinematics 
        {
            public:
              ThreeWheelsKinematics() {}; // = delete
              ThreeWheelsKinematics(double const& length, double const& wheelRadius);

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
              double length_; ///< Radius of the robot: distance from center of the robot to the wheel, in m.
              double wheelRadius_; ///< Radius of the wheel, in m.

            }; // class ThreeWheelsKinematics
        }
#endif // MIAM_THREE_WHEELS_KINEMATICS_HPP
