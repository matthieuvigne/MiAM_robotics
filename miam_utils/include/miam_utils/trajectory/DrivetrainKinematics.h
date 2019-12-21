/// \file trajectory/DrivetrainKinematics.h
/// \brief Implements the kinematics of a simple two-wheel drivetrain with optional encoders.
/// \details The position of the drivetrain is defined by a cartesian coordinate and an angle, grouped into
///          a RobotPosition object. It's velocity contains two composants, a linear and an angular velocity.
///          This class implements the direct (motor velocity to drivetrain velocity) and indirect (linear/angular
///          velocity to motor velocity) kinematics, as well
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MIAM_DRIVETRAIN_KINEMATICS
#define MIAM_DRIVETRAIN_KINEMATICS

    #include "MiAMEurobot/trajectory/RobotPosition.h"

    /// \brief Class to represent base speed.
    class BaseSpeed
    {
        public:
            BaseSpeed(double const& linearIn, double const& angularIn):
                linear(linearIn),
                angular(angularIn)
            {}
            BaseSpeed():
                linear(0.0),
                angular(0.0)
            {}
            double linear; ///< Base linear velocity, by convention in mm.
            double angular; ///< Base angular velocity, by convention in rad/s.
    };

    /// \brief Class to represent wheel speed.
    class WheelSpeed
    {
        public:
            WheelSpeed(double const& rightIn, double const& leftIn):
                right(rightIn),
                left(leftIn)
            {}
            WheelSpeed():
                right(0.0),
                left(0.0)
            {}
            double right; ///< Right wheel speed, by convention in rad/s.
            double left; ///< Left wheel speed, by convention in rad/s.
    };

    /// \brief Kinematics of a two-wheel drivetrain with optional encoders.
    /// \details The position of the drivetrain is defined by a cartesian coordinate and an angle, grouped into
    ///          a RobotPosition object. It's velocity contains two composants, a linear and an angular velocity.
    ///          This class implements the direct (wheel speed to base speed) and indirect (base velocity to wheel
    ///          velocity) kinematics, as well as integration functions (from velocity to position).
    class DrivetrainKinematics
    {
        public:
            /// \brief Default constructor.
            DrivetrainKinematics();

            /// \brief Constructor, defining the parameters of the drivetrain.
            ///
            /// \param[in] motorWheelRadius Radius of the motor wheel, in mm.
            /// \param[in] motorWheelSpacing Distance between the center of the robot and the motor wheel, in mm.
            /// \param[in] encoderWheelRadius Radius of the encoder wheel, in mm.
            /// \param[in] encoderWheelSpacing Distance between the center of the robot and the encoder wheel, in mm.
            DrivetrainKinematics(double const& motorWheelRadiusIn,
                                 double const& motorWheelSpacingIn,
                                 double const& encoderWheelRadiusIn,
                                 double const& encoderWheelSpacingIn);

            /// \brief Forward kinematics, i.e. convert wheel speed to base speed
            /// \details By default, wheelAngularSpeedIn is intepreted as the angular velocity of the motor wheels:
            ///          set useEncoders to compute kinematics for the encoders.
            ///
            /// \param[in] wheelAngularSpeedIn Wheel angular velocity (in rad/s).
            /// \param[in] useEncoders If true, compute kinematics using the encoders.
            ///                        Otherwise, use the motor parameters. Default: false
            ///
            /// \return Speed of the base.
            BaseSpeed forwardKinematics(WheelSpeed const& wheelSpeedIn, bool const& useEncoders = false);

            /// \brief Inverse kinematics, i.e. convert base speed to wheel speed
            ///
            /// \param[in] baseSpeedIn Base velocity
            ///
            /// \return Motor wheel speed.
            WheelSpeed inverseKinematics(BaseSpeed const& baseSpeedIn);

            /// \brief Integrate the (infinitesimal) wheel displacement into a robot position.
            /// \details By default, wheelSpeedIn is intepreted as the angular displacement of the encoder wheels:
            ///          set useEncoders to compute kinematics for the motors.
            ///
            /// \param[in] wheelSpeedIn Input wheel displacement (in rad).
            /// \param[in] positionInOut The position from which to integrate ; contains the integration result.
            /// \param[in] useEncoders If true, compute kinematics using the encoders.
            ///                        Otherwise, use the motor parameters. Default: true
            void integratePosition(WheelSpeed const& wheelSpeedIn, miam::RobotPosition & positionInOut, bool const& useEncoders = true);

        private:
            double motorWheelRadius_; ///< Radius of the motor wheel, in mm.
            double motorWheelSpacing_; ///< Distance between the center of the robot and the motor wheel, in mm.
            double encoderWheelRadius_; ///< Radius of the encoder wheel, in mm.
            double encoderWheelSpacing_; ///< Distance between the center of the robot and the encoder wheel, in mm.
    };
#endif
