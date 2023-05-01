/// \file Parameters.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef ROBOT_PARAMETERS_H
    #define ROBOT_PARAMETERS_H
        #include <math.h>

        #include <miam_utils/trajectory/Trajectory.h>

        // Parameters defining a robot
        struct RobotParameters {
            std::string name;
            double wheelRadius; ///< Wheel radius, in mm
            double wheelSpacing; ///< Wheel spacing from robot center, in mm
            double encoderWheelRadius;  ///< Radius of encoder wheels, in mm.
            double encoderWheelSpacing;  ///< Spacing between both encoder wheels, in mm.

            double maxWheelSpeed; ///< Maximum wheel speed, in mm/s.
            double maxWheelAcceleration; ///< Maximum wheel acceleration, in mm/s^2.

            double maxWheelSpeedTrajectory; ///< Maximum wheel speed, in mm/s, for trajectory generation.
            double maxWheelAccelerationTrajectory; ///< Maximum wheel acceleration, in mm/s^2, for trajectory generation.

            double linearKp; // Servoing gains: translation PID
            double linearKd;
            double linearKi;
            double transverseKp;  // Servoing gains: transverse offset
            double rotationKp;  // Servoing gains: rotation PID
            double rotationKd;
            double rotationKi;

            // Chassis physical dimensions
            double CHASSIS_FRONT;
            double CHASSIS_BACK;
            double CHASSIS_WIDTH;
            double SUCTION_CENTER;

            inline miam::trajectory::TrajectoryConfig getTrajConf()
            {
                miam::trajectory::TrajectoryConfig c;
                c.maxWheelVelocity = maxWheelSpeedTrajectory;
                c.maxWheelAcceleration = maxWheelAccelerationTrajectory;
                c.robotWheelSpacing = wheelSpacing;

                return c;
            };

            int rightMotorId;
            int leftMotorId;
            int rightMotorDirection = 1; // Invert motor if needed: positive is moving forward.
            int leftMotorDirection = -1; // Invert motor if needed: positive is moving forward.

            int rightEncoderId;
            int leftEncoderId;
            int rightEncoderDirection = 1; // Invert encoder if needed: positive is moving forward.
            int leftEncoderDirection = -1; // Invert encoder if needed: positive is moving forward.

            double lidarOffset = M_PI / 4.0;
        };

 #endif
