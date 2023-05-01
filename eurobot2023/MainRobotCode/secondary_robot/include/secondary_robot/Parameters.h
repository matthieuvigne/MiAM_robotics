/// \file Parameters.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef SECONDARY_ROBOT_PARAMETERS_H
    #define SECONDARY_ROBOT_PARAMETERS_H
        #include <math.h>

        #include "common/RobotParameters.h"

        namespace secondary_robot{
            inline RobotParameters generateParams()
            {
                RobotParameters param;
                param.name = "SecondaryRobot";

                param.wheelRadius = 55.4; ///< Wheel radius, in mm - identified during open loop experiments.

                // si le robot tourne trop, il faut diminuer wheel spacing
                param.wheelSpacing = 85.0; ///< Wheel spacing from robot center, in mm - identified during open loop experiments.
                param.encoderWheelRadius = 26.3289; ///< Radius of encoder wheels, in mm.
                param.encoderWheelSpacing = 114.5; ///< Encoder wheel spacing from robot center, in mm.

                param.maxWheelSpeed = 600; ///< Maximum wheel speed, in mm/s.
                param.maxWheelAcceleration = 1200; ///< Maximum wheel acceleration, in mm/s^2.

                param.maxWheelSpeedTrajectory = 300; ///< Maximum wheel speed, in mm/s, for trajectory generation.
                param.maxWheelAccelerationTrajectory = 600; ///< Maximum wheel acceleration, in mm/s^2, for trajectory generation.

                // Motion servoing gains
                param.linearKp = 10.0; // previously 3.5
                param.linearKd = 0.01;
                param.linearKi = 0.01;

                param.transverseKp = 0.005;

                param.rotationKp = 10.0;
                param.rotationKd = 0.01;
                param.rotationKi = 0.0;


                // Chassis physical dimensions
                param.CHASSIS_FRONT = 151.0;
                param.CHASSIS_BACK = 95.0;
                param.CHASSIS_WIDTH = 125.0;

                param.rightMotorId = 2;
                param.leftMotorId = 1;
                param.rightMotorDirection = 1;
                param.leftMotorDirection = -1;

                param.rightEncoderId = 1;
                param.leftEncoderId = 0;
                param.rightEncoderDirection = 1;
                param.leftEncoderDirection = -1;

                param.lidarOffset = -M_PI_4;

                return param;
            }
        }
 #endif
