/// \file Parameters.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef MAIN_ROBOT_PARAMETERS_H
    #define MAIN_ROBOT_PARAMETERS_H
        #include <math.h>

        #include "common/RobotParameters.h"

        namespace main_robot{
            inline RobotParameters generateParams()
            {
                RobotParameters param;
                param.name = "MainRobot";

                param.wheelRadius = 49.1; ///< Wheel radius, in mm - identified during open loop experiments.
                param.wheelSpacing = 100.5; ///< Wheel spacing from robot center, in mm - identified during open loop experiments.
                param.encoderWheelRadius = 25.3; ///< Radius of encoder wheels, in mm.
                param.encoderWheelSpacing = 139.0; ///< Encoder wheel spacing from robot center, in mm.

                param.maxWheelSpeed = 400; ///< Maximum wheel speed, in mm/s.
                param.maxWheelAcceleration = 800; ///< Maximum wheel acceleration, in mm/s^2.

                param.maxWheelSpeedTrajectory = 300; ///< Maximum wheel speed, in mm/s, for trajectory generation.
                param.maxWheelAccelerationTrajectory = 300; ///< Maximum wheel acceleration, in mm/s^2, for trajectory generation.

                // Chassis physical dimensions
                param.CHASSIS_FRONT = 115.0;
                param.CHASSIS_BACK = 121.0;
                param.CHASSIS_WIDTH = 150.0;
                param.SUCTION_CENTER = 180.0;

                param.rightMotorId = 4;
                param.leftMotorId = 3;
                param.rightMotorDirection = 1;
                param.leftMotorDirection = -1;

                param.rightEncoderId = 0;
                param.leftEncoderId = 1;
                param.rightEncoderDirection = 1;
                param.leftEncoderDirection = -1;

                return param;
            }
        }
 #endif
