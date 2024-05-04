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

                // Coefficients qui marchent à droite (false)
                param.rightWheelRadius = 55.5 * 0.999; ///< Right wheel radius, in mm - identified during open loop experiments.
                param.leftWheelRadius = 55.5; ///< Left wheel radius, in mm - identified during open loop experiments.
                param.wheelSpacing = 103.3; ///< Wheel spacing from robot center, in mm - identified during open loop experiments.
                param.rightEncoderWheelRadius = 25.9 * 1.015; ///< Radius of encoder wheels, in mm.
                param.leftEncoderWheelRadius = 25.9; ///< Radius of encoder wheels, in mm.
                param.encoderWheelSpacing = 132.0; ///< Encoder wheel spacing from robot center, in mm.

                param.maxWheelSpeedTrajectory = 600.0; ///< Maximum wheel speed, in mm/s, for trajectory generation.
                param.maxWheelAccelerationTrajectory = 700.; ///< Maximum wheel acceleration, in mm/s^2, for trajectory generation.

                param.linearKp = 3.5;
                param.linearKd = 0.01;
                param.linearKi = 0.0;
                param.transverseKp = 0.005;
                param.rotationKp = 5.0;
                param.rotationKd = 0.005;
                param.rotationKi = 0.0;

                // Chassis physical dimensions
                param.CHASSIS_FRONT = 140.0;
                param.CHASSIS_BACK = 140.0;
                param.CHASSIS_WIDTH = 140.0;
                // param.SUCTION_CENTER = 180.0;

                param.rightMotorId = 4;
                param.leftMotorId = 3;
                param.rightMotorDirection = 1;
                param.leftMotorDirection = -1;

                param.rightEncoderDirection = -1;
                param.leftEncoderDirection = 1;

                param.lidarOffset = M_PI_4;

                return param;
            }
        }
 #endif
