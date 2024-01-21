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
                param.wheelRadius = 55.4; ///< Wheel radius, in mm - identified during open loop experiments.
                param.wheelSpacing = 105.0; ///< Wheel spacing from robot center, in mm - identified during open loop experiments.
                param.encoderWheelRadius = 26.25; ///< Radius of encoder wheels, in mm.
                param.encoderWheelSpacing = 131.2; ///< Encoder wheel spacing from robot center, in mm.

                param.maxWheelSpeedTrajectory = 500.0; ///< Maximum wheel speed, in mm/s, for trajectory generation.
                param.maxWheelAccelerationTrajectory = 800.0/3.; ///< Maximum wheel acceleration, in mm/s^2, for trajectory generation.

                param.linearKp = 3.5;
                param.linearKd = 0.01;
                param.linearKi = 0.0;
                param.transverseKp = 0.005;
                param.rotationKp = 5.0;
                param.rotationKd = 0.005;
                param.rotationKi = 0.0;

                // Chassis physical dimensions
                param.CHASSIS_FRONT = 90.0;
                param.CHASSIS_BACK = 130.0;
                param.CHASSIS_WIDTH = 125.0;
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
