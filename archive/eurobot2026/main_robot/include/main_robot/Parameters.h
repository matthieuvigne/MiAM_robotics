/// \file Parameters.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef MAIN_ROBOT_PARAMETERS_H
    #define MAIN_ROBOT_PARAMETERS_H
        #include <math.h>

        #include "common/RobotParameters.h"

        #include "MPCParameters.h"

        namespace main_robot{
            inline RobotParameters generateParams()
            {
                RobotParameters param;
                param.name = "MainRobot";

                // Coefficients qui marchent à droite (false)
                param.rightWheelRadius = 54.4; ///< Right wheel radius, in mm - identified during open loop experiments.
                param.leftWheelRadius = 55.0; ///< Left wheel radius, in mm - identified during open loop experiments.
                param.wheelSpacing = REFERENCE_WHEEL_SPACING; ///< Wheel spacing from robot center, in mm - identified during open loop experiments.
                param.rightEncoderWheelRadius = 26.42; ///< Radius of encoder wheels, in mm.
                param.leftEncoderWheelRadius = 26.42; ///< Radius of encoder wheels, in mm.
                param.encoderWheelSpacing = 144.0; ///< Encoder wheel spacing from robot center, in mm. Decrease if robot turns too much.

                param.maxWheelSpeedTrajectory = MAX_WHEEL_SPEED; ///< Maximum wheel speed, in mm/s, for trajectory generation.
                param.maxWheelAccelerationTrajectory = MAX_WHEEL_ACCELERATION; ///< Maximum wheel acceleration, in mm/s^2, for trajectory generation.

                param.linearKp = 3.5;
                param.linearKd = 0.01;
                param.linearKi = 0.0;
                param.transverseKp = 0.005;
                param.rotationKp = 5.0;
                param.rotationKd = 0.005;
                param.rotationKi = 0.0;

                param.chassisBack = 145.0;
                param.chassisFront = 145.0;
                param.chassisHalfWidth = 155.0;


                param.rightMotorDirection = 1;
                param.leftMotorDirection = -1;

                param.rightEncoderDirection = -1;
                param.leftEncoderDirection = 1;

                param.lidarOffset = M_PI_4;

                return param;
            }
        }
 #endif
