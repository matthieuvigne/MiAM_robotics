/// \file Parameters.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef PARAMETERS_H
    #define PARAMETERS_H
        #include <miam_utils/drivers/MaestroServoDriver.h>

        // Dimensions of the robot
        namespace robotdimensions
        {
            double const wheelRadius = 49.3; ///< Wheel radius, in mm - identified during open loop experiments.
            double const wheelSpacing = 100.5; ///< Wheel spacing from robot center, in mm - identified during open loop experiments.
            double const encoderWheelRadius = 25.3; ///< Radius of encoder wheels, in mm.
            //~ double const encoderWheelSpacing = 140.0; ///< Encoder wheel spacing from robot center, in mm.
            double const encoderWheelSpacing = 140.0; ///< Encoder wheel spacing from robot center, in mm.

            double const stepSize = 2 * M_PI / 600.0; ///< Size of a motor step, in rad.

            double const maxWheelSpeed = 200; ///< Maximum wheel speed, in mm/s.
            double const maxWheelAcceleration = 3000; ///< Maximum wheel acceleration, in mm/s^2.

            double const maxWheelSpeedTrajectory = 400; ///< Maximum wheel speed, in mm/s, for trajectory generation.
            double const maxWheelAccelerationTrajectory = 400; ///< Maximum wheel acceleration, in mm/s^2, for trajectory generation.
        }
 #endif
