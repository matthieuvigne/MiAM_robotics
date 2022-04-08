/// \file Parameters.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef PARAMETERS_H
    #define PARAMETERS_H
        #include <math.h>

        // Dimensions of the robot
        namespace robotdimensions
        {
            double const wheelRadius = 49.3; ///< Wheel radius, in mm - identified during open loop experiments.
            double const wheelSpacing = 100.5; ///< Wheel spacing from robot center, in mm - identified during open loop experiments.
            double const encoderWheelRadius = 25.3; ///< Radius of encoder wheels, in mm.
            //~ double const encoderWheelSpacing = 140.0; ///< Encoder wheel spacing from robot center, in mm.
            double const encoderWheelSpacing = 140.0; ///< Encoder wheel spacing from robot center, in mm.

            double const stepSize = 2 * M_PI / 600.0; ///< Size of a motor step, in rad.

            double const maxWheelSpeed = 500; ///< Maximum wheel speed, in mm/s.
            double const maxWheelAcceleration = 800; ///< Maximum wheel acceleration, in mm/s^2.

            double const maxWheelSpeedTrajectory = 400; ///< Maximum wheel speed, in mm/s, for trajectory generation.
            double const maxWheelAccelerationTrajectory = 400; ///< Maximum wheel acceleration, in mm/s^2, for trajectory generation.

            // Chassis physical dimensions
            double const CHASSIS_FRONT = 115.0;
            double const CHASSIS_BACK = 121.0;
            double const CHASSIS_WIDTH = 150.0;
            double const SUCTION_CENTER = 180.0;
        }


        // Servo configuration
        enum SERVOS
        {
            VALVE_RIGHT = 0,
            VALVE_CENTER = 1,
            VALVE_LEFT = 2,
            VALVE = 3,
            MAGNET = 4,
            SUCTION_RIGHT = 6,
            SUCTION_CENTER = 7,
            SUCTION_LEFT = 8,
            ELEVATOR = 9,
            RIGHT_ARM = 10,
            RIGHT_FINGER = 11,
            LEFT_ARM = 12,
            LEFT_FINGER = 13,
            STATUE = 14
        };

        std::string const SERVO_NAMES[18] = {
            "Valve right",
            "Valve center",
            "Valve left",
            "Tap",
            "Magnet",
            "",
            "Suction right",
            "Suction center",
            "Suction left",
            "Elevator",
            "Right arm",
            "Right finger",
            "Left arm",
            "Left finger",
            "Statue",
            "",
            "",
            ""
        };

        enum class ExcavationSquareColor{
            NONE = 0,
            PURPLE = 1,
            YELLOW = 2,
            RED = 3
        };
 #endif
