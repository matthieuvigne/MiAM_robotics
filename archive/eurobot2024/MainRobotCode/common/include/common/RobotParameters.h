/// \file Parameters.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef ROBOT_PARAMETERS_H
    #define ROBOT_PARAMETERS_H
        #include <math.h>

        #include <miam_utils/trajectory/Trajectory.h>

        // Robot update period, in ns
        #define ROBOT_UPDATE_PERIOD 5e6
        // Discharge voltage, i.e. voltage at which we refuse to start a match.
        // For typical LiPo, 3.5V corresponds to about 15% of remaining capacity
        // and is typically used as low voltage warning on drones.
        #define UNDERVOLTAGE_LEVEL 17.5


        // Parameters defining a robot
        struct RobotParameters {
            std::string name;
            double rightWheelRadius; ///< Right wheel radius, in mm
            double leftWheelRadius; ///< Left wheel radius, in mm
            double wheelSpacing; ///< Wheel spacing from robot center, in mm
            double rightEncoderWheelRadius;  ///< Radius of encoder wheels, in mm.
            double leftEncoderWheelRadius;  ///< Radius of encoder wheels, in mm.
            double encoderWheelSpacing;  ///< Spacing between both encoder wheels, in mm.

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

            // Number of lidar ponts per turn ; 800 is the default for RPLIDAR V2
            unsigned int lidarNPointsPerTurn = 800;

            inline miam::trajectory::TrajectoryConfig getTrajConf() const
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

            int rightEncoderDirection = 1; // Invert encoder if needed: positive is moving forward.
            int leftEncoderDirection = -1; // Invert encoder if needed: positive is moving forward.

            double lidarOffset = M_PI / 4.0;
        };

 #endif
