/// \file Robot.h
/// \brief This file defines global variables representing the hardware on the robot, that needs sharing between
///        different files.
///
/// \details This header is included in all other source files. It defines a structure representing the physical robot
///          in the code. Note that these variables are directly available, unprotected: it is to the user to ensure
///          no race condition may occur, for instance when writing the current robot position.
/// \author MiAM Robotique, Matthieu Vigne
/// \author Rodolphe Dubois
/// \author Quentin Chan-Wai-Nam
/// \copyright GNU GPLv3

#ifndef ROBOT_H
     #define ROBOT_H

    ///< Global includes
    #include <miam_utils/miam_utils.h>
    #include <miam_utils/raspberry_pi/RaspberryPi.h>
    #include <miam_utils/drivers/SPI-Wrapper.h>
    #include <miam_utils/drivers/RMDX.h>
    #include <miam_utils/drivers/RMDXController.h>
    #include <miam_utils/drivers/AS5045Driver.h>
    #include <miam_utils/drivers/BMI088Driver.h>
    #include <miam_utils/drivers/NautilusWrapper.h>
    #include <miam_utils/drivers/INA226Driver.h>
    #include <miam_utils/drivers/PCA9546ADriver.h>
    #include <miam_utils/trajectory/PointTurn.h>
    #include <miam_utils/trajectory/Utilities.h>
    #include <miam_utils/trajectory/DrivetrainKinematics.h>
    #include <miam_utils/RPLidarHandler.h>


    #include <math.h>
    #include <stdlib.h>
    #include <stdio.h>

    #include <memory>
    #include <vector>
    #include <mutex>

    #include "common/RobotInterface.h"

    // Right and left macros, for array addressing.
    using miam::RobotPosition;
    using miam::ProtectedPosition;
    using miam::trajectory::Trajectory;

    class Robot : public RobotInterface
    {
        public:

            /// \brief Constructor: do nothing for now.
            Robot(RobotParameters const& parameters,
                  AbstractStrategy *strategy,
                  RobotGUI *gui,
                  bool const& testMode,
                  bool const& disableLidar,
                  bool const& silent);


            /////////////////////////////////////
            /// Overload of pure virtual functions
            /////////////////////////////////////
            void stopMotors() override;
            bool initSystem() override;
            void wait(double const& waitTimeS) override;
            void updateSensorData() override;
            void applyMotorTarget(DrivetrainTarget const& target) override;
            void matchEnd() override;
            bool isStartingSwitchPluggedIn() const override;

            /// \brief Shut down the robot when Ctrl+C is pressed.
            void shutdown() override;

        private:
            // Robot hardware
            RPLidarHandler lidar_; ///< Lidar
            bool disableLidar_; // Disable lidar (works only in test mode)

            NautilusWrapper rightMotor_;
            NautilusWrapper leftMotor_;
            INA226 ina226_;
            VL53L0X vlxSensor_;

            PCA9546A i2cExpander_;
            INA226 ina226_7V_;
            INA226 ina226_12V_;
            BMI088 imu_;
            bool isI2CExpanderInit_ = false;
            bool isINA7Init_ = false;
            bool isINA12Init_ = false;
            bool isIMUInit_ = false;


            void updateRangeMeasurement(); // Thread handling VLX acquisition
            void detectBorders(); // Thread to detect borders, for match start.
            bool touchBorder();

            // Init variables.
            bool isMotorsInit_ = false;
            bool isEncodersInit_ = false;
            bool isLidarInit_ = false; ///< Boolean representing the initialization of the lidar.
            bool isServoInit_ = false;
            bool isINAInit_ = false;

            bool areMotorsLocked_ = false;

            WheelSpeed lastEncoderReading_;
            WheelSpeed lastMotorReading_;
            bool isEncoderInvalid_ = false;

            bool inBorderDetection_ = false;
    };
 #endif
