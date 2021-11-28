/// \file ServoConfig.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef SERVO_CONFIG_H
    #define SERVO_CONFIG_H
        #include <miam_utils/drivers/MaestroServoDriver.h>

        /// \brief Helper class for controlling the robot servos.
        class ServoHandler
        {
            public:
                /// \brief Constructor
                ServoHandler();

                /// \brief Initialize communication with the Maestro servo driver.
                ///
                /// \param portName Serial port file name ("/dev/ttyOx")
                /// \returns   true on success, false otherwise.
                bool init(std::string const& portName);

                // Valve control
                void openValve();
                void closeValve();

                void openTube(int tubeNumber); ///< Open suction air tube.
                void closeTube(int tubeNumber); ///< Close suction air tube.

                void tapOpen(); ///< Open air tap.
                void tapClose(); ///< Close air tap.

                void shutdownServos(); ///< Turn off all servos.

                void turnOnPump();
                void turnOffPump();

                // Electro magnet and arm control
                void electroMagnetOn();
                void electroMagnetOff();

                void figurineArmLow();
                void figurineArmMiddle();
                void figurineArmHigh();

                // Figurine speed
                void figurineArmSpeedLow();
                void figurineArmSpeedHigh();
                
                void moveSuction(bool high, bool moveMiddle = true);
                void moveMiddle();
                void moveMiddleSuctionForDrop(bool drop = false);

                void moveRail(int velocity);

                void foldArms();
                void unfoldArms(bool isPlayingRightSide);
                void raiseArms(bool isPlayingRightSide);
                void moveArmForDrop(bool isPlayingRightSide);
                void moveSuctionForGoldDrop();
            private:
                MaestroDriver maestro_;
        };
 #endif
