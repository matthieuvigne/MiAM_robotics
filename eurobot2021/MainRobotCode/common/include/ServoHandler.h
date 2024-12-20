/// \file ServoConfig.h
/// \brief Configuration for the main robot servos.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef SERVO_CONFIG_H
    #define SERVO_CONFIG_H
        #include <miam_utils/drivers/MaestroServoDriver.h>

        enum class statue
        {
            TRANSPORT = 0,
            CATCH = 1,
            DROP = 2,
            FOLD = 3
        };

        enum class arm
        {
            RAISE = 0,
            MEASURE = 1,
            FOLD = 2
        };

        enum class finger
        {
            PUSH = 0,
            MEASURE = 1,
            FOLD = 2
        };

        enum class suction
        {
            FOLD = 0,
            HORIZONTAL = 1,
            DROP_SAMPLE = 2,
            DROP_STATUE = 4,
            LOWER_SAMPLE = 5,
            VERTICAL = 6
        };

        enum class claw
        {
            FOLD = 0,
            FRONT = 1,
            SIDE = 2,
            VPOSITION = 3,
            PUSH_SAMPLE_SHELTER = 4,
            PUSH_SAMPLE_SHELTER_WIDER = 5,
            SIDE_SHALLOWER = 6
        };

        /// \brief Helper class for controlling the robot servos.
        class ServoHandler
        {
            public:
                /// \brief Constructor
                ServoHandler(MaestroDriver *maestro);

                /// \brief Initialize communication with the Maestro servo driver.
                ///
                /// \param portName Serial port file name ("/dev/ttyOx")
                /// \returns   true on success, false otherwise.
                bool init(std::string const& portName);
                void shutdownServos(); ///< Turn off all servos.

                // Valve / suction control
                void openValve();
                void closeValve();

                void openTube(int tubeNumber); ///< Open suction air tube.
                void closeTube(int tubeNumber); ///< Close suction air tube.
                void moveSuction(int const& suctionNumber, suction const& position);

                void moveRail(int velocity);

                void activatePump(bool const& pumpOn);

                void moveClaw(claw const& pose);

                // Statue-related functions
                void activateMagnet(bool const& magnetOn);
                void moveStatue(statue const& pose);

                // Side arm functions
                void moveArm(bool const& rightArm, arm const& pose);
                void moveFinger(bool const& rightArm, finger const& pose);

                void setMaestro(MaestroDriver & maestro);
                bool isPumpOn_;
            private:
                MaestroDriver * maestro_;
        };
 #endif
