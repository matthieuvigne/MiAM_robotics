/// \file RobotInterface.h
/// \brief Interface for 2021 robot - for use on the robot and in simulation
/// \author MiAM Robotique, Matthieu Vigne
/// \author Rodolphe Dubois
/// \author Quentin Chan-Wai-Nam
/// \copyright GNU GPLv3

#ifndef ROBOT_INTERFACE_H
     #define ROBOT_INTERFACE_H

    #include <miam_utils/drivers/STSServoDriver.h>
    #include <unistd.h>
    #include <mutex>
    #include "common/RobotParameters.h"
    #include "common/MotionController.h"
    #include "common/Types.h"
#ifdef SIMULATION
    #include "FakeMetronome.h"
#endif

    class RobotGUI;
    class AbstractStrategy;

    /// @brief An abstract class representing a robot
    ///
    /// @details This class is responsible for all the 'logic' of the robot:
    ///            - a lowLevelLoop thread runs periodically
    ///            - a state-machine handles sensor init, wait for start and match start
    ///          Only hardware-related functions (sensor sampling and motor output) are
    ///          left to the child class.
    class RobotInterface
    {
        public:
            /// @brief An abstract class representing a robot
            /// @param robotParameters Robot physical parameters
            /// @param gui Graphical user interface
            /// @param strategy Strategy of the robot
            /// @param testMode Enable test mode (i.e. robot does not wait for start jack)
            /// @param teleplotPrefix Optional prefix for teleplot variables (used in simulation)
            RobotInterface(RobotParameters const& robotParameters,
                           RobotGUI *gui,
                           AbstractStrategy *strategy,
                           bool const& testMode,
                           std::string const& teleplotPrefix = "");

            /// @brief The low-level loop.
            ///
            /// @details This function is meant to run in a separate, high-priority thread.
            ///          It realises fixed-frequency robot control (based on a metronome_ object).
            void lowLevelLoop();

            /////////////////////////////////////
            /// Virtual functions - to overload
            /////////////////////////////////////

            /// \brief Stop the wheel motors.
            virtual void stopMotors() = 0;

            /// \brief Initialize every system of the robot.
            /// \details This function tries to initialize every component of the robot, by creating the software
            ///          object and, when applicable, testing the connection with the real component.
            ///          This function can be called several times, and only tries to re-initialize a component
            ///          if previous initializations failed.
            /// \return true if all components were initialized successfully, false otherwise.
            virtual bool initSystem() = 0;

            // Sleep a specified number of seconds.
            virtual void wait(double const& waitTimeS) = 0;

            /// @brief Update all sensors.
            virtual void updateSensorData() = 0;

            /// @brief Apply motor target
            /// @param target Target motor speed.
            virtual void applyMotorTarget(DrivetrainTarget const& target) = 0;

            /// @brief This function is called once at the end of the match.
            virtual void matchEnd() = 0;

            /// @brief Get state of the starting switch
            /// @return True is the starting switch is plugged in the robot
            virtual bool isStartingSwitchPluggedIn() const = 0;

            /////////////////////////////////////
            /// Getter/ Setter functions
            /////////////////////////////////////

            /// \brief Update the robot score.
            /// \details This function increments the score then updates the display accordingly.
            ///
            /// \param[in] scoreIncrement Amount to increment the score, both positive or negative.
            void updateScore(int const& scoreIncrement);

            RobotParameters getParameters() const;

            bool getTestMode() const;


            MotionController* getMotionController() { return &motionController_;}
            STSServoDriver* getServos() { return &servos_;}

            /// \brief Get time in current match.
            /// \return Time since start of the match, or 0 if not started.
            double getMatchTime();

            bool isStrategyTop() const;

            bool isPlayingRightSide() const;

            Logger logger_; ///< Logger object

        protected:
#ifdef SIMULATION
            FakeMetronome metronome_;
#else
            Metronome metronome_;
#endif
            MotionController motionController_;
            STSServoDriver servos_;
            RobotGUI *gui_;
            AbstractStrategy *strategy_;

            RobotGUIData guiState_; ///< Current robot state, as displayed on the gui.
            std::mutex mutex_;

            bool testMode_; // Test mode: no initial wait.

            bool hasMatchStarted_{false};
            double matchStartTime_{0.0};
            double currentTime_{0.0};
            double dt_{0.0};

            std::string teleplotPrefix_;
            RobotMeasurements measurements_;

        private:
            /// \brief Perform robot setup, returns true the match has started or not.
            ///
            /// \details This function is called periodically before the match starts. It is responsible for
            ///          updating the display and status according to user input. It returns true whenever the match
            ///          has started.
            bool setupBeforeMatchStart();

            /// @brief Init logger object.
            void initLogger();

    };
 #endif
