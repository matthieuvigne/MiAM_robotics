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
    #include <miam_utils/drivers/USBLCDDriver.h>
    #include <miam_utils/raspberry_pi/RaspberryPi.h>
    #include <miam_utils/trajectory/PointTurn.h>
    #include <miam_utils/trajectory/Utilities.h>
    #include <miam_utils/trajectory/DrivetrainKinematics.h>
    #include "miam_utils/RPLidarHandler.h"
    #include <math.h>
    #include <stdlib.h>
    #include <stdio.h>

    #include <memory>
    #include <vector>
    #include <mutex>

    #include "uCListener.h"
    #include "ServoHandler.h"
    #include "LoggerFields.h"
    #include "RobotInterface.h"
    #include "Strategy.h"

    // Right and left macros, for array addressing.
    int const RIGHT = 0;
    int const LEFT = 1;

    using miam::RobotPosition;
    using miam::ProtectedPosition;
    using miam::trajectory::Trajectory;

    ///< The various steps of the startup process.
    enum startupstatus{
        INIT,
        WAITING_FOR_CABLE,
        WAITING_FOR_START
    };


    // Controller parameters
    namespace controller
    {
        //~ double const transverseKp = 0.1;

        double const linearKp = 3.5; // previously 3.0
        double const linearKd = 0.01; // previously 0.0
        double const linearKi = 0.0; // previously 0.1

        double const transverseKp = 0.005;

        double const rotationKp = 10.0;
        //~ double const rotationKp = 0.0;
        double const rotationKd = 0.01;
        double const rotationKi = 0.0;

        double const railKp = 20.0;
        double const railKd = 0.0;
        double const railKi = 0.0;
    }

    // Detection parameters
    namespace detection {

      // Zone radius
      double constexpr r1 = 400;
      double constexpr r2 = 700;

      // Zone angular width
      double constexpr theta1 = M_PI_2;
      double constexpr theta2 = 0.70;

      double const x_max = 500;
      double const y_max = 300;
      double const xfar_max = 700;
      double const yfar_max = 500;
    }

    // Dimensions of the table
    namespace table_dimensions {
      double constexpr table_max_x = 2950;
      double constexpr table_max_y = 1950;
      double constexpr table_min_x = 50;
      double constexpr table_min_y = 50;
    } // namespace table dimensions

    class Robot : public RobotInterface
    {
        public:

            /// \brief Constructor: do nothing for now.
            Robot(bool const& testMode, bool const& disableLidar);

            /// \brief Initialize every system of the robot.
            /// \details This function tries to initialize every component of the robot, by creating the software
            ///          object and, when applicable, testing the connection with the real component.
            ///          This function can be called several times, and only tries to re-initialize a component
            ///          if previous initializations failed.
            /// \return true if all components were initialized successfully, false otherwise.
            bool initSystem();

            /// \brief The low-level thread of the robot.
            /// \details This thread runs a periodic loop. At each iteration, it updates sensor values,
            ///          estimates the position of the robot on the table, and performs motor servoing.
            ///          It also logs everything in a log file.
            void lowLevelLoop() override;

            /// \brief Set a new target to the rail.
            ///
            /// \param position Relative rail position, form 0 (down) to 1 (up).
            /// \param wait If set, this function blocks until motion is complete.
            void moveRail(double const& position) override;

            /// \brief Get current rail position.
            ///
            /// \return Current relative rail position, form 0 (down) to 1 (up).
            double getRailPosition();

            /// \brief Update the robot score.
            /// \details This function increments the score then updates the display accordingly.
            ///
            /// \param[in] scoreIncrement Amount to increment the score, both positive or negative.
            void updateScore(int const& scoreIncrement) override;

            void stopMotors() override;

            // List of all system on the robot, public for easy external access (they might be moved latter on).
            ServoHandler servos_; ///< Interface for the servo driver.
            MaestroDriver maestro_;
            USBLCD screen_; ///< LCD screen and buttons.
            RPLidarHandler lidar_; ///< Lidar

            bool isPlayingRightSide() const override
            {
                return isPlayingRightSide_;
            }

            /// \brief Get time in current match.
            /// \return Time since start of the match, or 0 if not started.
            double getMatchTime();

            bool ignoreDetection_; ///<< Turn off detection in some very specific instants.
            int avoidanceTimeout_; ///<< Time to wait before abording, in number of iterations.

            ExcavationSquareColor getExcavationReadings(bool readRightSide) override;

            bool getTestMode() const override
            {
                return testMode_;
            }

            double getRangeSensorMeasurement(bool measureRightSide) const override
            {
                return rangeMeasurements_[measureRightSide ? RIGHT : LEFT];
            }

        private:
            bool testMode_; // Test mode: no initial wait.
            bool disableLidar_; // Disable lidar (works only in test mode)

            /// \brief Update the logfile with current values.
            void updateLog();

            /// \brief Update the target of the trajectory following algorithm.
            /// \details This function is responsible for handling new trajectories, and switching through
            ///          the trajectory vector to follow.
            /// \param[in] dt Time since this function was last called, for PD controller.
            void updateTrajectoryFollowingTarget(double const& dt);

            /// \brief Follow a trajectory.
            /// \details This function computes motor velocity to reach a specific trajectory point, and sends
            ///          it to the motors.
            /// \param[in] traj Current trajectory to follow.
            /// \param[in] timeInTrajectory Current time since the start of the trajectory.
            /// \param[in] dt Time since last servoing call, for PID controller.
            /// \return True if trajectory following should continue, false if trajectory following is completed.
            bool followTrajectory(Trajectory *traj, double const& timeInTrajectory, double const& dt);

            /// \brief Updates the LiDAR and sets the avoidance strategy
            /// \param [out] coefficient for trajectory time increase
            double avoidOtherRobots();
            bool isLidarPointWithinTable(LidarPoint const& point);

            /// \brief Perform robot setup, return wheather the match has started or not.
            ///
            /// \details This function is called periodically before the match starts. It is responsible for
            ///          updating the display and status according to user input. It returns true whenever the match
            ///          has started.
            bool setupBeforeMatchStart();

            /// \brief Move the rail to the target position.
            /// \details This function computes the velocity to be applied to the rail servo in order to reach
            /// the desired target rail position using the potentiometer data.
            /// \param[in] dt Time since last servoing call, for PID controller.
            void updateMoveRail(double const& dt);

            void calibrateRail();
            int railHigh_;

            miam::L6470 stepperMotors_; ///< Robot driving motors.

            uCData microcontrollerData_; ///< Data structure containing informations from the arduino board.
            Logger logger_; ///< Logger object.

            // Traking errors.
            double trackingLongitudinalError_; ///< Tracking error along tangent to trajectory.
            double trackingTransverseError_; ///< Tracking error along normal to trajectory.
            double trackingAngleError_; ///< Tracking angle error.

            // Tracking PIDs
            miam::PID PIDLinear_; ///< Longitudinal PID.
            miam::PID PIDAngular_; ///< Angular PID.

            // Rail PID
            miam::PID PIDRail_; ///< PID for the rail.

            // Rail
            int targetRailPosition_; ///< The desired rail position (in potentiometer unit). Should be -1 if not yet set during the match.

            // Kinematics
            DrivetrainKinematics kinematics_;
            double coeff_ = 1.0;

            // Init variables.
            bool isScreenInit_ = {false}; ///< Boolean representing the initialization of the screen motors.
            bool isStepperInit_ = {false}; ///< Boolean representing the initialization of the stepper motors.
            bool isServosInit_ = {false}; ///< Boolean representing the initialization of the servo driving board.
            bool isArduinoInit_ = {false}; ///< Boolean representing the initialization of the slave arduino board.
            bool isLidarInit_ = {false}; ///< Boolean representing the initialization of the lidar.
            bool isRangeSensorInit_[2] = {false, false}; ///< Initialization of range sensors.
            int score_={0}; ///< Current robot score.
            std::mutex mutex_; ///< Mutex, for thread safety.

            startupstatus startupStatus_; ///< Current startup status.
            bool initMotorBlocked_; ///< State of the motors during init.
            bool initStatueHigh_; ///< State of the motors during init.

            double curvilinearAbscissa_;
            int nLidarPoints_;  ///< Number of points read by the lidar.

            VL53L0X rangeSensors_[2];

            double rangeMeasurements_[2] = {0, 0};
            void updateRangeMeasurement(); // Range measurement thread

            Strategy strategy_;

            double timeSinceLastCheckOnRailHeightDuringInit_;
    };

    extern Robot robot;    ///< The robot instance, representing the current robot.
 #endif
