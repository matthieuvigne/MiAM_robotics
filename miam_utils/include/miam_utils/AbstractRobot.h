/// \file AbstractRobot.h
/// \brief Abstract class representing a robot (mostly its drivetrain), to give a common structure to both robot code.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef ABSTRACT_ROBOT_H
     #define ABSTRACT_ROBOT_H

    #include "MiAMEurobot/trajectory/DrivetrainKinematics.h"
    #include "MiAMEurobot/trajectory/RobotPosition.h"
    #include "MiAMEurobot/trajectory/Trajectory.h"
    #include "MiAMEurobot/drivers/L6470Driver.h"

    #include <memory>
    #include <vector>

    /// \brief Class representing the robot wheeled base.
    /// \details This class simply centralizes variables linked to robot motion.
    ///          It implements the low-level thread of the robot, responsible for driving it around the table, and logging.
    ///          Comunication with this thread is done through this class, in a thread-safe way when needed.
    class AbstractRobot
    {
        public:
            /// \brief Constructor: do nothing for now.
            AbstractRobot();

            /// \brief Get current robot position.
            /// \return Current robot position.
            miam::RobotPosition getCurrentPosition();

            /// \brief Get current robot base speed.
            /// \return Current robot base speed.
            BaseSpeed getCurrentBaseSpeed();

            /// \brief Reset the position of the robot on the table.
            ///
            /// \details This function might be used for example when the robot is put in contact with a side of the table,
            ///             to gain back absolute position accuracy.
            ///
            /// \param[in] resetPosition The position to which reset the robot.
            /// \param[in] resetX Wheather or not to reset the X coordinate.
            /// \param[in] resetY Wheather or not to reset the Y coordinate.
            /// \param[in] resetTheta Wheather or not to reset the angle.
            void resetPosition(miam::RobotPosition const& resetPosition, bool const& resetX, bool const& resetY, bool const& resetTheta);

            /// \brief Set new trajectory set to follow.
            /// \details This function is used to set the trajectories which will be followed by
            ///          the low-level thread. This function simply gives the input to the low-level thread
            ///          and returns immediately: use waitForTrajectoryFinish
            ///
            /// \param[in] trajectories Vector of trajectory to follow.
            void setTrajectoryToFollow(std::vector<std::shared_ptr<miam::trajectory::Trajectory>> const& trajectories);

            /// \brief Wait for the current trajectory following to be finished.
            /// \return true if trajectory following was successful, false otherwise.
            bool waitForTrajectoryFinished();

            /// \brief Get current trajectory following status.
            bool isTrajectoryFinished();

            /// \brief Stop the wheel motors.
            void stopMotors();

            /// \brief The low-level thread of the robot.
            /// \details This thread runs a periodic loop. At each iteration, it updates sensor values,
            ///          estimates the position of the robot on the table, and performs motor servoing.
            ///          It also logs everything in a log file.
            virtual void lowLevelLoop() = 0;

            /// \brief Get status of last trajectory following.
            bool wasTrajectoryFollowingSuccessful();
        protected:
            miam::L6470 stepperMotors_; ///< Robot driving motors.
            miam::ProtectedPosition currentPosition_; ///< Current robot position, thread-safe.
            BaseSpeed currentBaseSpeed_; ///< Current robot base speed.
            miam::trajectory::TrajectoryPoint trajectoryPoint_; ///< Current trajectory point.
            double currentTime_; ///< Current robot time, counted by low-level thread.
            std::vector<double> motorSpeed_; ///< Current motor speed.
            std::vector<int> motorPosition_; ///< Current motor position.

            // Trajectory definition.
            std::vector<std::shared_ptr<miam::trajectory::Trajectory>> newTrajectories_; ///< Vector of new trajectories to follow.
            std::vector<std::shared_ptr<miam::trajectory::Trajectory>> currentTrajectories_; ///< Current trajectories being followed.

            // Trajectory following timing.
            double trajectoryStartTime_; ///< Time at which the last trajectory following started.
            double lastTrajectoryFollowingCallTime_; ///< Time at which the last trajectory following started.

            // Init variables.
            bool isStepperInit_; ///< Boolean representing the initialization of the stepper motors.

            // Match-related variables.
            bool isPlayingRightSide_; ///< True if robot is playing on the right (purple) side of the field.
            bool hasMatchStarted_;    ///< Boolean flag to indicate match status.
            double matchStartTime_;   ///< Start time of the match, for end timer.

            bool wasTrajectoryFollowingSuccessful_; ///< Flag describing the success of the trajectory following process.
    };
 #endif
