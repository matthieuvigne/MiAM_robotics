/// \file MotionController.h
/// \brief Drivetrain control: odometry, obstacle avoidance, motion control
/// \author MiAM Robotique, Matthieu Vigne
/// \author Rodolphe Dubois
/// \author Quentin Chan-Wai-Nam
/// \copyright GNU GPLv3

#ifndef MOTION_CONTROLLER_H
     #define MOTION_CONTROLLER_H

    #include <miam_utils/AbstractRobot.h>
    #include <unistd.h>
    #include "RobotParameters.h"


    #include <miam_utils/miam_utils.h>
    #include "miam_utils/trajectory/RobotPosition.h"
    #include "miam_utils/trajectory/Trajectory.h"
    #include "miam_utils/trajectory/Utilities.h"
    #include "miam_utils/drivers/L6470Driver.h"

    #include <memory>
    #include <vector>
    #include <mutex>

    #include <common/MotionPlanner.h>
    #include <common/MotionParameters.h>
    #include <chrono>

    #include "common/Types.h"

    using miam::RobotPosition;
    using miam::ProtectedPosition;
    using miam::trajectory::Trajectory;
    using miam::trajectory::TrajectoryPoint;
    using miam::trajectory::TrajectoryVector;

    enum AvoidanceMode {
        AVOIDANCE_BASIC,
        AVOIDANCE_MPC,
        AVOIDANCE_OFF
    };

    enum MotionControllerState {
        CONTROLLER_STOP                 = 0,
        CONTROLLER_TRAJECTORY_TRACKING  = 1,
        CONTROLLER_WAIT_FOR_AVOIDANCE   = 2,
        CONTROLLER_WAIT_FOR_TRAJECTORY  = 3
    };

    typedef std::tuple<RobotPosition, double> Obstacle;

    using tf = miam::trajectory::flags;

    class MotionController
    {
        public:
            MotionController(RobotParameters const& robotParameters,
                             Logger *logger);

            /// \brief Initialize the system - this also starts the logger.
            /// \param[in] RobotPosition Starting position
            void init(RobotPosition const& startPosition);

            /// \brief Get current robot position.
            /// \return Current robot position.
            RobotPosition getCurrentPosition() const;

            /// \brief Reset the position of the robot on the table.
            ///
            /// \details This function might be used for example when the robot is put in contact with a side of the table,
            ///             to gain back absolute position accuracy.
            ///
            /// \param[in] resetPosition The position to which reset the robot.
            /// \param[in] resetX Wheather or not to reset the X coordinate.
            /// \param[in] resetY Wheather or not to reset the Y coordinate.
            /// \param[in] resetTheta Wheather or not to reset the angle.
            void resetPosition(RobotPosition const& resetPosition, bool const& resetX = true, bool const& resetY = true, bool const& resetTheta = true);

            /// \brief Set new trajectory set to follow.
            /// \details This function is used to set the trajectories which will be followed by
            ///          the low-level thread. This function simply gives the input to the low-level thread
            ///          and returns immediately: usVectore waitForTrajectoryFinish
            ///
            /// \param[in] trajectories Vector of trajectory to follow.
            bool setTrajectoryToFollow(TrajectoryVector const& trajectories);

            /// \brief Wait for the current trajectory following to be finished.
            /// \return true if trajectory following was successful, false otherwise.
            bool waitForTrajectoryFinished();

            /// @brief When set, stop the tracking of the current trajectory, clearing the current trajectory buffer.
            void stopCurrentTrajectoryTracking();

            /// \brief Get status of last trajectory following.
            bool isTrajectoryFinished();

            /// \brief Get status of last trajectory following.
            bool wasTrajectoryFollowingSuccessful();

            /// @brief  Follow a specific rounded corner path
            /// @details This function is blocking, and returns once the motion has completed.
            /// @details This function can be set to blocking (default) or not, depending on the flag value
            /// \param[in] radius Circle radius - the same is used at each point.
            /// \param[in] transitionVelocityFactor Percentage of the maximum velocity along the circle at which to do the transition.
            /// @param flags Trajectory flags.
            /// @return True is move is successful, false otherwise
            bool goToRoundedCorners(std::vector<RobotPosition> const& positions,
                                    double radius = 200,
                                    double transitionVelocityFactor = 0.5,
                                    tf const& flags = tf::DEFAULT);

            /// @brief  Go to a target position in a straight line.
            /// @details This function can be set to blocking (default) or not, depending on the flag value
            /// @param targetPosition Target position
            /// @param enforceEndAngle If set, a final rotation is added to match the end angle
            /// @param speedRatio Ratio of maximum speed
            /// @param flags Trajectory flags (backward, end angle, blocking or not)
            /// @return True is move is successful, false otherwise ; if non-blocking always returns true.
            bool goToStraightLine(RobotPosition const& targetPosition,
                                  double const& speedRatio = 1.0,
                                  tf const& flags = tf::DEFAULT);

            /// @brief  Move a certain distance is a straight line.
            /// @details This function can be set to blocking (default) or not, depending on the flag value
            /// @param distance Distance - negative to go backward
            /// @param speedRatio Ratio of maximum speed
            /// @param flags Trajectory flags. Note that backward is overwritten by the sign of the distance.
            /// @return True is move is successful, false otherwise
            bool goStraight(double const& distance,
                            double const& speedRatio = 1.0,
                            tf const& flags = tf::DEFAULT);

            /// @brief  Turn a certain angle.
            /// @details This function can be set to blocking (default) or not, depending on the flag value
            /// @param angle Distance - negative to go backward
            /// @param speedRatio Ratio of maximum speed
            /// @param flags Trajectory flags.
            /// @return True is move is successful, false otherwise
            bool pointTurn(double const& angle,
                           double const& speedRatio = 1.0,
                           tf const& flags = tf::DEFAULT);

            /// \brief Compute next motor target.
            ///
            /// \param[in] measurements Latest robot measurements
            /// \param[in] dt Elapsed time since last call.
            /// \return Target motor velocity
            DrivetrainTarget computeDrivetrainMotion(DrivetrainMeasurements const& measurements,
                                                     double const& dt);

            bool isPlayingRightSide_ = false;

            DrivetrainKinematics getKinematics()
            {
                return kinematics_;
            }

            /// \brief Helper to log a variable at the current (match) time.
            /// \param[in] variableName Name of the variable
            /// \param[in] variableValue Value of the variable at the current time
            void log(std::string const& variableName, double const& variableValue)
            {
                logger_->log(variableName, currentTime_, variableValue);
            }

            RobotParameters robotParams_;

            MotionPlanner* getMotionPlanner()
            {
                return &motionPlanner_;
            }

            void setAvoidanceMode(AvoidanceMode avoidanceMode);


            // void setDetectedObstacles(std::vector<RobotPosition> detectedObstacles);
            std::vector<Obstacle> getDetectedObstacles(bool includePersistentObstacles = true);
            std::vector<Obstacle> getPersistentObstacles();
            void addPersistentObstacle(Obstacle obstacle);
            void clearPersistentObstacles();
            void popBackPersistentObstacles();

            /// @brief Compute a trajectory using MPC going to the target position
            /// @param targetPosition Position to reach
            /// @param detectedObstacles Obstacles that were detected
            /// @param flags Flags to configure the motion (backward, end angle)
            /// @param initialBackwardMotionMargin Positive margin of backward motion for the robot (used for avoidance)
            /// @return The computed trajectory vector
            TrajectoryVector computeMPCTrajectory(
                RobotPosition const targetPosition,
                std::vector<Obstacle> const detectedObstacles,
                tf const& flags,
                double const initialBackwardMotionMargin = 0);

            TrajectoryVector computeBasicAvoidanceTrajectory(
                RobotPosition targetPosition,
                std::vector<Obstacle> detectedObstacles,
                tf const& flags);

            std::vector<miam::RobotPosition> displayDetectedObstacles_; ///< Obstacles to display

            // bool avoidPersistentObstacles_;
            // void setAvoidPersistentObstacles(bool flag);

            TrajectoryVector getCurrentTrajectories();
            double getCurvilinearAbscissa();

            GameState *getGameState() { return &gameState_;};

            Map map_;

            TrajectoryConfig getCurrentTrajectoryParameters()
            {
                TrajectoryConfig conf = robotParams_.getTrajConf();
                if (gameState_.isBackClawFull || gameState_.isFrontClawFull)
                {
                    conf.maxWheelVelocity *= 0.6;
                    conf.maxWheelAcceleration *= 0.5;
                }
                return conf;
            }

        private:
            MotionPlanner motionPlanner_;
            Logger *logger_; ///< Logger object.
            ProtectedPosition currentPosition_; ///< Current robot position, thread-safe.
            double currentTime_{0.0};

            // Trajectory definition.
            TrajectoryVector newTrajectories_; ///< Vector of new trajectories to follow.
            TrajectoryVector currentTrajectories_; ///< Current trajectories being followed.
            bool wasTrajectoryFollowingSuccessful_{true}; ///< Flag describing the success of the trajectory following process.
            std::mutex newTrajectoryMutex_;
            RobotPosition currentTargetEndPosition_; // < The current target position (saved for replanning)

            double curvilinearAbscissa_{0.0}; ///< Curvilinear abscissa of the current trajectory.
            DrivetrainKinematics kinematics_;

            // Tracking PIDs
            miam::PID PIDLinear_; ///< Longitudinal PID.
            miam::PID PIDAngular_; ///< Angular PID.

            /// \brief Follow a trajectory.
            /// \details This function computes motor velocity to reach a specific trajectory point, and sends
            ///          it to the motors.
            /// \param[in] traj Current trajectory to follow.
            /// \param[in] timeInTrajectory Current time since the start of the trajectory.
            /// \param[in] dt Time since last servoing call, for PID controller.
            /// \param[out] target Motor target
            /// \return True if trajectory following should continue, false if trajectory following is completed.
            bool computeMotorTarget(Trajectory *traj,
                                    double const& timeInTrajectory,
                                    double const& dt,
                                    double const& slowDownRatio,
                                    DrivetrainMeasurements const &measurements,
                                    DrivetrainTarget &target);

            /// \brief Updates the LiDAR and sets the avoidance strategy
            /// \param [in] detectedRobots Obstacles detected by the lidar.
            /// \return coefficient for trajectory time increase
            double computeObstacleAvoidanceSlowdown(std::deque<DetectedRobot> const& detectedRobots);

            double computeObstacleAvoidanceSlowdownAnticipateTrajectory();

            /// @brief Update trajectory to perform avoidance
            /// @return avoidance traj
            TrajectoryVector performAvoidance();

            RobotPosition lidarPointToRobotPosition(LidarPoint const &point);
            bool isLidarPointWithinTable(LidarPoint const& point);

            // Avoidance functions
            AvoidanceMode avoidanceMode_;
            int avoidanceCount_;
            const int maxAvoidanceAttempts_ = 2;
            bool isStopped_;
            bool trajectoryDone_;

            double slowDownCoeff_;
            double clampedSlowDownCoeff_ = 1.0;

            std::chrono::steady_clock::time_point timeSinceFirstStopped_;
            std::chrono::steady_clock::time_point timeSinceLastAvoidance_;

            std::mutex avoidanceComputationMutex_;
            bool avoidanceComputationScheduled_;
            TrajectoryVector avoidanceComputationResult_;
            void loopOnAvoidanceComputation();
            std::vector<pthread_t> createdThreads_;

            // List of obstacles
            std::vector<Obstacle> detectedObstacles_;
            std::mutex detectedObstaclesMutex_;

            std::mutex persistentObstaclesMutex_;
            std::vector<Obstacle> persistentObstacles_;

            // // Handle robot stops
            // int numStopIters_ = 0.;
            // const int minStopIters_ = 12; // Minimum number of iterations to stop, i.e 10ms.
            // const int minRestartIters_ = 20; // Minimum number of iterations to restart, i.e 10ms.
            // int const maxStopIters_ = 50; // Maximum number of iterations until attempting something

            double trajectoryTimeout_ = 1.0; // Number of seconds after the end of trajectory after which timeout is raised

            // Motion controller state
            MotionControllerState motionControllerState_;
            void changeMotionControllerState();
            DrivetrainTarget resolveMotionControllerState(DrivetrainMeasurements const &measurements,
                                                           double const &dt,
                                                           bool const &hasMatchStarted);


            bool askedForTrackingStop_ = false;

            std::mutex motionPlanningMutex_;

            GameState gameState_;
            DrivetrainTarget lastTarget_;
    };
 #endif
