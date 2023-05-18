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
    #include "miam_utils/trajectory/DrivetrainKinematics.h"
    #include "miam_utils/trajectory/RobotPosition.h"
    #include "miam_utils/trajectory/Trajectory.h"
    #include "miam_utils/trajectory/Utilities.h"
    #include "miam_utils/drivers/L6470Driver.h"
    #include "miam_utils/RPLidarHandler.h"
    #include "miam_utils/Types.h"

    #include <memory>
    #include <vector>
    #include <mutex>

    #include <common/MotionPlanner.h>
    #include <chrono>

    using miam::RobotPosition;
    using miam::ProtectedPosition;
    using miam::trajectory::Trajectory;
    using miam::trajectory::TrajectoryPoint;
    using miam::trajectory::TrajectoryVector;

    namespace side{
        int const RIGHT = 0;
        int const LEFT = 1;
    }

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

    typedef struct {
        Vector2 motorSpeed = Vector2::Zero(); ///<< Target motor speed, in rad/s
    }DrivetrainTarget;

    typedef struct {
        Vector2 encoderPosition; ///< Encoder position, rad.
        WheelSpeed encoderSpeed; ///< Speed, measured by the encoders
        Vector2 motorSpeed; ///<< Measured motor speed, in rad/s
        std::deque<DetectedRobot> lidarDetection; ///< Robots detected by the lidar.
    }DrivetrainMeasurements;

    // Detection arameters
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

      // during avoidance...
      double const x_max_avoidance = 380;
      double const y_max_avoidance = 300;
      double const xfar_max_avoidance = 380;
      double const yfar_max_avoidance = 300;

      double const mpc_obstacle_size = 380;

      // for ending trajectory...
      double const x_max_ending = 150;
      double const y_max_ending = 300;
    }

    // Dimensions of the table
    namespace table_dimensions {
      double constexpr table_max_x = 1950;
      double constexpr table_max_y = 2950;
      double constexpr table_min_x = 50;
      double constexpr table_min_y = 50;
    } // namespace table dimensions

    typedef std::tuple<RobotPosition, double> Obstacle;

    class MotionController
    {
        public:
            MotionController(RobotParameters const& robotParameters);

            /// \brief Initialize the system - this also starts the logger.
            /// \param[in] RobotPosition Starting position
            /// \param[in] teleplotPrefix Optional prefix for variables in teleplot: used in simulation
            ///                           where several robots are logging.
            void init(RobotPosition const& startPosition, std::string const& teleplotPrefix = "");

            /// \brief Get current robot position.
            /// \return Current robot position.
            RobotPosition getCurrentPosition();

            /// \brief Reset the position of the robot on the table.
            ///
            /// \details This function might be used for example when the robot is put in contact with a side of the table,
            ///             to gain back absolute position accuracy.
            ///
            /// \param[in] resetPosition The position to which reset the robot.
            /// \param[in] resetX Wheather or not to reset the X coordinate.
            /// \param[in] resetY Wheather or not to reset the Y coordinate.
            /// \param[in] resetTheta Wheather or not to reset the angle.
            void resetPosition(RobotPosition const& resetPosition, bool const& resetX, bool const& resetY, bool const& resetTheta);

            /// \brief Set new trajectory set to follow.
            /// \details This function is used to set the trajectories which will be followed by
            ///          the low-level thread. This function simply gives the input to the low-level thread
            ///          and returns immediately: use waitForTrajectoryFinish
            ///
            /// \param[in] trajectories Vector of trajectory to follow.
            bool setTrajectoryToFollow(TrajectoryVector const& trajectories);

            /// \brief Wait for the current trajectory following to be finished.
            /// \return true if trajectory following was successful, false otherwise.
            bool waitForTrajectoryFinished();


            /// \brief Get status of last trajectory following.
            bool isTrajectoryFinished();

            /// \brief Get status of last trajectory following.
            bool wasTrajectoryFollowingSuccessful();


            /// \brief Compute next motor target.
            ///
            /// \param[in] measurements Latest robot measurements
            /// \param[in] dt Elapsed time since last call.
            /// \return Target motor velocity
            DrivetrainTarget computeDrivetrainMotion(DrivetrainMeasurements const& measurements,
                                                     double const& dt,
                                                     bool const& hasMatchStarted);

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
                logger_.log(variableName, currentTime_, variableValue);
            }

            RobotParameters robotParams_;
            Logger logger_; ///< Logger object.
            MotionPlanner* motionPlanner_;

            void setAvoidanceMode(AvoidanceMode avoidanceMode);


            // void setDetectedObstacles(std::vector<RobotPosition> detectedObstacles);
            std::vector<Obstacle> getDetectedObstacles();
            std::vector<Obstacle> getPersistentObstacles();
            void addPersistentObstacle(Obstacle obstacle);
            void clearPersistentObstacles();
            void popBackPersistentObstacles();

            TrajectoryVector computeMPCTrajectory(RobotPosition targetPosition, std::vector<Obstacle> detectedObstacles, bool forward, bool avoidanceEnabled = false, bool ensureEndAngle = true);
            TrajectoryVector computeBasicAvoidanceTrajectory(RobotPosition targetPosition, std::vector<Obstacle> detectedObstacles, bool forward);

            std::vector<miam::RobotPosition> filteredDetectedObstacles_; ///< Detected obstables ; angle is M_PI if outside table else 0.
        private:
            ProtectedPosition currentPosition_; ///< Current robot position, thread-safe.
            double currentTime_{0.0};

            // Trajectory definition.
            TrajectoryVector newTrajectories_; ///< Vector of new trajectories to follow.
            TrajectoryVector currentTrajectories_; ///< Current trajectories being followed.
            bool wasTrajectoryFollowingSuccessful_; ///< Flag describing the success of the trajectory following process.
            std::mutex newTrajectoryMutex_;
            RobotPosition currentTargetEndPosition_; // < The current target position (saved for replanning)

            double curvilinearAbscissa_; ///< Curvilinear abscissa of the current trajectory.
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
            double computeObstacleAvoidanceSlowdown(std::deque<DetectedRobot> const& detectedRobots, bool const& hasMatchStarted);

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

            double slowDownCoeff_;
            double clampedSlowDownCoeff_ = 1.0;

            std::chrono::steady_clock::time_point timeSinceFirstStopped_;
            std::chrono::steady_clock::time_point timeSinceLastAvoidance_;

            std::mutex avoidanceComputationMutex_;
            bool avoidanceComputationScheduled_;
            bool avoidanceComputationEnded_;
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

    };
 #endif
