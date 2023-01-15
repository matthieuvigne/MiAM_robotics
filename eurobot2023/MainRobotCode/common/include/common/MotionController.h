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
    #include "miam_utils/drivers/L6470Driver.h"
    #include "miam_utils/RPLidarHandler.h"
    #include "miam_utils/Types.h"

    #include <memory>
    #include <vector>
    #include <mutex>

    using miam::RobotPosition;
    using miam::ProtectedPosition;
    using miam::trajectory::Trajectory;
    using miam::trajectory::TrajectoryPoint;

    namespace side{
        int const RIGHT = 0;
        int const LEFT = 1;
    }


    typedef struct {
        Vector2 motorSpeed = Vector2::Zero(); ///<< Target motor speed, in rad/s
    }DrivetrainTarget;

    typedef struct {
        Vector2 encoderPosition; ///< Encoder position, rad.
        WheelSpeed encoderSpeed; ///< Speed, measured by the encoders
        Vector2 motorSpeed; ///<< Measured motor speed, in rad/s
        std::deque<DetectedRobot> lidarDetection; ///< Robots detected by the lidar.
    }DrivetrainMeasurements;


    // Controller parameters
    namespace motioncontroller
    {
        //~ double const transverseKp = 0.1;

        double const linearKp = 3.5; // previously 3.0
        double const linearKd = 0.01; // previously 0.0
        double const linearKi = 0.0; // previously 0.1

        double const transverseKp = 0.005;

        double const rotationKp = 10.0;
        double const rotationKd = 0.01;
        double const rotationKi = 0.0;
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
      double constexpr table_max_x = 1950;
      double constexpr table_max_y = 2950;
      double constexpr table_min_x = 50;
      double constexpr table_min_y = 50;
    } // namespace table dimensions

    class MotionController
    {
        public:
            MotionController(RobotParameters const& robotParameters);

            /// \brief Initialize the system - this also starts the logger.
            void init(RobotPosition const& startPosition);

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
            bool setTrajectoryToFollow(std::vector<std::shared_ptr<Trajectory>> const& trajectories);

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

            RobotParameters robotParams_;

        private:
            Logger logger_; ///< Logger object.
            ProtectedPosition currentPosition_; ///< Current robot position, thread-safe.
            double currentTime_{0.0};

            // Trajectory definition.
            std::vector<std::shared_ptr<Trajectory>> newTrajectories_; ///< Vector of new trajectories to follow.
            std::vector<std::shared_ptr<Trajectory>> currentTrajectories_; ///< Current trajectories being followed.
            bool wasTrajectoryFollowingSuccessful_; ///< Flag describing the success of the trajectory following process.
            std::mutex newTrajectoryMutex_;

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

            bool isLidarPointWithinTable(LidarPoint const& point);


            // Handle robot stops
            int num_stop_iters = 0.;
            const int min_stop_iters = 12; // Minimum number of iterations to stop, i.e 10ms.
            const int min_restart_iter = 20; // Minimum number of iterations to restart, i.e 10ms.
    };
 #endif
