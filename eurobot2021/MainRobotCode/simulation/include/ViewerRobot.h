/// \file ViewerRobot.h
/// \brief Represent a robot in the viewer, following a trajectory.
/// \author Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef VIEWER_ROBOT_H
    #define VIEWER_ROBOT_H

    #include <gtkmm.h>
    #include <iostream>
    #include <miam_utils/trajectory/Trajectory.h>

    #include "RobotInterface.h"
    #include "ServoHandler.h"
    #include "MaestroMock.h"
    #include "Strategy.h"

    // Replay timestep.
    static double const TIMESTEP = 0.01;

    using miam::RobotPosition;

    struct ViewerTrajectoryPoint{
        double time;
        RobotPosition position;
        double linearVelocity;
        double angularVelocity;
        int score; ///< Current robot score.
        std::vector<double> servoState_;
        bool isPumpOn_;

        ViewerTrajectoryPoint():
            time(0.0),
            position(),
            linearVelocity(0.0),
            angularVelocity(0.0),
            score(0),
            servoState_(std::vector<double>(18, 0.0)),
            isPumpOn_(false)
        {}
    };

    class ViewerRobot: public RobotInterface
    {
        public:
            /// \brief Constructor.
            ViewerRobot(std::string const& imageFileName,
                        Strategy const& strategy,
                        double const& r = 1.0, double const& g = 0.0, double const& b = 0.0);

            /// \brief Get current robot position.
            RobotPosition getCurrentPosition() override;

            /// \brief Get viewer position.
            ViewerTrajectoryPoint getViewerPoint(int const& index);

            /// \brief Mock trajectory following.
            /// \details Returns true on succes, false if obstacle was encounterd.
            bool setTrajectoryToFollow(std::vector<std::shared_ptr<miam::trajectory::Trajectory>> const& trajectories) override;

            /// \brief Mock trajectory following.
            /// \details Returns true on succes, false if obstacle was encounterd.
            bool waitForTrajectoryFinished() override;

            /// \brief Reset robot position (velocity is set to 0, this is mostly done for position reset and init).
            void resetPosition(RobotPosition const& resetPosition, bool const& resetX = true, bool const& resetY = true, bool const& resetTheta = true) override;

            /// \brief Draw the robot and trajectory on the surface.
            void draw(const Cairo::RefPtr<Cairo::Context>& cr, double const& mmToCairo, int const& currentIndex);

            /// \brief Get length (i.e. number of samples) of the trajectory.
            int getTrajectoryLength();

            /// \brief Pad trajectory with the last point (zero velocity) to the desired length.
            void padTrajectory(int const& desiredLength);

            ///< Function computing the strategy, for a given obstacle position.
            void recomputeStrategy(int const& obstacleX, int const& obstacleY, int const& obstacleSize, bool const& isPlayingRightSide);

            ///< Increment robot score.
            void updateScore(int const& scoreIncrement) override;

            double getMatchTime() override;

            void lowLevelLoop() override {};

            void stopMotors() override {};

            ///< Reset robot score.
            void clearScore();

            ///< Robot is always playing on the right side.
            bool isPlayingRightSide()
            {
                return isPlayingRightSide_;
            }

            void moveRail(double const& position) override;

            void wait(double const& waitTimeS) override;

            int getScore(int const& index);

        private:
            bool followTrajectory(miam::trajectory::Trajectory * traj); ///< Perform actual trajectory following.
            Strategy strategy_;
            ServoHandler handler_;
            MaestroMock servoMock_;
            std::vector<ViewerTrajectoryPoint> trajectory_;

            Glib::RefPtr<Gdk::Pixbuf> image_;

            double r_;  ///< Trajectory color.
            double g_;  ///< Trajectory color.
            double b_;  ///< Trajectory color.
            int score_; ///< Current robot score.
            bool trajectoryFollowingStatus_; ///< Status of last trajectory following.


            double obstacleX_;  // Obstacle information.
            double obstacleY_;
            double obstacleSize_;

            bool isRobotPositionInit_;
            bool isPlayingRightSide_;
    };

#endif
