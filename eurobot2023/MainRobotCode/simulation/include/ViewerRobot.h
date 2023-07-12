/// \file ViewerRobot.h
/// \brief Represent a robot in the viewer, following a trajectory.
/// \author Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef VIEWER_ROBOT_H
    #define VIEWER_ROBOT_H

    #include <gtkmm.h>
    #include <thread>
    #include <iostream>
    #include <miam_utils/trajectory/Trajectory.h>
    #include <miam_utils/Types.h>

    #include "common/RobotParameters.h"
    #include "common/RobotInterface.h"
    #include "common/AbstractStrategy.h"
    #include "common/RobotGUI.h"

    #include "MaestroMock.h"


    #define TABLE_WIDTH_MM 2000.0
    #define TABLE_HEIGHT_MM 3000.0
    #define TABLE_MARGIN_MM 0.0

    // Replay timestep.
    static double const TIMESTEP = 0.01;

    using miam::RobotPosition;

    struct ViewerTrajectoryPoint{
        double time;
        RobotPosition position;
        int score; ///< Current robot score.
        bool isPumpOn_;

        ViewerTrajectoryPoint():
            time(0.0),
            position(),
            score(0),
            isPumpOn_(false)
        {}
    };

    class ViewerRobot: public RobotInterface
    {
        public:
            /// \brief Constructor.
            ViewerRobot(RobotParameters const& robotParameters,
                        std::string const& imageFileName,
                        AbstractStrategy *strategy,
                        double const& r = 1.0, double const& g = 0.0, double const& b = 0.0,
                        std::string const& teleplotPrefix_ = "");


            bool isPlayingRightSide() const override
            {
                return isPlayingRightSide_;
            }

            // Sleep a specified number of seconds.
            virtual void wait(double const& waitTimeS) override;

            /// \brief Update the robot score.
            /// \details This function increments the score then updates the display accordingly.
            ///
            /// \param[in] scoreIncrement Amount to increment the score, both positive or negative.
            void updateScore(int const& scoreIncrement)
            {
                score_ += scoreIncrement;
            }

            int getScore()
            {
                return score_;
            }

            /// \brief Draw the robot and trajectory on the surface.
            void draw(const Cairo::RefPtr<Cairo::Context>& cr, double const& mmToCairo);

            /// \brief Get time in current match.
            /// \return Time since start of the match, or 0 if not started.
            double getMatchTime() override;

            /// \brief Stop the wheel motors.
            void stopMotors() override {}

            // Functions specific to the simulation.
            void reset(bool const& isPlayingRightSide);

            void tick(double const& dt, double const& simulationTime, std::vector<Vector2 > const& obstaclesPosition);

            RobotPosition getPosition();

        private:
            AbstractStrategy *strategy_;
            std::vector<ViewerTrajectoryPoint> trajectory_;
            std::vector<ViewerTrajectoryPoint> currentTrajectory_;
            RobotPosition simulationPosition_;
            DrivetrainMeasurements measurements_;
            DrivetrainTarget motionTarget_;
            DrivetrainKinematics kinematics_;

            Glib::RefPtr<Gdk::Pixbuf> image_;

            double r_;  ///< Trajectory color.
            double g_;  ///< Trajectory color.
            double b_;  ///< Trajectory color.
            int score_; ///< Current robot score.
            std::string teleplotPrefix_;

            bool isPlayingRightSide_ = false;

            double simulationTime_ = 0.0;

            pthread_t runningThread_ = 0;

            RobotGUI gui_;
    };

#endif
