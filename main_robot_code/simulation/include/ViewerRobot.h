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


    #define TABLE_WIDTH_MM 3000.0
    #define TABLE_HEIGHT_MM 2000.0
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

    struct SimulatorData{
        bool isStartingSwitchPluggedIn = false;
        std::vector<Vector2> obstaclesPosition;
        double batteryVoltage = 20;
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


            /////////////////////////////////////
            /// Overload of pure virtual functions
            /////////////////////////////////////
            void stopMotors() override;
            bool initSystem() override;
            void wait(double const& waitTimeS) override;
            void updateSensorData() override;
            void applyMotorTarget(DrivetrainTarget const& target) override;
            void matchEnd() override;
            bool isStartingSwitchPluggedIn() const override;
            void shutdown() override;

            /////////////////////////////////////
            // Functions specific to the simulation.
            /////////////////////////////////////
            void reset(bool const& isPlayingRightSide);
            void tick(SimulatorData const& simulationData);
            RobotPosition getPosition();

            /// \brief Draw the robot and trajectory on the surface.
            void draw(const Cairo::RefPtr<Cairo::Context>& cr, double const& mmToCairo);

        private:
            std::vector<ViewerTrajectoryPoint> trajectory_;
            std::vector<ViewerTrajectoryPoint> currentTrajectory_;
            RobotPosition simulationPosition_;
            Vector2 simulatedEncoders_;
            WheelSpeed simulationSpeed_;
            DrivetrainTarget motionTarget_;
            DrivetrainKinematics kinematics_;

            Glib::RefPtr<Gdk::Pixbuf> image_;

            double r_;  ///< Trajectory color.
            double g_;  ///< Trajectory color.
            double b_;  ///< Trajectory color.
            int score_; ///< Current robot score.

            std::vector<pthread_t> runningThreads_;

            RobotGUI gui_;

            // Simulation-related variables

            std::condition_variable cv_;    ///< Conditional variable for thread synchronization.
            SimulatorData simulatorData_;
            std::thread lowLevelThread_;
            robotstate lastState_;
            std::mutex simulationMutex_;
            bool hasRobotRun_{false};
            bool hasMatchEnded_{false};
    };

#endif
