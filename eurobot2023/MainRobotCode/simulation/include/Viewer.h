/// \file ViewerRobot.h
/// \brief The viewer window itself.
/// \author Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef SK_HANDLER
#define SK_HANDLER

#include <gtkmm.h>
#include <iostream>

#include "ViewerRobot.h"

// Create the Viewer object, linked to the window.
class Viewer : public Gtk::Window
{
    public:
        // Create the window.
        Viewer(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade, std::string const& tableImagePath);
        virtual ~Viewer();

        /// \brief Add a robot to the display list.
        void addRobot(ViewerRobot &robot);

        /// \brief Start the simulation.
        void start();

        /// @brief Reset button clicked callback.
        void resetClicked();

    private:
        std::vector<ViewerRobot*> robots_;  ///< List of robots being displayed.

        Vector2 obstaclePosition_ = Vector2::Constant(-300);  // Obstacle position

        // Move the obstacle.
        bool mouseMove(GdkEventMotion* motion_event);
        bool clickObstacle(GdkEventButton* motion_event);

        // Recompute trajectories, based on obstacle positon.
        void playClicked();
        void pauseClicked();
        void updateTimeRatio();

        // Function actually running the simulation
        bool runSimulation();

        // GUI-related elements.
        double simulationTime_ = 0.0;
        double simulationTimeRatio_ = 1.0;
        bool isRunning_ = false;

        Gtk::Label *mousePositionLabel;
        Gtk::Label *timeLabel;
        Gtk::Label *scoreLabel;
        Gtk::CheckButton *sideButton;
        Gtk::ProgressBar *progressBar;
        Gtk::SpinButton *simulationRatioSpin;
        Gtk::DrawingArea *drawingArea;

        bool redraw(const Cairo::RefPtr<Cairo::Context>& cr);

        // Table and robot images.
        // The following hypothesis is made: robotImage is a 50x50cm image, the center is the origin of the robot,
        // zero angle is the standard trigonometric zero.
        // tableImage is a 3x4m image of the table, centered.
        Glib::RefPtr<Gdk::Pixbuf> tableImage;


        // Timer for playing trajectory.
        std::chrono::high_resolution_clock::time_point lastTime_;

        // Table to cairo mapping.
        double mmToCairo_;
        double originX_;
        double originY_;
};

#endif
