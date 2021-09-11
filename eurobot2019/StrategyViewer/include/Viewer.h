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
        Viewer(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade);
        virtual ~Viewer();

        /// \brief Add a robot to the display list.
        void addRobot(ViewerRobot robot);

    private:
        std::vector<ViewerRobot> robots_;  ///< List of robots being displayed.

        int currentTrajectoryIndex_; ///< Current index in the replay.
        int trajectoryLength_; ///< Length of the given trajectories.

        double obstacleX_; ///< Position of the obstacle.
        double obstacleY_; ///< Position of the obstacle.
        double obstacleSize_; ///< Size of the obstacle.
        // Move the obstacle.
        bool moveObstacle(GdkEventMotion* motion_event);
        bool clickObstacle(GdkEventButton* motion_event);

        // Recompute trajectories, based on obstacle positon.
        void recompute();

        Gtk::Label *replayTime;
        Gtk::Label *velocityLabel;
        Gtk::Scale *timeSlider_;
        Gtk::Scale *playbackSpeed;
        Gtk::DrawingArea *drawingArea;
        Gtk::ToggleButton *playButton;

        bool redraw(const Cairo::RefPtr<Cairo::Context>& cr);
        bool timeChanged(Gtk::ScrollType scroll, double new_value);

        // Table and robot images.
        // The following hypothesis is made: robotImage is a 50x50cm image, the center is the origin of the robot,
        // zero angle is the standard trigonometric zero.
        // tableImage is a 3x4m image of the table, centered.
        Glib::RefPtr<Gdk::Pixbuf> tableImage;
        Glib::RefPtr<Gdk::Pixbuf> robotImage;
        Glib::RefPtr<Gdk::Pixbuf> secondaryRobotImage;


        bool playTrajectory();
        void toggleReplayState();

        // Connection and timer for playing trajectory.
        sigc::connection replayConnection_;
        std::chrono::high_resolution_clock::time_point lastTime_;

        // Table to cairo mapping.
        double mmToCairo_;
        double originX_;
        double originY_;
};

#endif
