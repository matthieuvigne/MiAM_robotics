#ifndef MCVIEWER
#define MCVIEWER

#include <gtkmm.h>
#include <iostream>

#include <miam_utils/Logger.h>

#include "common/MotionController.h"
#include "main_robot/Parameters.h"

using miam::RobotPosition;
using miam::trajectory::TrajectoryVector;

// Create the MotionControllerTestingViewer object, linked to the window.
class MotionControllerTestingViewer : public Gtk::Window
{
public:
    // Create the window.
    MotionControllerTestingViewer(
        BaseObjectType *cobject,
        const Glib::RefPtr<Gtk::Builder> &refGlade,
        std::string const &tableImagePath,
        MotionController *motionController,
        Logger *logger);
    virtual ~MotionControllerTestingViewer();


    void setStartPosition(RobotPosition const& pos);
    void setEndPosition(RobotPosition const& pos);

    void recompute();


private:
    bool runAnimation();


    RobotPosition startPosition_;
    RobotPosition endPosition_;
    RobotPosition currentEditPosition_;

    // Mouse motion event.
    bool mouseMove(GdkEventMotion *motionEvent);
    bool mouseClicked(GdkEventButton *buttonEvent);
    bool mouseScrolled(GdkEventScroll *scrollEvent);
    void runClicked();

    void refresh();

    // GUI-related elements.
    double simulationTime_ = 0.0;
    bool isRunning_ = false;

    Gtk::DrawingArea *drawingArea;
    Gtk::Label *infoLabel;
    Gtk::RadioButton *buttonSetEndpoints;
    Gtk::RadioButton *buttonSetObstacle;

    Gtk::CheckButton *buttonShowStraightLine;
    Gtk::CheckButton *buttonShowRounded;
    Gtk::CheckButton *buttonShowMPC;

    TrajectoryVector mpcTrajectory_;


    bool redraw(const Cairo::RefPtr<Cairo::Context> &cr);

    // Table and robot images.
    // The following hypothesis is made: robotImage is a 50x50cm image, the center is the origin of the robot,
    // zero angle is the standard trigonometric zero.
    // tableImage is a 3x4m image of the table, centered.
    Glib::RefPtr<Gdk::Pixbuf> tableImage;

    // Timer for playing trajectory.
    std::chrono::high_resolution_clock::time_point animationStartTime_;

    // Table to cairo mapping.
    double mmToCairo_;
    double originX_;
    double originY_;

    MotionController *motionController_;
    Logger *logger_;

    std::vector<Obstacle> obstacles_;
};


#endif
