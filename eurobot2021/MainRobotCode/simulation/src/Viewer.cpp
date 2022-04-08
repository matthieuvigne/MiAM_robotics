/// \author Matthieu Vigne
/// \copyright GNU GPLv3

#include "Viewer.h"
#include <iomanip>
#include <sstream>


// Build window from Glade.
Viewer::Viewer(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade, std::string const& tableImagePath) :
    Gtk::Window(cobject),
    obstacleX_(-500),
    obstacleY_(-500),
    obstacleSize_(200),
    mmToCairo_(1.0),
    originX_(0.0),
    originY_(0.0)
{
    currentTrajectoryIndex_ = 0;
    // Associate key press to entry.
    Gtk::Widget *window;
    refGlade->get_widget("mainWindow", window);

    // Associate widgets.
    refGlade->get_widget("replayTime", replayTime);
    refGlade->get_widget("velocityLabel", velocityLabel);
    refGlade->get_widget("drawingArea", drawingArea);
    drawingArea->signal_draw().connect(sigc::mem_fun(this, &Viewer::redraw));

    set_events(Gdk::BUTTON_PRESS_MASK  | Gdk::BUTTON_RELEASE_MASK | Gdk::KEY_PRESS_MASK | Gdk::KEY_RELEASE_MASK | Gdk::ENTER_NOTIFY_MASK);
    drawingArea->set_events(Gdk::BUTTON_PRESS_MASK  | Gdk::BUTTON_RELEASE_MASK | Gdk::BUTTON_MOTION_MASK | Gdk::SCROLL_MASK);

    drawingArea->signal_motion_notify_event().connect(sigc::mem_fun(this, &Viewer::moveObstacle));
    drawingArea->signal_button_press_event().connect(sigc::mem_fun(this, &Viewer::clickObstacle));
    drawingArea->signal_button_release_event().connect(sigc::mem_fun(this, &Viewer::clickObstacle));

    Gtk::Button *button;
    refGlade->get_widget("recomputeButton", button);
    button->signal_clicked().connect(sigc::mem_fun(this, &Viewer::recompute));
    refGlade->get_widget("rightSideToggle", rightSideButton_);
    rightSideButton_->signal_toggled().connect(sigc::mem_fun(this, &Viewer::recompute));
    // Configure slider based on viewerTrajectory.
    refGlade->get_widget("playbackSpeed", playbackSpeed);
    refGlade->get_widget("timeSlider", timeSlider_);
    timeSlider_->signal_change_value().connect(sigc::mem_fun(this, &Viewer::timeChanged));

    refGlade->get_widget("playButton", playButton);
    playButton->signal_toggled().connect(sigc::mem_fun(this, &Viewer::toggleReplayState));

    // Load images.
    tableImage = Gdk::Pixbuf::create_from_file(tableImagePath, -1, -1);
    trajectoryLength_ = 0;
}


Viewer::~Viewer()
{

}

void Viewer::addRobot(ViewerRobot & robot)
{
    robots_.push_back(&robot);
    recompute();
}

bool Viewer::redraw(const Cairo::RefPtr<Cairo::Context>& cr)
{
    // Check replay index.
    if(currentTrajectoryIndex_ < 0)
        currentTrajectoryIndex_ = 0;
    if(currentTrajectoryIndex_ >= trajectoryLength_)
        currentTrajectoryIndex_ = trajectoryLength_ - 1;


    // Put scaled table, keeping ratio.
    double heightToWidthRatio = tableImage->get_height() / (1.0 * tableImage->get_width());
    int widgetWidth = drawingArea->get_allocated_width();
    int widgetHeight = drawingArea->get_allocated_height();

    double newWidth = std::min(widgetWidth, static_cast<int>(widgetHeight / heightToWidthRatio));
    double newHeight = std::min(widgetHeight, static_cast<int>(heightToWidthRatio * widgetWidth));
    Gdk::Cairo::set_source_pixbuf(cr,
                                  tableImage->scale_simple(newWidth, newHeight, Gdk::INTERP_BILINEAR ),
                                  (widgetWidth - newWidth) / 2,
                                  (widgetHeight - newHeight) / 2);
    cr->paint();

    // Get table origin and scaling.
    mmToCairo_ = newWidth / 4000.0;

    originX_ = (widgetWidth - newWidth) / 2 + 500.0 * mmToCairo_;
    originY_ = (widgetHeight - newHeight) / 2 + 500.0 * mmToCairo_;

    cr->translate(originX_, originY_);

    int score = 0;
    for(auto robot : robots_)
    {
        robot->draw(cr, mmToCairo_, currentTrajectoryIndex_);
        score += robot->getScore(currentTrajectoryIndex_);
    }

    // Draw obstacle.
    cr->set_source_rgb(1.0, 0.0, 0.0);
    cr->arc(mmToCairo_ * obstacleX_, mmToCairo_ * (2000 - obstacleY_), mmToCairo_ * obstacleSize_, 0, 2 * M_PI - 0.1);
    cr->fill();

    // Update labels.
    double currentTime = currentTrajectoryIndex_ * TIMESTEP ;
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << currentTime << "/" << ((trajectoryLength_ - 1) * TIMESTEP);
    replayTime->set_text("Time: " + stream.str());
    timeSlider_->set_value(currentTime);

    std::stringstream scoreStream;
    scoreStream << "Score: " << score;
    velocityLabel->set_text(scoreStream.str());


    return TRUE;
}

bool Viewer::timeChanged(Gtk::ScrollType scroll, double new_value)
{
    currentTrajectoryIndex_ = static_cast<int>(new_value / TIMESTEP);
    drawingArea->queue_draw();
    return true;
}


bool Viewer::playTrajectory()
{
    // Increment currentTrajectoryIndex_ based on time elapsed: increment as much as needed to catch back with real
    // clock.
    std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
    double realElapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastTime_).count();
    lastTime_ = currentTime;

    currentTrajectoryIndex_ += static_cast<int>(std::floor(realElapsedTime * playbackSpeed->get_value() / TIMESTEP));
    // If end of the trajectory is reached, stop the connection.
    if(currentTrajectoryIndex_ >= trajectoryLength_)
        toggleReplayState();
    drawingArea->queue_draw();

    return true;
}

void Viewer::toggleReplayState()
{
    if(!replayConnection_.connected())
    {
        replayConnection_ = Glib::signal_timeout().connect(sigc::mem_fun(*this,&Viewer::playTrajectory), 50);
        lastTime_ = std::chrono::high_resolution_clock::now();
    }
     else
        replayConnection_.disconnect();
}


bool Viewer::moveObstacle(GdkEventMotion* motion_event)
{
    obstacleX_ = motion_event->x;
    obstacleY_ = motion_event->y;
    // Convert coordinates to table coordinates
    obstacleX_ = (obstacleX_ - originX_) / mmToCairo_;
    obstacleY_ = 2000 - (obstacleY_ - originY_) / mmToCairo_;
    drawingArea->queue_draw();
    return true;
}


bool Viewer::clickObstacle(GdkEventButton* motion_event)
{
    obstacleX_ = motion_event->x;
    obstacleY_ = motion_event->y;
    // Convert coordinates to table coordinates
    obstacleX_ = (obstacleX_ - originX_) / mmToCairo_;
    obstacleY_ = 2000 - (obstacleY_ - originY_) / mmToCairo_;
    drawingArea->queue_draw();
    return true;
}

void Viewer::recompute()
{
    // Recompute strategies.
    for(auto r : robots_)
        r->recomputeStrategy(obstacleX_, obstacleY_, obstacleSize_, rightSideButton_->get_active());
    // Re-equalize time vectors.
    for(auto r : robots_)
        trajectoryLength_ = std::max(trajectoryLength_, r->getTrajectoryLength());
    for(auto r : robots_)
        r->padTrajectory(trajectoryLength_);
    // Update config.
    double endTime = TIMESTEP * (trajectoryLength_ - 1);

    Glib::RefPtr<Gtk::Adjustment> adjustment = timeSlider_->get_adjustment();
    adjustment->set_lower(0.0);
    adjustment->set_upper(endTime);
    adjustment->set_step_increment(TIMESTEP);
    timeSlider_->set_fill_level(endTime);
    drawingArea->queue_draw();
}
