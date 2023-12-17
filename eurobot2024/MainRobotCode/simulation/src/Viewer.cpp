/// \author Matthieu Vigne
/// \copyright GNU GPLv3

#include "Viewer.h"
#include <iomanip>
#include <sstream>

double const ROBOT_DT = 1.0e-9 * ROBOT_UPDATE_PERIOD;
double const MATCH_TIME = 100.0;


bool exitApp(GdkEventAny* event)
{
    Gtk::Main::quit();
    return true;
}

// Build window from Glade.
Viewer::Viewer(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade, std::string const& tableImagePath) :
    Gtk::Window(cobject),
    mmToCairo_(1.0),
    originX_(0.0),
    originY_(0.0)
{
    // Associate key press to entry.
    Gtk::Widget *window;
    refGlade->get_widget("mainWindow", window);

    // window->signal_destroy().connect(Gtk::Main::quit());
    window->signal_delete_event().connect(sigc::ptr_fun(&exitApp));

    // Associate widgets.
    refGlade->get_widget("mousePositionLabel", mousePositionLabel);
    refGlade->get_widget("timeLabel", timeLabel);
    refGlade->get_widget("scoreLabel", scoreLabel);
    refGlade->get_widget("progressBar", progressBar);
    refGlade->get_widget("switchButton", switchButton);
    refGlade->get_widget("simulationRatioSpin", simulationRatioSpin);
    simulationRatioSpin->signal_value_changed().connect(sigc::mem_fun(this, &Viewer::updateTimeRatio));
    refGlade->get_widget("drawingArea", drawingArea);
    drawingArea->signal_draw().connect(sigc::mem_fun(this, &Viewer::redraw));

    drawingArea->set_events(Gdk::POINTER_MOTION_MASK | Gdk::BUTTON_PRESS_MASK  | Gdk::BUTTON_RELEASE_MASK | Gdk::BUTTON_MOTION_MASK | Gdk::SCROLL_MASK);

    drawingArea->signal_motion_notify_event().connect(sigc::mem_fun(this, &Viewer::mouseMove));
    drawingArea->signal_button_press_event().connect(sigc::mem_fun(this, &Viewer::clickObstacle));
    drawingArea->signal_button_release_event().connect(sigc::mem_fun(this, &Viewer::clickObstacle));

    Gtk::Button *button;
    refGlade->get_widget("play", button);
    button->signal_clicked().connect(sigc::mem_fun(this, &Viewer::playClicked));
    refGlade->get_widget("pause", button);
    button->signal_clicked().connect(sigc::mem_fun(this, &Viewer::pauseClicked));
    refGlade->get_widget("reset", button);
    button->signal_clicked().connect(sigc::mem_fun(this, &Viewer::resetClicked));

    // Load images.
    tableImage = Gdk::Pixbuf::create_from_file(tableImagePath, -1, -1);

    Glib::signal_timeout().connect(sigc::mem_fun(*this,&Viewer::runSimulation), 50);
}


Viewer::~Viewer()
{

}

bool Viewer::runSimulation()
{
    // Increment currentTrajectoryIndex_ based on time elapsed: increment as much as needed to catch back with real
    // clock.
    std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
    double realElapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastTime_).count();
    while (isRunning_ && realElapsedTime > ROBOT_DT / simulationTimeRatio_)
    {
        // std::cout << "Running -- simulationTime_: " << simulationTime_ << std::endl;
        simulationTime_ += ROBOT_DT;
        for(unsigned int i = 0; i < robots_.size(); i++)
        {
            ViewerRobot* r = robots_.at(i);
            // get the other robots
            std::vector<Vector2 > obstaclesPosition;
            for (unsigned int j = 0; j < robots_.size(); j++)
            {
                if (j != i)
                {
                    Vector2 v;
                    v << robots_.at(j)->getPosition().x, robots_.at(j)->getPosition().y;
                    obstaclesPosition.push_back(v);
                }
            }
            // obstacles
            obstaclesPosition.push_back(obstaclePosition_);
            obstaclesPosition.push_back(obstacle2Position_);

            SimulatorData data;
            data.isStartingSwitchPluggedIn = switchButton->get_active();
            data.obstaclesPosition = obstaclesPosition;
            r->tick(data);
        }
        // Reset time on match start.
        static double lastMatchTime = 0.0;
        if (robots_.at(0)->getMatchTime() > 0 && lastMatchTime == 0)
            simulationTime_ = 0;
        lastMatchTime = robots_.at(0)->getMatchTime();

        realElapsedTime -= ROBOT_DT / simulationTimeRatio_;
        lastTime_ += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(ROBOT_DT / simulationTimeRatio_));
        if (simulationTime_ > MATCH_TIME && robots_.at(0)->getMatchTime() > 0)
        {
            isRunning_ = false;
            simulationTime_ = MATCH_TIME;
        }
    }
    if (!isRunning_)
        lastTime_ = currentTime;

    progressBar->set_fraction(std::min(simulationTime_ / MATCH_TIME, 1.0));
    std::stringstream stream;
    stream << "Time: " << std::fixed << std::setprecision(3) << simulationTime_;
    progressBar->set_text(stream.str());

    drawingArea->queue_draw();
    return true;
}


void Viewer::start()
{
    Glib::signal_timeout().connect(sigc::mem_fun(*this,&Viewer::runSimulation), 50);
    resetClicked();
}

void Viewer::addRobot(ViewerRobot & robot)
{
    robots_.push_back(&robot);
}

bool Viewer::redraw(const Cairo::RefPtr<Cairo::Context>& cr)
{
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
    mmToCairo_ = newWidth / TABLE_WIDTH_MM;

    originX_ = (widgetWidth - newWidth) / 2 + TABLE_MARGIN_MM * mmToCairo_;
    originY_ = (widgetHeight - newHeight) / 2 + TABLE_MARGIN_MM * mmToCairo_;

    cr->translate(originX_, originY_);

    int score = 0;
    for(auto robot : robots_)
    {
        robot->draw(cr, mmToCairo_);
    }

    // Draw obstacle.
    cr->set_source_rgb(1.0, 0.0, 0.0);
    cr->arc(mmToCairo_ * obstaclePosition_(0), mmToCairo_ * (TABLE_HEIGHT_MM - obstaclePosition_(1)), mmToCairo_ * 200, 0, 2 * M_PI - 0.1);
    cr->fill();

    // Draw obstacle2.
    cr->set_source_rgb(0.0, 0.0, 1.0);
    cr->arc(mmToCairo_ * obstacle2Position_(0), mmToCairo_ * (TABLE_HEIGHT_MM - obstacle2Position_(1)), mmToCairo_ * 200, 0, 2 * M_PI - 0.1);
    cr->fill();

    // Update labels.
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << simulationTime_;
    timeLabel->set_text("Time: " + stream.str());
    scoreLabel->set_text("Score: " + std::to_string(score));

    // Draw game state.
    cr->scale(mmToCairo_, mmToCairo_);
    robots_[0]->gameState_.draw(cr, robots_[0]->getPosition());

    return true;
}


bool Viewer::mouseMove(GdkEventMotion* motion_event)
{
    double posX, posY;
    posX = (motion_event->x - originX_) / mmToCairo_;
    posY = TABLE_HEIGHT_MM - (motion_event->y - originY_) / mmToCairo_;

    if (motion_event->state & Gdk::BUTTON1_MASK)
    {
        obstaclePosition_(0) = posX;
        obstaclePosition_(1) = posY;
    }
    else if (motion_event->state & Gdk::BUTTON3_MASK)
    {
        obstacle2Position_(0) = posX;
        obstacle2Position_(1) = posY;
    }
    mousePositionLabel->set_text("(" + std::to_string(static_cast<int>(posX)) + ", " + std::to_string(static_cast<int>(posY)) + ")");
    drawingArea->queue_draw();
    return true;
}


bool Viewer::clickObstacle(GdkEventButton* motion_event)
{

    if (motion_event->button == 1)
    {
        obstaclePosition_(0) = motion_event->x;
        obstaclePosition_(1) = motion_event->y;
        // Convert coordinates to table coordinates
        obstaclePosition_(0) = (obstaclePosition_(0) - originX_) / mmToCairo_;
        obstaclePosition_(1) = TABLE_HEIGHT_MM - (obstaclePosition_(1) - originY_) / mmToCairo_;

    }
    else if (motion_event->button == 3)
    {
        obstacle2Position_(0) = motion_event->x;
        obstacle2Position_(1) = motion_event->y;
        // Convert coordinates to table coordinates
        obstacle2Position_(0) = (obstacle2Position_(0) - originX_) / mmToCairo_;
        obstacle2Position_(1) = TABLE_HEIGHT_MM - (obstacle2Position_(1) - originY_) / mmToCairo_;
    }

    drawingArea->queue_draw();
    return true;
}

void Viewer::playClicked()
{
    isRunning_ = true;
}

void Viewer::pauseClicked()
{
    isRunning_ = false;
}

void Viewer::resetClicked()
{
    isRunning_ = false;
    simulationTime_ = 0.0;
    for(auto r : robots_)
        r->reset(false);
}

void Viewer::updateTimeRatio()
{
    simulationTimeRatio_ = simulationRatioSpin->get_value();
}