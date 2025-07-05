#include "MotionControllerTestingViewer.h"
#include <sstream>


#define TABLE_WIDTH_MM 3000.0
#define TABLE_HEIGHT_MM 2000.0
#define TABLE_MARGIN_MM 0.0

double const OBSTACLE_SIZE = 200;

bool exitApp(GdkEventAny* event)
{
    Gtk::Main::quit();
    return true;
}

// Build window from Glade.
MotionControllerTestingViewer::MotionControllerTestingViewer(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade, std::string const& tableImagePath, MotionController *motionController, Logger *logger) :
    Gtk::Window(cobject),
    mmToCairo_(1.0),
    originX_(0.0),
    originY_(0.0),
    motionController_(motionController),
    logger_(logger)
{
    // Associate key press to entry.
    Gtk::Widget *window;
    refGlade->get_widget("mainWindow", window);

    window->signal_delete_event().connect(sigc::ptr_fun(&exitApp));

    // Associate widgets.
    refGlade->get_widget("infoLabel", infoLabel);
    refGlade->get_widget("toggleEndpoints", buttonSetEndpoints);
    refGlade->get_widget("toggleObstacle", buttonSetObstacle);
    refGlade->get_widget("checkStraight", buttonShowStraightLine);
    refGlade->get_widget("checkRounded", buttonShowRounded);
    refGlade->get_widget("checkMPC", buttonShowMPC);
    refGlade->get_widget("checkEEA", buttonEnforceEndAngle);
    refGlade->get_widget("checkBackward", buttonBackward);
    refGlade->get_widget("checkGrid", buttonShowGrid);

    Gtk::Button *button;
    refGlade->get_widget("runButton", button);

    button->signal_clicked().connect(sigc::mem_fun(this, &MotionControllerTestingViewer::runClicked));


    buttonShowStraightLine->signal_clicked().connect(sigc::mem_fun(this, &MotionControllerTestingViewer::refresh));
    buttonShowRounded->signal_clicked().connect(sigc::mem_fun(this, &MotionControllerTestingViewer::refresh));
    buttonShowMPC->signal_clicked().connect(sigc::mem_fun(this, &MotionControllerTestingViewer::refresh));
    buttonEnforceEndAngle->signal_clicked().connect(sigc::mem_fun(this, &MotionControllerTestingViewer::recompute));
    buttonBackward->signal_clicked().connect(sigc::mem_fun(this, &MotionControllerTestingViewer::recompute));
    buttonShowGrid->signal_clicked().connect(sigc::mem_fun(this, &MotionControllerTestingViewer::recompute));


    refGlade->get_widget("drawingArea", drawingArea);
    drawingArea->signal_draw().connect(sigc::mem_fun(this, &MotionControllerTestingViewer::redraw));
    drawingArea->set_events(Gdk::POINTER_MOTION_MASK | Gdk::BUTTON_PRESS_MASK  | Gdk::BUTTON_RELEASE_MASK | Gdk::BUTTON_MOTION_MASK | Gdk::SCROLL_MASK);

    drawingArea->signal_motion_notify_event().connect(sigc::mem_fun(this, &MotionControllerTestingViewer::mouseMove));
    drawingArea->signal_button_press_event().connect(sigc::mem_fun(this, &MotionControllerTestingViewer::mouseClicked));
    drawingArea->signal_scroll_event().connect(sigc::mem_fun(this, &MotionControllerTestingViewer::mouseScrolled));

    // Load images.
    tableImage = Gdk::Pixbuf::create_from_file(tableImagePath, -1, -1);


    Glib::signal_timeout().connect(sigc::mem_fun(*this, &MotionControllerTestingViewer::runAnimation), 50);
}


MotionControllerTestingViewer::~MotionControllerTestingViewer()
{

}


bool MotionControllerTestingViewer::runAnimation()
{
    if (isRunning_)
        drawingArea->queue_draw();
    return true;
}


void MotionControllerTestingViewer::setStartPosition(RobotPosition const& pos)
{
    startPosition_ = pos;
    recompute();
}


void MotionControllerTestingViewer::setEndPosition(RobotPosition const& pos)
{
    endPosition_ = pos;
    recompute();
}


void MotionControllerTestingViewer::refresh()
{
    isRunning_ = false;
    std::stringstream stream;
    stream << "Start:" << startPosition_;
    stream << "\nEnd:" << endPosition_;
    stream << "\nTrajectory duration:";
    stream << "\n\tstraight line:" << UNITTEST_POINTTURN_TRAJ.getDuration();
    stream << "\n\trounded:" << UNITTEST_ROUNDED_TRAJ.getDuration();
    stream << "\n\tMPC:" << mpcTrajectory_.getDuration();
    infoLabel->set_text(stream.str());
    drawingArea->queue_draw();
}

void drawRobot(const Cairo::RefPtr<Cairo::Context>& cr,
               RobotPosition const& pos,
               double r,
               double g,
               double b,
               double alpha)
{
    cr->save();
    cr->translate(pos.x, TABLE_HEIGHT_MM - pos.y);
    // Minus sign: indirect convention is used in Cairo.
    cr->rotate(-pos.theta);

    // Draw
    cr->move_to(-100, 0);
    cr->line_to(-100, -75);
    cr->line_to(100, 0);
    cr->line_to(-100, 75);
    cr->close_path();
    cr->set_source_rgba(r, g, b, alpha);
    cr->fill();
    cr->restore();
}


void drawPoints(const Cairo::RefPtr<Cairo::Context>& cr,
               std::vector<RobotPosition> const& positions,
               double r,
               double g,
               double b,
               double alpha)
{
    for (auto p: positions)
    {
        cr->move_to(p.x, (TABLE_HEIGHT_MM - p.y));
        cr->arc(p.x, (TABLE_HEIGHT_MM - p.y), 10, 0, 2 * M_PI - 0.1);
        cr->close_path();
        cr->set_source_rgba(r, g, b, alpha);
        cr->fill();
    }
}

void drawTrajectory(const Cairo::RefPtr<Cairo::Context>& cr,
               TrajectoryVector const& traj,
               double r,
               double g,
               double b,
               double alpha)
{
    double const samplingTime = 0.020;
    double t = 0;

    double pointX = traj.getCurrentPoint(t).position.x;
    double pointY = (TABLE_HEIGHT_MM - traj.getCurrentPoint(t).position.y);

    cr->move_to(pointX, pointY);
    while (t < traj.getDuration())
    {
        RobotPosition const pos = traj.getCurrentPoint(t).position;
        pointX =  pos.x;
        pointY =  (TABLE_HEIGHT_MM - pos.y);
        cr->line_to(pointX, pointY);
        t += samplingTime;
    }
    cr->set_source_rgba(1.0, 1.0, 1.0, alpha);
    cr->set_line_width(8.0);
    cr->stroke_preserve();
    cr->set_source_rgba(r, g, b, alpha);
    cr->set_line_width(4.0);
    cr->stroke();
}


bool MotionControllerTestingViewer::redraw(const Cairo::RefPtr<Cairo::Context>& cr)
{
    std::chrono::high_resolution_clock::time_point time = std::chrono::high_resolution_clock::now();
    double const currentTime = std::chrono::duration_cast<std::chrono::duration<double>>(time - animationStartTime_).count();

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
    cr->scale(mmToCairo_, mmToCairo_);

    // Draw obstacles
    for (auto obstacle: obstacles_)
    {
        cr->set_source_rgb(1.0, 0.0, 0.0);
        cr->arc(std::get<0>(obstacle).x, TABLE_HEIGHT_MM - std::get<0>(obstacle).y, OBSTACLE_SIZE, 0, 2 * M_PI - 0.01);
        cr->fill();
    }

    // Robot robot start / end
    drawRobot(cr, startPosition_, 0.8, 0.3, 0.3, 0.8);
    drawRobot(cr, endPosition_, 0.3, 0.8, 0.3, 0.8);
    if (buttonSetObstacle->get_active())
    {
        cr->set_source_rgba(0.3, 0.3, 0.3, 0.8);
        cr->arc(currentEditPosition_.x, TABLE_HEIGHT_MM - currentEditPosition_.y, OBSTACLE_SIZE, 0, 2 * M_PI - 0.01);
        cr->fill();
    }
    else
    {
        drawRobot(cr, currentEditPosition_, 0.3, 0.3, 0.3, 0.8);
    }
    std::vector<RobotPosition> pos;
    pos.push_back(startPosition_);
    pos.push_back(endPosition_);
    drawTrajectory(cr,
        miam::trajectory::computeTrajectoryStraightLineToPoint(motionController_->getCurrentTrajectoryParameters(), startPosition_, endPosition_),
        0.3, 0.3, 0.3, 0.8);


    drawPoints(cr, UNITTEST_ASTAR_POS, 0.0, 0.0, 1.0, 1.0);

    if (buttonShowStraightLine->get_active())
    {
        drawTrajectory(cr, UNITTEST_POINTTURN_TRAJ, 1.0, 0.0, 0.0, 1.0);

        if (isRunning_)
        {
            drawRobot(cr,
                UNITTEST_POINTTURN_TRAJ.getCurrentPoint(currentTime).position,
                1.0, 0.0, 0.0, 1.0);
        }
    }

    if (buttonShowRounded->get_active())
    {
        drawTrajectory(cr, UNITTEST_ROUNDED_TRAJ, 0.0, 0.0, 1.0, 1.0);

        if (isRunning_)
        {
            drawRobot(cr,
                UNITTEST_ROUNDED_TRAJ.getCurrentPoint(currentTime).position,
                0.0, 0.0, 1.0, 1.0);
        }
    }

    if (buttonShowMPC->get_active())
    {
        drawTrajectory(cr, mpcTrajectory_, 0.0, 1.0, 0.5, 1.0);

        if (isRunning_)
        {
            drawRobot(cr,
                mpcTrajectory_.getCurrentPoint(currentTime).position,
                0.0, 1.0, 0.5, 1.0);
        }
    }

    // Draw grid
    if (buttonShowGrid->get_active())
    {
        int const resolution = motionController_->map_.getGridSize();

        // Draw grid
        cr->set_source_rgb(0.0, 1.0, 0.0);
        for (int i = 0; i < motionController_->map_.rows(); i++)
        {
            cr->move_to(resolution * i, -10);
            cr->line_to(resolution * i, 2010);
            cr->stroke();
        }
        for (int i = 0; i < motionController_->map_.cols(); i++)
        {
            cr->move_to(-10, resolution * i);
            cr->line_to(3010, resolution * i);
            cr->stroke();
        }

        cr->set_source_rgba(1.0, 0.0, 0.0, 0.5);
        for (int i = 0; i < motionController_->map_.rows(); i++)
            for (int j = 0; j < motionController_->map_.cols(); j++)
            {
                if (motionController_->map_.coeff(i, j) == 1)
                {
                    cr->set_source_rgba(1.0, 0.0, 0.0, 0.5);
                    cr->rectangle(resolution * i, resolution * (motionController_->map_.cols() - j - 1), resolution, resolution);
                    cr->fill();
                }
                else if (motionController_->map_.coeff(i, j) == 2)
                {
                    cr->set_source_rgba(1.0, 1.0, 0.0, 0.5);
                    cr->rectangle(resolution * i, resolution * (motionController_->map_.cols() - j - 1), resolution, resolution);
                    cr->fill();
                }
            }
    }

    return true;
}


bool MotionControllerTestingViewer::mouseMove(GdkEventMotion* motion_event)
{
    double posX, posY;
    posX = (motion_event->x - originX_) / mmToCairo_;
    posY = TABLE_HEIGHT_MM - (motion_event->y - originY_) / mmToCairo_;

    currentEditPosition_.x = posX;
    currentEditPosition_.y = posY;

    drawingArea->queue_draw();
    return true;
}


bool MotionControllerTestingViewer::mouseClicked(GdkEventButton* buttonEvent)
{
    if (buttonSetEndpoints->get_active())
    {
        if (buttonEvent->button == 1)
            setStartPosition(currentEditPosition_);
        else if (buttonEvent->button == 3)
            setEndPosition(currentEditPosition_);
    }
    else if (buttonSetObstacle->get_active())
    {
        // Remove obstacle if they are too close.
        bool hasErased= false;
        for (auto iter = obstacles_.begin(); iter != obstacles_.end(); )
        {
            if ((currentEditPosition_ -std::get<0>(*iter)).norm() < OBSTACLE_SIZE)
            {
                iter = obstacles_.erase(iter);
                hasErased = true;
            }
            else
                iter++;
        }
        if (!hasErased)
            obstacles_.push_back(Obstacle(currentEditPosition_, OBSTACLE_SIZE));
        recompute();
    }
    drawingArea->queue_draw();
    return true;
}


bool MotionControllerTestingViewer::mouseScrolled(GdkEventScroll* scrollEvent)
{
    if (scrollEvent->direction == GDK_SCROLL_UP)
        currentEditPosition_.theta += 0.05;
    else
        currentEditPosition_.theta -= 0.05;
    drawingArea->queue_draw();
    return true;
}

void MotionControllerTestingViewer::runClicked()
{
    isRunning_ = true;
    animationStartTime_ = std::chrono::high_resolution_clock::now();
}

void MotionControllerTestingViewer::recompute()
{
    static int nIter = 0;
    isRunning_ = false;
    logger_->start("testOutput" + std::to_string(nIter) + ".miam");
    nIter++;

    motionController_->resetPosition(startPosition_);
    tf flags = tf::DEFAULT;
    if (!buttonEnforceEndAngle->get_active())
        flags = static_cast<tf>(flags | tf::IGNORE_END_ANGLE);
    if (buttonBackward->get_active())
        flags = static_cast<tf>(flags | tf::BACKWARD);

    motionController_->getGameState()->arePAMIMoving_ = true;
    mpcTrajectory_ = motionController_->computeMPCTrajectory(
        endPosition_,
        obstacles_,
        flags);
    refresh();
}