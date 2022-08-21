/// \author Matthieu Vigne
/// \copyright GNU GPLv3
#include "ViewerRobot.h"
#include "Parameters.h"
#include <string>


ViewerRobot::ViewerRobot(std::string const& imageFileName,
                         Strategy const& strategy,
                         double const& r, double const& g, double const& b):
    handler_(&servoMock_),
    RobotInterface(&handler_),
    strategy_(strategy),
    r_(r),
    g_(g),
    b_(b),
    score_(0),
    isPlayingRightSide_(false)
{
    image_ = Gdk::Pixbuf::create_from_file(imageFileName, -1, -1);
}

void ViewerRobot::draw(const Cairo::RefPtr<Cairo::Context>& cr, double const& mmToCairo)
{
    if (trajectory_.size() == 0)
        return;
    // Draw robot at current point.
    ViewerTrajectoryPoint currentViewerPoint = trajectory_.back();
    RobotPosition p = currentViewerPoint.position;

    double robotImageSize = 500 * mmToCairo;
    double robotOriginX = p.x * mmToCairo;
    double robotOriginY = (2000 - p.y) * mmToCairo;

    cr->save();
    cr->translate(robotOriginX, robotOriginY);
    // Minus sign: indirect convention is used in Cairo.
    cr->rotate(-p.theta);
    Gdk::Cairo::set_source_pixbuf(cr,
                                  image_->scale_simple(robotImageSize, robotImageSize, Gdk::INTERP_BILINEAR ),
                                  -250 * mmToCairo,
                                  -250 * mmToCairo);
    cr->paint();
    cr->restore();

    // Draw robot path.
    double pointX =  mmToCairo * trajectory_.at(0).position.x;
    double pointY =  mmToCairo * (2000 - trajectory_.at(0).position.y);

    cr->move_to(pointX, pointY);
    for(unsigned long i = 0; i < trajectory_.size(); i+=10)
    {
        pointX =  mmToCairo * trajectory_.at(i).position.x;
        pointY =  mmToCairo * (2000 - trajectory_.at(i).position.y);
        cr->line_to(pointX, pointY);
    }
    cr->set_source_rgb(1.0, 1.0, 1.0);
    cr->set_line_width(5.0);
    cr->stroke_preserve();
    cr->set_source_rgb(r_, g_, b_);
    cr->set_line_width(2.0);
    cr->stroke();

    cr->set_line_width(3.0);
    double font_size = 55 * mmToCairo;
    cr->set_font_size(font_size);
    double const X = 3100;
    double const Y = 700;
    cr->save();
    cr->translate(mmToCairo * X, mmToCairo * Y);

    cr->move_to(2 * font_size, 0);
    cr->set_source_rgb(0.0, 0.0, 0.0);
    cr->show_text("Servo state");

    uint lenght = currentViewerPoint.servoState_.size();
    for (uint i = 0; i < lenght; i++)
    {
        cr->move_to(0, 1.2 * font_size * (1 + i));
        cr->set_source_rgb(0.0, 0.0, 0.0);
        cr->show_text(std::to_string(i));
        cr->move_to(1.8 * font_size, 1.2 * font_size * (1 + i));
        cr->set_source_rgb(1.0, 0.0, 0.0);
        int const servoState = int(currentViewerPoint.servoState_[i]);
        if (servoState == 0)
        {
            cr->set_source_rgb(1.0, 0.0, 0.0);
            cr->show_text("OFF");
        }
        else
        {
            cr->set_source_rgb(0.0, 0.5, 0.0);
            cr->show_text(std::to_string(servoState));
        }
        cr->move_to(5 * font_size, 1.2 * font_size * (1 + i));
        cr->set_source_rgb(0.0, 0.0, 0.0);
        cr->show_text(SERVO_NAMES[i]);
    }
    cr->move_to(0, - 1.2 * font_size);
    cr->set_source_rgb(0.0, 0.0, 0.0);
    cr->show_text("Pump: ");
    if (currentViewerPoint.isPumpOn_)
    {
        cr->set_source_rgb(0.0, 0.5, 0.0);
        cr->show_text("ON");
    }
    else
    {
        cr->set_source_rgb(1.0, 0.0, 0.0);
        cr->show_text("OFF");
    }
    cr->stroke();
    cr->restore();
}


double ViewerRobot::getMatchTime()
{
    return simulationTime_;
}

// void ViewerRobot::moveRail(double const& position)
// {
//     handler_.moveRail(100 * position);
// }

void ViewerRobot::wait(double const& waitTimeS)
{
    if (simulationTime_ < 1e-3 && waitTimeS < 90)
        return;
    double const targetTime = simulationTime_ + waitTimeS;
    while (simulationTime_ < targetTime)
        usleep(1000);
}

// Simulation function

void ViewerRobot::tick(double const& dt, double const& simulationTime, Vector2 const& obstaclePosition)
{
    simulationTime_ = simulationTime;
    // Integrate previous command, and compute sensor data.
    WheelSpeed speed(motionTarget_.motorSpeed * dt);
    kinematics_.integratePosition(speed, simulationPosition_, false);

    // Create measurements
    measurements_.encoderSpeed = kinematics_.inverseKinematics(kinematics_.forwardKinematics(speed), true);
    measurements_.encoderPosition += dt * measurements_.encoderSpeed.toVector();
    measurements_.motorSpeed = motionTarget_.motorSpeed;

    // Create lidar measurements.
    Vector2 relativePosition;
    relativePosition << simulationPosition_.x, simulationPosition_.y;
    relativePosition = obstaclePosition - relativePosition;
    Vector2 rotatedPosition;
    rotatedPosition(0) = std::cos(simulationPosition_.theta) * relativePosition(0) + std::sin(simulationPosition_.theta) * relativePosition(1);
    rotatedPosition(1) = -std::sin(simulationPosition_.theta) * relativePosition(0) + std::cos(simulationPosition_.theta) * relativePosition(1);

    DetectedRobot robot;
    robot.point = LidarPoint(relativePosition.norm(), std::atan2(rotatedPosition(1), rotatedPosition(0)));
    measurements_.lidarDetection.clear();
    measurements_.lidarDetection.push_back(robot);


    // Run iteration of controller
    motionTarget_ = motionController_.computeDrivetrainMotion(measurements_, dt, true, isPlayingRightSide_);

    // Store result.
    ViewerTrajectoryPoint p;
    p.time = simulationTime;
    p.position = simulationPosition_;
    p.score = score_;
    p.servoState_ = servoMock_.getState();

    // Miror.
    if (isPlayingRightSide_)
    {
        p.position.x = 3000 - p.position.x;
        p.position.theta = M_PI - p.position.theta;
    }
    trajectory_.push_back(p);
}

void ViewerRobot::reset(bool const& isPlayingRightSide)
{
    simulationTime_ = 0.0;
    trajectory_.clear();
    kinematics_ = motionController_.getKinematics();
    measurements_.encoderPosition = Vector2::Zero();
    isPlayingRightSide_ = isPlayingRightSide;

    if (runningThread_ > 0)
        pthread_cancel(runningThread_);
    for (auto t : strategy_.createdThreads_)
        pthread_cancel(t);
    strategy_.createdThreads_.clear();
    strategy_.setup(this);
    simulationPosition_ = motionController_.getCurrentPosition();

    std::thread tr = std::thread(&Strategy::match, &strategy_);
    runningThread_ = tr.native_handle();
    tr.detach();
}