/// \author Matthieu Vigne
/// \copyright GNU GPLv3
#include "ViewerRobot.h"
#include <string>


ViewerRobot::ViewerRobot(RobotParameters const& robotParameters,
                         std::string const& imageFileName,
                         AbstractStrategy *strategy,
                         double const& r, double const& g, double const& b,
                         std::string const& teleplotPrefix):
    RobotInterface(robotParameters),
    strategy_(strategy),
    r_(r),
    g_(g),
    b_(b),
    score_(0),
    teleplotPrefix_(teleplotPrefix),
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

    double robotImageSize = 500.0 * mmToCairo;
    double robotOriginX = p.x * mmToCairo;
    double robotOriginY = (TABLE_HEIGHT_MM - p.y) * mmToCairo;

    cr->save();
    cr->translate(robotOriginX, robotOriginY);
    // Minus sign: indirect convention is used in Cairo.
    cr->rotate(-p.theta);
    Gdk::Cairo::set_source_pixbuf(cr,
                                  image_->scale_simple(robotImageSize, robotImageSize, Gdk::INTERP_BILINEAR ),
                                  -robotImageSize / 2.0,
                                  -robotImageSize / 2.0);
    cr->paint();
    cr->restore();

    // Draw robot path.
    double pointX =  mmToCairo * trajectory_.at(0).position.x;
    double pointY =  mmToCairo * (TABLE_HEIGHT_MM - trajectory_.at(0).position.y);

    cr->move_to(pointX, pointY);
    for(unsigned long i = 0; i < trajectory_.size(); i+=10)
    {
        pointX =  mmToCairo * trajectory_.at(i).position.x;
        pointY =  mmToCairo * (TABLE_HEIGHT_MM - trajectory_.at(i).position.y);
        cr->line_to(pointX, pointY);
    }
    cr->set_source_rgb(1.0, 1.0, 1.0);
    cr->set_line_width(5.0);
    cr->stroke_preserve();
    cr->set_source_rgb(r_, g_, b_);
    cr->set_line_width(2.0);
    cr->stroke();

        // Draw robot target trajectory.
    if (!currentTrajectory_.empty())
    {
        double pointX =  mmToCairo * currentTrajectory_.at(0).position.x;
        double pointY =  mmToCairo * (TABLE_HEIGHT_MM - currentTrajectory_.at(0).position.y);

        cr->move_to(pointX, pointY);
        for(unsigned long i = 0; i < currentTrajectory_.size(); i+=10)
        {
            pointX =  mmToCairo * currentTrajectory_.at(i).position.x;
            pointY =  mmToCairo * (TABLE_HEIGHT_MM - currentTrajectory_.at(i).position.y);
            cr->line_to(pointX, pointY);
        }
        cr->set_source_rgb(1.0, 1.0, 1.0);
        cr->set_line_width(5.0);
        cr->stroke_preserve();
        cr->set_source_rgb(1.0-r_, 1.0-g_, 1.0-b_);
        cr->set_line_width(2.0);
        // std::vector<double> dashed3({1.0});
        // cr ->set_dash(dashed3, 1.0);
        cr->stroke();
    }
    
}


double ViewerRobot::getMatchTime()
{
    return simulationTime_;
}

void ViewerRobot::wait(double const& waitTimeS)
{
    if (simulationTime_ < 1e-3 && waitTimeS < 90)
        return;
    double const targetTime = simulationTime_ + waitTimeS;
    while (simulationTime_ < targetTime)
        usleep(1000);
}

RobotPosition ViewerRobot::getPosition()
{
    return simulationPosition_;
}

// Simulation function

void ViewerRobot::tick(double const& dt, double const& simulationTime, std::vector<Vector2 > const& obstaclesPosition)
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
    measurements_.lidarDetection.clear();
    for (auto& obstaclePosition : obstaclesPosition)
    {
        Vector2 relativePosition;
        relativePosition << simulationPosition_.x, simulationPosition_.y;
        relativePosition = obstaclePosition - relativePosition;
        Vector2 rotatedPosition;
        rotatedPosition(0) = std::cos(simulationPosition_.theta) * relativePosition(0) + std::sin(simulationPosition_.theta) * relativePosition(1);
        rotatedPosition(1) = -std::sin(simulationPosition_.theta) * relativePosition(0) + std::cos(simulationPosition_.theta) * relativePosition(1);

        DetectedRobot robot;
        robot.point = LidarPoint(relativePosition.norm(), std::atan2(rotatedPosition(1), rotatedPosition(0)));
        measurements_.lidarDetection.push_back(robot);
    }

    // Run iteration of controller
    motionTarget_ = motionController_.computeDrivetrainMotion(measurements_, dt, true);

    // Run periodic action
    strategy_->periodicAction();

    // Store result.
    ViewerTrajectoryPoint p;
    p.time = simulationTime;
    p.position = simulationPosition_;
    p.score = score_;

    // Miror.
    if (isPlayingRightSide_)
    {
        p.position.x = TABLE_WIDTH_MM - p.position.x;
        p.position.theta = M_PI - p.position.theta;
    }
    trajectory_.push_back(p);

    // Current trajectory
    currentTrajectory_.clear();
    for (int i = 0; i < motionController_.getCurrentTrajectories().size(); i++)
    {
        Trajectory* traj = motionController_.getCurrentTrajectories().at(i).get();
        double startAbscissa = (i == 0) ? motionController_.getCurvilinearAbscissa() : 0.0;
        for (double t = startAbscissa ; t < traj->getDuration(); t += 0.01)
        {
            ViewerTrajectoryPoint p;
            p.time = 0;
            p.position = traj->getCurrentPoint(t).position;
            p.score = 0;
            currentTrajectory_.push_back(p);
        }
    }

    // Update gui
    RobotGUIData data;
    data.detectedObstacles = motionController_.filteredDetectedObstacles_;
    data.currentPosition = p.position;
    data.batteryVoltage = 24.0;
    data.currentMatchTime = simulationTime_;
    data.state = robotstate::MATCH;
    data.score = score_;
    gui_.update(data);
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
    for (auto t : strategy_->createdThreads_)
        pthread_cancel(t);
    strategy_->createdThreads_.clear();
    strategy_->setup(this);
    simulationPosition_ = motionController_.getCurrentPosition();
    motionController_.init(simulationPosition_, teleplotPrefix_);

    std::thread tr = std::thread(&AbstractStrategy::match, strategy_);
    runningThread_ = tr.native_handle();
    tr.detach();
    gui_.unfullscreen();
    gui_.show_all();
}