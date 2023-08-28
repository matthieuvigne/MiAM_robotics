/// \author Matthieu Vigne
/// \copyright GNU GPLv3
#include "ViewerRobot.h"
#include <string>

double const ROBOT_DT = 1.0e-9 * ROBOT_UPDATE_PERIOD;


ViewerRobot::ViewerRobot(RobotParameters const& robotParameters,
                         std::string const& imageFileName,
                         AbstractStrategy *strategy,
                         double const& r, double const& g, double const& b,
                         std::string const& teleplotPrefix):
    RobotInterface(robotParameters,
                   &gui_,
                   strategy,
                   false,
                   teleplotPrefix),
    r_(r),
    g_(g),
    b_(b)
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

void ViewerRobot::stopMotors()
{
    // Empty on purpose
}

bool ViewerRobot::initSystem()
{
    return true; // Simulation has no hardware so it is always ready
}

void ViewerRobot::wait(double const& waitTimeS)
{
    if (metronome_.getElapsedTime() < 1e-3 && waitTimeS < 90)
        return;
    double const targetTime = metronome_.getElapsedTime() + waitTimeS;
    while (metronome_.getElapsedTime() < targetTime)
        usleep(1000);
}

void ViewerRobot::updateSensorData()
{
    measurements_.drivetrainMeasurements.encoderSpeed = kinematics_.inverseKinematics(kinematics_.forwardKinematics(simulationSpeed_), true);
    simulatedEncoders_ += measurements_.drivetrainMeasurements.encoderSpeed.toVector();
    measurements_.drivetrainMeasurements.encoderPosition = simulatedEncoders_;
    measurements_.drivetrainMeasurements.motorSpeed = motionTarget_.motorSpeed;
    measurements_.batteryVoltage = simulatorData_.batteryVoltage;

    // Create lidar measurements.
    measurements_.drivetrainMeasurements.lidarDetection.clear();
    for (auto& obstaclePosition : simulatorData_.obstaclesPosition)
    {
        Vector2 relativePosition;
        relativePosition << simulationPosition_.x, simulationPosition_.y;
        relativePosition = obstaclePosition - relativePosition;
        Vector2 rotatedPosition;
        rotatedPosition(0) = std::cos(simulationPosition_.theta) * relativePosition(0) + std::sin(simulationPosition_.theta) * relativePosition(1);
        rotatedPosition(1) = -std::sin(simulationPosition_.theta) * relativePosition(0) + std::cos(simulationPosition_.theta) * relativePosition(1);

        DetectedRobot robot;
        robot.point = LidarPoint(relativePosition.norm(), std::atan2(rotatedPosition(1), rotatedPosition(0)));
        measurements_.drivetrainMeasurements.lidarDetection.push_back(robot);
    }
}

void ViewerRobot::applyMotorTarget(DrivetrainTarget const& target)
{
    // Handle init
    if (guiState_.state == robotstate::MATCH && lastState_ != robotstate::MATCH)
    {
        simulationPosition_ = motionController_.getCurrentPosition();

        if (isPlayingRightSide())
        {
            simulationPosition_.x = TABLE_WIDTH_MM - simulationPosition_.x;
            simulationPosition_.theta = M_PI - simulationPosition_.theta;
        }
        trajectory_.clear();
    }
    lastState_ = guiState_.state ;
    simulationSpeed_ = WheelSpeed(ROBOT_DT * target.motorSpeed);
    kinematics_.integratePosition(simulationSpeed_, simulationPosition_, false);

    simulationMutex_.lock();
    hasRobotRun_ = true;
    simulationMutex_.unlock();
    cv_.notify_all();
}

void ViewerRobot::matchEnd()
{
    hasMatchEnded_ = true;
    simulationMutex_.lock();
    hasRobotRun_ = true;
    simulationMutex_.unlock();
    cv_.notify_all();
}

// Simulation function

RobotPosition ViewerRobot::getPosition()
{
    return simulationPosition_;
}

void ViewerRobot::tick(SimulatorData const& simulationData)
{
    // Store simulation data
    simulatorData_ = simulationData;

    if (!hasMatchEnded_)
    {
        // Run one low-level iteration
        metronome_.tick();

        // Wait for low-level loop to run.
        std::unique_lock<std::mutex> lck(simulationMutex_);
        if (!hasRobotRun_)
            cv_.wait(lck);
        hasRobotRun_ = false;
    }

    // Store result.
    ViewerTrajectoryPoint p;
    p.time = metronome_.getElapsedTime();
    p.position = simulationPosition_;
    p.score = score_;

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
            // Miror.
            if (isPlayingRightSide())
            {
                p.position.x = TABLE_WIDTH_MM - p.position.x;
                p.position.theta = M_PI - p.position.theta;
            }
            p.score = 0;
            currentTrajectory_.push_back(p);
        }
    }
}

void ViewerRobot::reset(bool const& isPlayingRightSide)
{
    metronome_.reset();
    trajectory_.clear();
    kinematics_ = motionController_.getKinematics();
    simulatedEncoders_ = Vector2::Zero();
    hasMatchEnded_ = false;

    for (auto t : strategy_->createdThreads_)
        pthread_cancel(t);
    strategy_->createdThreads_.clear();
    metronome_.reset();
    if (lowLevelThread_.joinable())
        lowLevelThread_.join();
    for (auto t: runningThreads_)
        pthread_cancel(t);
    runningThreads_.clear();

    lowLevelThread_ = std::thread(&RobotInterface::lowLevelLoop, this);
    gui_.unfullscreen();
    gui_.show_all();
}


bool ViewerRobot::isStartingSwitchPluggedIn() const
{
    return simulatorData_.isStartingSwitchPluggedIn;
}