/// \author Matthieu Vigne
/// \copyright GNU GPLv3
#include "ViewerRobot.h"
#include "Parameters.h"
#include <string>


ViewerRobot::ViewerRobot(std::string const& imageFileName,
                         Strategy const& strategy,
                         double const& r, double const& g, double const& b):
    strategy_(strategy),
    handler_(&servoMock_),
    r_(r),
    g_(g),
    b_(b),
    score_(0),
    trajectoryFollowingStatus_(true),
    obstacleX_(0.0),
    obstacleY_(0.0),
    obstacleSize_(0.0),
    isPlayingRightSide_(false)
{
    image_ = Gdk::Pixbuf::create_from_file(imageFileName, -1, -1);
}


RobotPosition ViewerRobot::getCurrentPosition()
{
    if (trajectory_.size() == 0)
        return RobotPosition(0.0, 0.0, 0.0);
    RobotPosition p = trajectory_.back().position;
    if (isPlayingRightSide_)
    {
        p.x = 3000 - p.x;
        p.theta = M_PI - p.theta;
    }

    return p;
}


bool ViewerRobot::followTrajectory(miam::trajectory::Trajectory *traj)
{
    double currentTrajectoryTime = 0.0;
    ViewerTrajectoryPoint viewerPoint = trajectory_.back();

    miam::trajectory::TrajectoryPoint currentPoint;
    while(currentTrajectoryTime < traj->getDuration())
    {
        currentPoint = traj->getCurrentPoint(currentTrajectoryTime);
        // Check obstacle.
        double distance = std::sqrt((currentPoint.position.x - obstacleX_) * (currentPoint.position.x - obstacleX_) + (currentPoint.position.y - obstacleY_) * (currentPoint.position.y - obstacleY_));
        if (distance < obstacleSize_ + 150.0)
            return false;

        viewerPoint.time += TIMESTEP;
        viewerPoint.position = currentPoint.position;
        viewerPoint.linearVelocity = currentPoint.linearVelocity;
        viewerPoint.angularVelocity = currentPoint.angularVelocity;
        viewerPoint.score = score_;
        viewerPoint.servoState_ = servoMock_.getState();
        viewerPoint.isPumpOn_ = handler_.isPumpOn_;
        if (isPlayingRightSide_)
        {
            viewerPoint.position.x = 3000 - viewerPoint.position.x;
            viewerPoint.position.theta = M_PI - viewerPoint.position.theta;
            viewerPoint.angularVelocity = -currentPoint.angularVelocity;
        }

        trajectory_.push_back(viewerPoint);

        if (!isRobotPositionInit_)
        {
            isRobotPositionInit_ = true;
            for (uint i = 0; i < trajectory_.size(); i++)
                trajectory_.at(i).position = viewerPoint.position;
        }
        currentTrajectoryTime += TIMESTEP;
    }
    return true;
}

bool ViewerRobot::setTrajectoryToFollow(std::vector<std::shared_ptr<miam::trajectory::Trajectory>> const& trajectories)
{
    trajectoryFollowingStatus_ = true;
    for(std::shared_ptr<miam::trajectory::Trajectory> t: trajectories)
        if(!followTrajectory(t.get()))
            trajectoryFollowingStatus_ = false;
    return trajectoryFollowingStatus_;
}

bool ViewerRobot::waitForTrajectoryFinished()
{
    return trajectoryFollowingStatus_;
}

void ViewerRobot::resetPosition(RobotPosition const& resetPosition, bool const& resetX, bool const& resetY, bool const& resetTheta)
{
    ViewerTrajectoryPoint p;
    if (trajectory_.empty())
        p.time = 0.0;
    else
        p.time = trajectory_.back().time + TIMESTEP;
    if (resetX)
    {
        p.position.x = resetPosition.x;
        if (isPlayingRightSide_)
            p.position.x = 3000 - p.position.x;
    }
    if (resetY)
        p.position.y = resetPosition.y;
    if (resetTheta)
    {
        p.position.theta = resetPosition.theta;
        if (isPlayingRightSide_)
            p.position.theta = M_PI - p.position.theta;
    }

    p.linearVelocity = 0.0;
    p.angularVelocity = 0.0;
    p.servoState_ = servoMock_.getState();
    p.isPumpOn_ = handler_.isPumpOn_;

    trajectory_.push_back(p);
}

void ViewerRobot::draw(const Cairo::RefPtr<Cairo::Context>& cr, double const& mmToCairo, int const& currentIndex)
{
    // Draw robot at current point.
    ViewerTrajectoryPoint currentViewerPoint = trajectory_.at(currentIndex);
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
    for(int i = 0; i < currentIndex; i+=10)
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


int ViewerRobot::getTrajectoryLength()
{
   return trajectory_.size();
}

void ViewerRobot::padTrajectory(int const& desiredLength)
{
    ViewerTrajectoryPoint lastPoint;
    if (!trajectory_.empty() )
        lastPoint = trajectory_.back();
    lastPoint.linearVelocity = 0.0;
    lastPoint.angularVelocity = 0.0;
    lastPoint.servoState_ = servoMock_.getState();
    lastPoint.isPumpOn_ = handler_.isPumpOn_;
    while(static_cast<int>(trajectory_.size()) < desiredLength)
    {
        lastPoint.time += TIMESTEP;
        trajectory_.push_back(lastPoint);
    }
}

ViewerTrajectoryPoint ViewerRobot::getViewerPoint(int const& index)
{
    return trajectory_.at(index);
}

void ViewerRobot::clearScore()
{
    score_ = 0;
}

void ViewerRobot::updateScore(int const& scoreIncrement)
{
    score_ += scoreIncrement;
    if (!trajectory_.empty())
    {
        trajectory_.back().score = score_;
    }
}

double ViewerRobot::getMatchTime()
{
    return trajectory_.back().time;
}

void ViewerRobot::moveRail(double const& position)
{
    handler_.moveRail(100 * position);
}

void ViewerRobot::wait(double const& waitTimeS)
{
    int const nPoints = waitTimeS / TIMESTEP;
    padTrajectory(nPoints);
}

void ViewerRobot::recomputeStrategy(int const& obstacleX, int const& obstacleY, int const& obstacleSize, bool const& isPlayingRightSide)
{
    isPlayingRightSide_ = isPlayingRightSide;
    obstacleX_ = obstacleX;
    obstacleY_ = obstacleY;
    obstacleSize_ = obstacleSize;
    trajectory_.clear();
    servoMock_.init("", 0);
    clearScore();
    isRobotPositionInit_ = false;
    strategy_.setup(this, &this->handler_);
    strategy_.match();
}


int ViewerRobot::getScore(int const& index)
{
    return trajectory_[index].score;
}

