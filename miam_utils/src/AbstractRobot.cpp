/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include <unistd.h>
#include "miam_utils/AbstractRobot.h"

AbstractRobot::AbstractRobot():
    currentPosition_(),
    currentBaseSpeed_(),
    trajectoryPoint_(),
    currentTime_(0.0),
    newTrajectories_(),
    currentTrajectories_(),
    trajectoryStartTime_(0.0),
    lastTrajectoryFollowingCallTime_(0.0),
    wasTrajectoryFollowingSuccessful_(true)
{
    motorSpeed_.push_back(0.0);
    motorSpeed_.push_back(0.0);
    motorPosition_.push_back(0);
    motorPosition_.push_back(0);
}


miam::RobotPosition AbstractRobot::getCurrentPosition()
{
    return currentPosition_.get();
}


BaseSpeed AbstractRobot::getCurrentBaseSpeed()
{
    return currentBaseSpeed_;
}


void AbstractRobot::resetPosition(miam::RobotPosition const& resetPosition, bool const& resetX, bool const& resetY, bool const& resetTheta)
{
    miam::RobotPosition position = currentPosition_.get();
    if(resetX)
        position.x = resetPosition.x;
    if(resetY)
        position.y = resetPosition.y;
    if(resetTheta)
        position.theta = resetPosition.theta;
    currentPosition_.set(position);
}


bool AbstractRobot::setTrajectoryToFollow(std::vector<std::shared_ptr<miam::trajectory::Trajectory>> const& trajectories, bool const& matchEndLock)
{
    newTrajectoryMutex_.lock();
    newTrajectories_ = trajectories;
    wasTrajectoryFollowingSuccessful_ = true;
    if (!matchEndLock)
        newTrajectoryMutex_.unlock();
    return true;
}


bool AbstractRobot::waitForTrajectoryFinished()
{
    while(!isTrajectoryFinished())
        usleep(15000);
    return wasTrajectoryFollowingSuccessful_;
}


bool AbstractRobot::isTrajectoryFinished()
{
    return (currentTrajectories_.size() == 0 && newTrajectories_.size() == 0);
}

bool AbstractRobot::wasTrajectoryFollowingSuccessful()
{
    return wasTrajectoryFollowingSuccessful_;
}
