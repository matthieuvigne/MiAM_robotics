#include <common/MotionController.h>

RobotPosition MotionController::lidarPointToRobotPosition(LidarPoint const &point)
{
    // 1 Get the robot current position
    miam::RobotPosition const robot_position = this->getCurrentPosition();
    double const T_x_R = robot_position.x;
    double const T_y_R = robot_position.y;
    double const theta_T_R = robot_position.theta;

    // 2. Project the Lidar point within the table
    double const T_x_fi = T_x_R + point.r * std::cos(theta_T_R + point.theta);
    double const T_y_fi = T_y_R + point.r * std::sin(theta_T_R + point.theta);

    RobotPosition robotPosition;
    robotPosition.x = T_x_fi;
    robotPosition.y = T_y_fi;

    return robotPosition;
}

bool MotionController::isLidarPointWithinTable(LidarPoint const &point)
{

    RobotPosition robotPosition = lidarPointToRobotPosition(point);

    // 3. Check if the lidar point falls within the table
    if (robotPosition.x < table_dimensions::table_max_x and robotPosition.x > table_dimensions::table_min_x
        and robotPosition.y < table_dimensions::table_max_y and robotPosition.y > table_dimensions::table_min_y)
    {
        return true;
    }

    return false;
}


// void MotionController::setDetectedObstacles(std::vector<RobotPosition> detectedObstacles)
// {
//     detectedObstaclesMutex_.lock();
//     detectedObstacles_.clear();
//     copy(detectedObstacles.begin(), detectedObstacles.end(), back_inserter(detectedObstacles_));
//     detectedObstaclesMutex_.unlock();
// }

std::vector<Obstacle> MotionController::getDetectedObstacles(bool includePersistentObstacles)
{
    detectedObstaclesMutex_.lock();
    std::vector<Obstacle> detectedObstacles;
    copy(detectedObstacles_.begin(), detectedObstacles_.end(), back_inserter(detectedObstacles));
    if (includePersistentObstacles)
    {
        copy(persistentObstacles_.begin(), persistentObstacles_.end(), back_inserter(detectedObstacles));
    }
    detectedObstaclesMutex_.unlock();
    return detectedObstacles;
}

std::vector<Obstacle> MotionController::getPersistentObstacles()
{
    persistentObstaclesMutex_.lock();
    std::vector<Obstacle> persistentObstacles;
    copy(persistentObstacles_.begin(), persistentObstacles_.end(), back_inserter(persistentObstacles));
    persistentObstaclesMutex_.unlock();
    return persistentObstacles;
}

void MotionController::addPersistentObstacle(Obstacle obstacle)
{
    persistentObstaclesMutex_.lock();
    persistentObstacles_.push_back(obstacle);
    persistentObstaclesMutex_.unlock();
}

void MotionController::clearPersistentObstacles()
{
    persistentObstaclesMutex_.lock();
    persistentObstacles_.clear();
    persistentObstaclesMutex_.unlock();
}

void MotionController::popBackPersistentObstacles()
{
    persistentObstaclesMutex_.lock();
    persistentObstacles_.pop_back();
    persistentObstaclesMutex_.unlock();
}