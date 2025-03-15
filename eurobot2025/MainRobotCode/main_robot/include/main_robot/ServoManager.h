#ifndef MAIN_ROBOT_SERVO_MANAGER_H
#define MAIN_ROBOT_SERVO_MANAGER_H

#include "common/RobotInterface.h"

class ServoManager
{
public:
    ServoManager() = default;
    void init(RobotInterface *robot, bool const& isTurretAlreadyCalibrated);

    void foldBanner();
    void dropBanner();

private:
    RobotInterface *robot_;
    STSServoDriver *servos_;
};

#endif
