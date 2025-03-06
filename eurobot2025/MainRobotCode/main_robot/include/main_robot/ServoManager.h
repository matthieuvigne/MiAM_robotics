#ifndef MAIN_ROBOT_SERVO_MANAGER_H
#define MAIN_ROBOT_SERVO_MANAGER_H

#include "common/RobotInterface.h"

/// @brief Class responsible for handling servo action, including turret motion.

enum turretPosition{
    FRONT = 0,
    BACK = 1,
};

enum class ClawPosition{
    LOW_POSITION = 0,
    MEDIUM_POSITION = 1,
    HIGH_POSITION = 2,
    MEDIUM_POSITION_PLUS = 3,
    START_POSITION = 4,
    TURN_POSITION = 5
};

enum class ClawSide{
    FRONT,
    BACK,
};

namespace turret
{
    double const FRONT = 0.2;
    double const BACK = 0.2 + M_PI;
    enum state
    {
        CALIBRATING = 0,
        IDLE = 1,
        MOVING_CLOCKWISE = 2,
        MOVING_COUNTER_CLOCKWISE = 3
    };
}


class ServoManager
{
public:
    ServoManager() {}

    void init(RobotInterface *robot, bool const& isTurretAlreadyCalibrated);

    void foldBanner();
    void dropBanner();



private:
    RobotInterface *robot_;
    STSServoDriver *servos_;
};

#endif
