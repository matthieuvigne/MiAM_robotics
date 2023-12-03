#ifndef MAIN_ROBOT_SERVO_MANAGER_H
#define MAIN_ROBOT_SERVO_MANAGER_H

#include "common/RobotInterface.h"

/// @brief Class responsible for handling servo action, including turret motion.

enum turretPosition{
    FRONT = 0,
    BACK = 1,
};


namespace turret
{
    double const FRONT = 0.2;
    double const BACK = 0.2 + M_PI;
    enum state
    {
        CALIBRATING,
        IDLE,
        MOVING_CLOCKWISE,
        MOVING_COUNTER_CLOCKWISE
    };
}


class ServoManager
{
public:
    ServoManager() {}

    void init(RobotInterface *robot);

    void openClaw(int const& clawId);
    void closeClaw(int const& clawId);

    /// @brief Move turret to specified position, in rad.
    void moveTurret(double const& turretPos);

    void waitForTurret();
    bool isTurretMotionless()
    {
        return turretState_ == turret::state::IDLE;
    }

private:
    RobotInterface *robot_;
    STSServoDriver *servos_;

    void turretMotionThread();

    turret::state turretState_{turret::state::CALIBRATING};
};

#endif
