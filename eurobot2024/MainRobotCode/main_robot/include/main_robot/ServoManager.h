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

    void init(RobotInterface *robot);

    void openClaw(int const& clawId);
    void closeClaw(int const& clawId);

    void openClaws(bool const& front);
    void closeClaws(bool const& front);

    void updateClawContent(bool const& front, GameState & gameState);

    void setClawPosition(ClawSide const& side, ClawPosition const& claw_position);

    /// @brief Move turret to specified position, in rad.
    void moveTurret(double const& turretPos);

    void waitForTurret();
    bool isTurretMotionless()
    {
        return turretState_ == turret::state::IDLE;
    }

    double getTurretPosition() const;

    void raiseSolarPanelArm();
    void lowerSolarPanelArm();
    void spinSolarPanel(bool const& spin);
private:
    RobotInterface *robot_;
    STSServoDriver *servos_;

    void turretMotionThread();

    turret::state turretState_{turret::state::CALIBRATING};

    double currentTurretPosition_{0.0};
    double targetTurretPosition_{0.0};
    int lastTurretPosition_{0};

    void updateTurretPosition();
};

#endif
