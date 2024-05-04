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

    void openClaw(int const& clawId, bool const& halfOpen);
    void closeClaw(int const& clawId);

    void openClaws(bool const& front, bool const& halfOpen = false);
    void openAvailableClaws(bool const& front, GameState & gameState);

    void closeClaws(bool const& front);

    /// @brief Update the claw content, returning the number of plants that changed since
    ///        the last call.
    int updateClawContent(bool const& front, GameState & gameState);

    void setClawPosition(ClawSide const& side, ClawPosition const& claw_position);

    /// @brief Move turret to specified position, in rad.
    void moveTurret(double const& turretPos);

    void waitForTurret();
    bool isTurretMotionless()
    {
        return turretState_ == turret::state::IDLE;
    }

    double getTurretPosition() const;

    void raiseSolarPanelArm(bool const& medium = false);
    void lowerSolarPanelArm();
    void spinSolarPanel(bool const& isPlayingRightSide);
    void stopSolarPanel();

    void halfOpenElectromagnetArms();
    void openElectromagnetArms();
    void closeElectromagnetArms();

    void turnOnMagnets();
    void turnOffMagnets();


private:
    RobotInterface *robot_;
    STSServoDriver *servos_;

    void turretMotionThread(bool const& isTurretAlreadyCalibrated);

    turret::state turretState_{turret::state::CALIBRATING};

    double currentTurretPosition_{0.0};
    double targetTurretPosition_{0.0};
    int lastTurretPosition_{0};

    bool isClawServoClosed_[6] = {false, false, false, false, false, false};

    void updateTurretPosition();
};

#endif
