#ifndef MAIN_ROBOT_SERVO_MANAGER_H
#define MAIN_ROBOT_SERVO_MANAGER_H

#include "common/RobotInterface.h"

enum ArmPosition {
    CALIBRATE
};

enum RailPosition {
    STARTUP
};

enum class Side : int {
    RIGHT = 0,
    LEFT = 1
};

class ServoManager
{
public:
    ServoManager();
    void init(RobotInterface *robot);

    void cursorFold();
    void cursorUnfold();

    void bedFold();
    void bedUnfold();

    void moveRails(RailPosition const& position);
    bool areRailsMoving();

    void moveArm(ArmPosition const& position);

    /// Transate suction, 0=close, 1=full open
    void translateSuction(Side const side, double const ratio =0.0);

    void pumpOn(Side const side);
    void pumpOff(Side const side);
private:
    RobotInterface *robot_;
    STSScheduler *servos_;

    std::shared_ptr<RailServo> railX_;
    std::shared_ptr<RailServo> railY_;

    int lastCloseTarget_back_R;
    int lastCloseTarget_back_L;
    int lastCloseTarget_front_R;
    int lastCloseTarget_front_L;
};

#endif
