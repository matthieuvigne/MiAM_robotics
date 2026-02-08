#ifndef MAIN_ROBOT_SERVO_MANAGER_H
#define MAIN_ROBOT_SERVO_MANAGER_H

#include "common/RobotInterface.h"

enum ArmPosition {
    CALIBRATE
};

enum RailPosition {
    STARTUP
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
