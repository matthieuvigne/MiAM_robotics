#ifndef MAIN_ROBOT_SERVO_MANAGER_H
#define MAIN_ROBOT_SERVO_MANAGER_H

#include "common/RobotInterface.h"

class ServoManager
{
public:
    ServoManager();
    void init(RobotInterface *robot);

    void cursorFold();
    void cursorUnfold();

private:
    RobotInterface *robot_;
    STSScheduler *servos_;

    RailServo *railX_;
    RailServo *railY_;

    int lastCloseTarget_back_R;
    int lastCloseTarget_back_L;
    int lastCloseTarget_front_R;
    int lastCloseTarget_front_L;
};

#endif
