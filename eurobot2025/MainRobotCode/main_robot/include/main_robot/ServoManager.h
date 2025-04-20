#ifndef MAIN_ROBOT_SERVO_MANAGER_H
#define MAIN_ROBOT_SERVO_MANAGER_H

#include "common/RobotInterface.h"

#include "Claw.h"
#include "RailServo.h"

class ServoManager
{
public:
    ServoManager();
    void init(RobotInterface *robot);

    bool isRailCalibDone() {return railManager_.areCalibrated();}
    void setRailsToInitPosition();

    void foldBanner();
    void dropBanner();

    void grabPlank();
    void releasePlank();
    void foldPlank();

    void prepareGrab(bool const& front);
    void grab(bool const& front);

    void frontClawOpen();
    void frontClawClose();
    void backClawOpen();
    void backClawClose();
// private:
    RobotInterface *robot_;
    STSServoDriver *servos_;
    RailManager railManager_;

    Claw frontRightClaw_;
    Claw frontLeftClaw_;

    RailServo backRail_;
    RailServo frontPlankRail_;
    RailServo frontCanRail_;
};

#endif
