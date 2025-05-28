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

    bool buildFrontTower();
    void dropBackCans(bool ground = true);

    void foldBanner();
    void dropBanner();

    void grabPlank();
    void releasePlank();
    void foldPlank();

    void prepareGrab(bool const& front);
    void clawsToMoveConfiguration(bool const& front);

    bool grab(bool const& front, bool const& frontFullGrab = true);
    bool checkGrab(bool const& front);
    bool areBothFrontSideClawsFull();

    void frontClawOpen();
    void frontClawClose();
    void backClawOpen();
    void backClawClose();

    void foldClaws();

    void grabBackTwoPlanks();
    void grabBackOnePlank();
    void releaseBackPlank();
    void foldBackPlank(bool init = false);

    void raiseFrontSideClaws();

// private:
    RobotInterface *robot_;
    STSServoDriver *servos_;
    RailManager railManager_;

    Claw frontRightClaw_;
    Claw frontLeftClaw_;

    RailServo backRail_;
    RailServo frontPlankRail_;
    RailServo frontCanRail_;

    int lastCloseTarget_back_R;
    int lastCloseTarget_back_L;
    int lastCloseTarget_front_R;
    int lastCloseTarget_front_L;
};

#endif
