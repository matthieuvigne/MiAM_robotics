#include "main_robot/Claw.h"
#include <unistd.h>



#define CLAW_STRAIGHT 2048

Claw::Claw(STSServoDriver *driver,
           RailServo rail,
           int const& wristServoId,
           int const& clawServoId,
           int const& clawCloseValue, bool mirror):
    rail_(rail),
    driver_(driver),
    wristServoId_(wristServoId),
    clawServoId_(clawServoId),
    sign_((mirror ? -1 : 1)),
    clawCloseValue_(clawCloseValue)
{

}


void Claw::openClaw()
{
    driver_->setTargetPosition(clawServoId_, clawCloseValue_ + sign_ * 100);

}

void Claw::closeClaw()
{

    driver_->setTargetPosition(clawServoId_, clawCloseValue_);
}

void Claw::fold()
{
    driver_->setTargetPosition(wristServoId_, CLAW_STRAIGHT + sign_ * 700);
}

void Claw::unfold()
{
    driver_->setTargetPosition(wristServoId_, CLAW_STRAIGHT);
}
