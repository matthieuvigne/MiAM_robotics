#include "main_robot/Claw.h"
#include <unistd.h>



#define CLAW_STRAIGHT 2048

Claw::Claw(RailServo rail,
           int const& wristServoId,
           int const& clawServoId,
           int const& clawCloseValue,
           bool mirror):
    rail_(rail),
    wristServoId_(wristServoId),
    clawServoId_(clawServoId),
    clawCloseValue_(clawCloseValue),
    sign_((mirror ? -1 : 1))
{

}

void Claw::init(STSServoDriver *driver)
{
    driver_ = driver;
}

void Claw::openClaw()
{
    driver_->setTargetPosition(clawServoId_, clawCloseValue_ + sign_ * 150);

}

void Claw::closeClaw()
{

    driver_->setTargetPosition(clawServoId_, clawCloseValue_);
}


void Claw::move(ClawPosition const& clawPos)
{
    switch (clawPos)
    {
        case ClawPosition::FOLDED:
            driver_->setTargetPosition(wristServoId_, mirror(1000));
            break;
        case ClawPosition::SIDE:
            driver_->setTargetPosition(wristServoId_, mirror(2048));
            break;
        case ClawPosition::FORWARD:
        default:
            driver_->setTargetPosition(wristServoId_, mirror(3000));
            break;
    }
}

int Claw::mirror(int const& pos)
{
    return 2048 + sign_ * (pos - 2048);
}