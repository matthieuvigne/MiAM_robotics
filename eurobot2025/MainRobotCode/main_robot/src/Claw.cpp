#include "main_robot/Claw.h"
#include <unistd.h>



#define CLAW_STRAIGHT 2048

Claw::Claw(RailServo rail,
           int const& wristServoId,
           int const& clawServoId,
           int const& clawOpenValue,
           bool mirror):
    rail_(rail),
    wristServoId_(wristServoId),
    clawServoId_(clawServoId),
    clawOpenValue_(clawOpenValue),
    sign_((mirror ? -1 : 1))
{

}

void Claw::init(STSServoDriver *driver)
{
    driver_ = driver;
}

void Claw::openClaw()
{
    driver_->setTargetPosition(clawServoId_, clawOpenValue_);

}

void Claw::closeClaw()
{

    driver_->setTargetPosition(clawServoId_, clawOpenValue_ + sign_ * 120);
}


void Claw::move(ClawPosition const& clawPos)
{
    switch (clawPos)
    {
        case ClawPosition::FOLDED:
            driver_->setTargetPosition(wristServoId_, mirror(2850));
            break;
        case ClawPosition::SIDE:
            driver_->setTargetPosition(wristServoId_, mirror(1000));
            break;
        case ClawPosition::FORWARD:
        default:
            driver_->setTargetPosition(wristServoId_, mirror(2048));
            break;
    }
}

int Claw::mirror(int const& pos)
{
    return 2048 + sign_ * (pos - 2048);
}
