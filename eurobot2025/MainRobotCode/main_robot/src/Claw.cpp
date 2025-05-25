#include "main_robot/Claw.h"
#include <unistd.h>
#include <iostream>


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

#define CLOSE_RANGE 130

void Claw::closeClaw()
{
    driver_->setTargetPosition(clawServoId_, clawOpenValue_ + sign_ * CLOSE_RANGE);
}

void Claw::foldClaw()
{
    driver_->setTargetPosition(clawServoId_, clawOpenValue_ + sign_ * 155);
}

bool Claw::isClawFull()
{
#ifdef SIMULATION
    return true;
#endif
    int const MIN_TH = 10;
    int const MAX_TH = 50;
    int const err = std::abs(driver_->getCurrentPosition(clawServoId_) - (clawOpenValue_ + sign_ * CLOSE_RANGE));
    std::cout << "Claw grab check: " << err << std::endl;
    return err > MIN_TH && err < MAX_TH;
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
