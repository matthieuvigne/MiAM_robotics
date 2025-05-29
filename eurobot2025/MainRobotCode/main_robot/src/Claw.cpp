#include "main_robot/Claw.h"
#include <unistd.h>
#include <iostream>
#include <thread>

#define CLAW_STRAIGHT 2048
#define CLOSE_RANGE 110

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
    defaultCloseTarget_ = clawOpenValue_ + sign_ * CLOSE_RANGE;
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
    driver_->setTargetPosition(clawServoId_, defaultCloseTarget_);
    lastCloseTarget_ = defaultCloseTarget_;
}

void Claw::foldClaw()
{
    driver_->setTargetPosition(clawServoId_, clawOpenValue_ + sign_ * 130);
}

bool Claw::isClawFull(int &error)
{
#ifdef SIMULATION
    return true;
#endif
    int const MIN_TH = 10;
    int const MAX_TH = 50;

    error = std::abs(driver_->getCurrentPosition(clawServoId_) - lastCloseTarget_);

    bool success = error < MAX_TH && error > MIN_TH;

    int const MAX_ATTEMPT = 2;
    int const OFFSET_CLOSE_RANGE = 10;

    // If last target is not default, then do not attempt to compensate
    if (lastCloseTarget_ == defaultCloseTarget_)
    {
        for (uint currentAttempt=0; currentAttempt < MAX_ATTEMPT; currentAttempt++)
        {
            std::cout << "[Claw] Grab check: lastCloseTarget_=" << lastCloseTarget_ << std::endl;

            if (success)
            {
                break;
            }

            int newValue = clawOpenValue_ + sign_ * CLOSE_RANGE + (currentAttempt+1) * sign_ * OFFSET_CLOSE_RANGE;

            std::cout << "[Claw] Grab check " << clawServoId_ << " iter " << currentAttempt << " newValue " << newValue << " error " << error << std::endl;
            driver_->setTargetPosition(clawServoId_, newValue);
            lastCloseTarget_ = newValue;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            error = std::abs(driver_->getCurrentPosition(clawServoId_) - newValue);
            // TODO here we are grabbing even more if error > MAX_TH! we should avoid that
            success = error < MAX_TH && error > MIN_TH;
            std::cout << "[Claw] Current error: " << error << std::endl;
        }
    }

    return success;
}

void Claw::move(ClawPosition const& clawPos)
{
    switch (clawPos)
    {
        case ClawPosition::FOLDED:
            driver_->setTargetPosition(wristServoId_, mirror(2750));
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
