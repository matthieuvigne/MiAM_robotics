#include "miam_utils/drivers/RMDXController.h"

#include <iostream>
#include <algorithm>


RMDXController::RMDXController(RMDX *driver,
                               unsigned char const& motorId,
                               double const& Kp,
                               double const& Ki,
                               double const& maxOutput,
                               double const& filterCutoff,
                               double const& maxFeedforward,
                               double const& maxAcceleration):
    position_(0),
    velocity_(0),
    rawVelocity_(0),
    targetCurrent_(0),
    current_(0),
    driver_(driver),
    motorId_(motorId),
    Kp_(Kp),
    Ki_(Ki),
    integralValue_(0),
    maxOutput_(maxOutput),
    isStopped_(true),
    lowPass_(filterCutoff),
    maxFeedforward_(maxFeedforward),
    maxAcceleration_(maxAcceleration)
{
    std::cout << maxAcceleration_ << std::endl;

}

double RMDXController::sendTarget(double const& targetVelocity, double const& dt)
{
    // Get position from motor, compute velocity
    double const newPos = driver_->getCurrentPosition(motorId_);
    if (isStopped_)
    {
        position_ = newPos;
        integralValue_ = 0;
        clampedTargetVelocity_ = 0.0;
    }

    // Process only if valid signal is obtained
    if (driver_->lastError_ == RMDX::ErrorCode::OK)
    {
        rawVelocity_ = (newPos - position_) / dt;
        // Handle communication problems: only perform computation when a valid position is returned.
        if (std::abs(rawVelocity_) < 100)
        {
            position_ = newPos;
            velocity_ = lowPass_.filter(rawVelocity_, dt);
        }
    }

    // Clamp target to maximum acceleration - but always allow deceleration.
    if (clampedTargetVelocity_ > 0.01)
        clampedTargetVelocity_ = std::clamp(targetVelocity, 0.0, std::max(velocity_, clampedTargetVelocity_) + maxAcceleration_ * dt);
    else if (clampedTargetVelocity_ < -0.01)
        clampedTargetVelocity_ = std::clamp(targetVelocity, std::min(velocity_, clampedTargetVelocity_) - maxAcceleration_ * dt, 0.0);
    else
        clampedTargetVelocity_ = std::clamp(targetVelocity, clampedTargetVelocity_ - maxAcceleration_ * dt, clampedTargetVelocity_ + maxAcceleration_ * dt);
    // Compute PI output, update integral only if not in saturation.
    double err = velocity_ - clampedTargetVelocity_;

    // Clamp error - to avoid discontinuous targets.
    // err = std::clamp(err, -1.0, 1.0);

    targetCurrent_ = - Kp_ * (err + Ki_ * integralValue_);
    if (targetCurrent_ + (-Kp_ * Ki_ * err * dt) < maxOutput_ && targetCurrent_ + (-Kp_ * Ki_ * err * dt) > -maxOutput_)
    {
        integralValue_ += err * dt;
        targetCurrent_ = - Kp_ * (err + Ki_ * integralValue_);
    }
    // Feedforward
    double feedforward = 10 * targetVelocity;
    if (feedforward > maxFeedforward_)
        feedforward = maxFeedforward_;
    else if (feedforward < -maxFeedforward_)
        feedforward = -maxFeedforward_;
    targetCurrent_ += feedforward;
    if (targetCurrent_ > maxOutput_)
        targetCurrent_ = maxOutput_;
    if (targetCurrent_ < -maxOutput_)
        targetCurrent_ = -maxOutput_;

    current_ = driver_->setCurrent(motorId_, targetCurrent_);
    isStopped_ = false;
    return velocity_;
}


void RMDXController::stop()
{
    driver_->stop(motorId_);
    isStopped_ = true;
}