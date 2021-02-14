/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/PID.h"

namespace miam{
    PID::PID():
        Kp_(0.0),
        Kd_(0.0),
        Ki_(0.0),
        maxIntegral_(0.0),
        integral_(0.0),
        previousError_(0.0),
        lastCorrection_(0.0)
    {
    }


    PID::PID(double const& Kp, double const& Kd, double const& Ki, double const& maxIntegral):
        Kp_(Kp),
        Kd_(Kd),
        Ki_(Ki),
        maxIntegral_(maxIntegral),
        integral_(0.0),
        previousError_(0.0),
        lastCorrection_(0.0)
    {

    }


    double PID::computeValue(double const& error, double const& dt)
    {
        // Compute derivative.
        double derivative = 0.0;
        if(dt > 1e-6)
            derivative = (error - previousError_) / dt;

        return computeValue(error, derivative, dt);
    }


    double PID::computeValue(double const& error, double const& errorDerivative, double const& dt)
    {
        previousError_ = error;

        // Compute integral, clamped.
        integral_ = integral_ + dt * error;
        if(Ki_ > 1e-6)
        {
            if(Ki_ * integral_ > maxIntegral_)
                integral_ = maxIntegral_ / Ki_;
            if(Ki_ * integral_ < -maxIntegral_)
                integral_ = -maxIntegral_ / Ki_;
        }
        // Return result - minus because error is defined as current - target.
        lastCorrection_ = -Kp_ * (error + Kd_ * errorDerivative + Ki_ * integral_);
        return lastCorrection_;
    }


    void PID::resetIntegral(double const& value)
    {
        integral_ = value;
    }


    double PID::getCorrection()
    {
        return lastCorrection_;
    }


    double PID::getIntegral()
    {
        return integral_;
    }
}
