#include "miam_utils/trajectory/ThreeWheelsKinematics.hpp"
#include <cmath>

namespace omni
{
    ThreeWheelsKinematics::ThreeWheelsKinematics(double const& robotRadius, double const& wheelRadius): 
        robotRadius_(robotRadius),
        wheelRadius_(wheelRadius)
    {
    }


    BaseSpeed ThreeWheelsKinematics::forwardKinematics(WheelSpeed const& wheelSpeed) const
    {
        BaseSpeed baseSpeed;

        // Forward kinematics: reciprocal of inverse formulas.
        baseSpeed.omega_ = (wheelSpeed.w_[0] + wheelSpeed.w_[1] + wheelSpeed.w_[2]) / 3.0 * wheelRadius_ / robotRadius_;
        baseSpeed.vx_ = std::sqrt(3) / 4.0 * (wheelSpeed.w_[2] - wheelSpeed.w_[1]) * wheelRadius_;
        baseSpeed.vy_ =  2 * robotRadius_ * baseSpeed.omega_ - (wheelSpeed.w_[1] + wheelSpeed.w_[2]) * wheelRadius_;
        
        return baseSpeed;
    }


    WheelSpeed ThreeWheelsKinematics::inverseKinematics(BaseSpeed const& baseSpeed) const
    {
        WheelSpeed wheelSpeed;
        
        // Inverse dynamics: all motors run in inverse (clockwise) direction to provide more intuitive signs.
        wheelSpeed.w_[0] = robotRadius_ * baseSpeed.omega_ + baseSpeed.vy_;
        wheelSpeed.w_[1] = robotRadius_ * baseSpeed.omega_ - 0.5 * baseSpeed.vy_ - 2.0 / std::sqrt(3.0) * baseSpeed.vx_; 
        wheelSpeed.w_[2] = robotRadius_ * baseSpeed.omega_ - 0.5 * baseSpeed.vy_ + 2.0 / std::sqrt(3.0) * baseSpeed.vx_; 
        
        // Linear velocity to wheel angular velocity.
        for(int i = 0; i < 3; i++)
            wheelSpeed.w_[i] /= wheelRadius_;
        return wheelSpeed;
    }
}
