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

      baseSpeed.omega_ =  wheelRadius_ * (wheelSpeed.wheelSpeed_[0] +
        wheelSpeed.wheelSpeed_[1] + wheelSpeed.wheelSpeed_[2]) / robotRadius_;

      baseSpeed.vy_ = - (wheelRadius_ * wheelSpeed.wheelSpeed_[0] +
        baseSpeed.omega_ * robotRadius_);

      baseSpeed.vx_ = (wheelRadius_ * wheelSpeed.wheelSpeed_[2] - 
        wheelSpeed.wheelSpeed_[1]) / std::sqrt(3);

      return baseSpeed;
    }


    WheelSpeed ThreeWheelsKinematics::inverseKinematics(BaseSpeed const& baseSpeed) const
    {
        WheelSpeed wheelSpeed;

        wheelSpeed.wheelSpeed_[0] = - baseSpeed.vy_ -
            baseSpeed.omega_ * robotRadius_;

        wheelSpeed.wheelSpeed_[1] = 0.5 * (baseSpeed.vy_ -
            std::sqrt(3) * baseSpeed.vx_) -
            baseSpeed.omega_ * robotRadius_;

        wheelSpeed.wheelSpeed_[2] = 0.5 * (baseSpeed.vy_ +
            std::sqrt(3) * baseSpeed.vx_) -
            baseSpeed.omega_ * robotRadius_;
        
        for(int i = 0; i < 3; i++)
            wheelSpeed.wheelSpeed_[i] /= wheelRadius_;
        return wheelSpeed;
    }
}
