#include "miam_utils/trajectory/ThreeWheelsKinematics.hpp"
#include <cmath>

namespace omni
{
    //======================================================================
    // Constructors
    //======================================================================
    ThreeWheelsKinematics::ThreeWheelsKinematics(double const& length, double const& wheelRadius): 
        length_(length),
        wheelRadius_(wheelRadius)
    {
    }

    //======================================================================
    // Public functions
    //======================================================================

    BaseSpeed ThreeWheelsKinematics::forwardKinematics(WheelSpeed const& wheelSpeed) const
    {
      BaseSpeed baseSpeed;

      baseSpeed.angular_velocity_ =  wheelRadius_ * (wheelSpeed.wheelSpeed_[0] +
        wheelSpeed.wheelSpeed_[1] + wheelSpeed.wheelSpeed_[2]) / length_;

      baseSpeed.linear_velocity_y_ = - (wheelRadius_ * wheelSpeed.wheelSpeed_[0] +
        baseSpeed.angular_velocity_ * this->length_);

      baseSpeed.linear_velocity_x_ = (wheelRadius_ * wheelSpeed.wheelSpeed_[2] - 
        wheelSpeed.wheelSpeed_[1]) / std::sqrt(3);

      return baseSpeed;
    }


    WheelSpeed ThreeWheelsKinematics::inverseKinematics(BaseSpeed const& baseSpeed) const
    {
        WheelSpeed wheelSpeed;

        wheelSpeed.wheelSpeed_[0] = - baseSpeed.linear_velocity_y_ -
            baseSpeed.angular_velocity_ * this->length_;

        wheelSpeed.wheelSpeed_[1] = 0.5 * (baseSpeed.linear_velocity_y_ -
            std::sqrt(3) * baseSpeed.linear_velocity_x_) -
            baseSpeed.angular_velocity_ * this->length_;

        wheelSpeed.wheelSpeed_[2] = 0.5 * (baseSpeed.linear_velocity_y_ +
            std::sqrt(3) * baseSpeed.linear_velocity_x_) -
            baseSpeed.angular_velocity_ * this->length_;

        for(int i = 0; i < 3; i++)
            wheelSpeed.wheelSpeed_[i] /= wheelRadius_;
        return wheelSpeed;
    }
}
