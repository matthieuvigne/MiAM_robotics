#include "miam_utils/LowPass.h"
#include <math.h>
#include <iostream>

namespace miam
{
LowPass::LowPass(double const& frequency):
    omega_(2 * M_PI * frequency),
    state_(0),
    isInit_(false)
{
}

void LowPass::reset(double const& value)
{
    state_ = value;
}

double LowPass::filter(double const& value, double const& dt)
{
    if (!isInit_)
    {
        state_ = value;
        isInit_ = true;
        std::cout << isInit_ << std::endl;
    }
    // Low-pass equation: y + dy / omega = x
    double const& b = std::exp(-dt * omega_);
    state_ = b * state_ + (1 - b) * value;
    return state_;
}

}