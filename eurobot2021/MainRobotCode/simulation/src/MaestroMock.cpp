/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "MaestroMock.h"



MaestroMock::MaestroMock()
{
    state_ = std::vector<double>();
    for (int i = 0; i < 18; i++)
        state_.push_back(0);
}


bool MaestroMock::init(std::string const& portName, int const& deviceID)
{
    return true;
}


void MaestroMock::setPosition(int const& servo, double const& position)
{
    state_[servo] = position;
}


void MaestroMock::setSpeed(int const& servo, int const& speed)
{
}


std::vector<double> MaestroMock::getState()
{
    return state_;
}

void MaestroMock::setState(std::vector<double> const& vectorIn)
{
    state_ = vectorIn;
}