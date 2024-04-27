/// \author MiAM Robotique
/// \copyright GNU GPLv3

#include <fstream>
#include <cstring>

#include "miam_utils/LogVariable.h"

int const INCREMENT = sizeof(double);
int const MARGIN = 500 - 3 * INCREMENT;

int const START = 1 + sizeof(uint16_t);

LogVariable::LogVariable()
{
}

void LogVariable::init(std::ofstream *file, uint16_t const& variableId)
{
    file_ = file;
    dataBuffer_[0] = 'd';
    memcpy(dataBuffer_ + 1, &variableId, sizeof(variableId));
    pos_ = START;
}

void LogVariable::addPoint(double const& time, double const& value)
{
    memcpy(dataBuffer_ + pos_, &time, INCREMENT);
    pos_ += INCREMENT;
    memcpy(dataBuffer_ + pos_, &value, INCREMENT);
    pos_ += INCREMENT;

    if (pos_ >= MARGIN)
        flush();
}

void LogVariable::maybe_flush()
{
    if (pos_ > BUFFER_SIZE / 2)
        flush();
}

void LogVariable::flush()
{
    // Nothing to do on empty data.
    if (file_ == nullptr || pos_ == START)
        return;
    file_->write(reinterpret_cast<char*>(&pos_), sizeof(uint16_t));
    file_->write(dataBuffer_, pos_);
    pos_ = START;
}
