/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/Logger.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <thread>


Logger::Logger():
    teleplot_(Teleplot::localhost()),
    teleplotPrefix_(),
    datasets_(),
    names_(),
    queuedDatapoints_(),
    mutex_()
{
}

void Logger::start(std::string const& filename, std::string const& teleplotPrefix)
{
    teleplotPrefix_ = teleplotPrefix;
    std::thread th = std::thread(&Logger::loggerThread, this, filename);
    th.detach();
}

void Logger::loggerThread(std::string const& filename)
{
    H5::H5File file(filename, H5F_ACC_TRUNC);

    while (true)
    {
        std::vector<Datapoint> newDataPoints;

        mutex_.lock();
        newDataPoints = queuedDatapoints_;
        queuedDatapoints_.clear();
        mutex_.unlock();

        // Process data points
        bool hasFlush = false;
        for (auto p : newDataPoints)
        {
            // Lookup name - if it doesn't exist, add a new entry.
            int id = 0;
            auto itr = std::find(names_.begin(), names_.end(), p.name);
            if (itr == names_.end())
            {
                id = names_.size();
                names_.push_back(p.name);
                datasets_.push_back(DatasetHandler(file, p.name));
            }
            else
                id = itr - names_.begin();

            hasFlush |= datasets_.at(id).addPoint(p.timestamp, p.value);
            teleplot_.update(teleplotPrefix_ + p.name, p.value);
        }
        // Flush everything if at least one buffer was full
        if (hasFlush)
        {
            for (auto d : datasets_)
                d.flush();
        }
        usleep(1000);
    }
}



void Logger::log(std::string const& name, double const& time, double const& value)
{
    mutex_.lock();
    queuedDatapoints_.push_back(Datapoint{name, time, value});
    mutex_.unlock();
}

void Logger::flush()
{

}

