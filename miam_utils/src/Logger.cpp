/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/Logger.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <thread>


Logger::Logger():
    teleplot_(Teleplot::localhost()),
    datasets_(),
    names_(),
    queuedDatapoints_(),
    mutex_()
{
}

void Logger::start(std::string const& filename)
{
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
        int nNames = names_.size();
        mutex_.unlock();
        // Create new datasets as needed
        for (unsigned int i = datasets_.size(); i < nNames; i++)
            datasets_.push_back(DatasetHandler(file, names_.at(i)));
        // Process data points
        bool hasFlush = false;
        for (auto p : newDataPoints)
        {
            hasFlush |= datasets_.at(p.datasetId).addPoint(p.timestamp, p.value);
            teleplot_.update(names_[p.datasetId], p.value);
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
    // Lookup name - if it doesn't exist, add a new entry.
    int id = 0;
    auto itr = std::find(names_.begin(), names_.end(), name);

    mutex_.lock();
    if (itr == names_.end())
    {
        id = names_.size();
        names_.push_back(name);
    }
    else
        id = itr - names_.begin();
    queuedDatapoints_.push_back(Datapoint{id, time, value});
    mutex_.unlock();
}

void Logger::flush()
{

}

