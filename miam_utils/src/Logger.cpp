/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/Logger.h"
#include <iostream>
#include <string>
#include <algorithm>

double const FLUSH_INTERVAL = 1.0;

Logger::Logger():
    teleplot_(Teleplot::localhost()),
    teleplotPrefix_(),
    datasets_(),
    names_(),
    queuedDatapoints_(),
    mutex_()
{
}

Logger::~Logger()
{
    askForTerminate_ = true;
    // Wait for thread to finish
    if (thread_.joinable())
        thread_.join();
}

void Logger::start(std::string const& filename, std::string const& teleplotPrefix)
{
    teleplotPrefix_ = teleplotPrefix;
    close(); // Close possibly existing log.
    clock_gettime(CLOCK_MONOTONIC, &timeOrigin_);
    thread_ = std::thread(&Logger::loggerThread, this, filename);
}

void Logger::setTimeOrigin(timespec const& origin)
{
    timeOrigin_ = origin;
}

double Logger::getElapsedTime()
{
    struct timespec currentTime;
    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    return currentTime.tv_sec - timeOrigin_.tv_sec + (currentTime.tv_nsec - timeOrigin_.tv_nsec) / 1e9;
}

void Logger::loggerThread(std::string const& filename)
{
    H5::H5File file(filename, H5F_ACC_TRUNC);
    TextLogHandler textHandler(file);
    double lastFlush = 0.0;

    while (!askForTerminate_ || queuedDatapoints_.size() > 0 || queuedText_.size() > 0)
    {
        std::vector<Datapoint> newDataPoints;
        std::vector<std::string> newStrings;

        mutex_.lock();
        newDataPoints = queuedDatapoints_;
        queuedDatapoints_.clear();
        newStrings = queuedText_;
        queuedText_.clear();
        mutex_.unlock();

        // Process data points
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

            datasets_.at(id).addPoint(p.timestamp, p.value);
            teleplot_.update(teleplotPrefix_ + p.name, p.value);
        }
        for (auto s : newStrings)
            textHandler.append(s);

        // Flush everything at least at the specified time interval.
        if (getElapsedTime() - lastFlush > FLUSH_INTERVAL)
        {
            for (auto d : datasets_)
                d.flush();
            textHandler.flush();
            lastFlush = getElapsedTime();
        }
        usleep(1000);
    }
    for (auto d : datasets_)
        d.flush();
    textHandler.flush();
    file.close();
}


void Logger::log(std::string const& name, double const& time, double const& value)
{
    mutex_.lock();
    queuedDatapoints_.push_back(Datapoint{name, time, value});
    mutex_.unlock();
}

void Logger::close()
{
    askForTerminate_ = true;
    if (thread_.joinable())
        thread_.join();
    askForTerminate_ = false;
}

Logger& Logger::operator<<(StandardEndLine manip)
{
    std::stringstream time;
    time << "[" << std::fixed << std::setprecision(6) << getElapsedTime() << "] " << textData_.str();
    std::cout << time.str() << std::endl;
    textData_.str("");
    mutex_.lock();
    queuedText_.push_back(time.str());
    mutex_.unlock();
    return *this;
}