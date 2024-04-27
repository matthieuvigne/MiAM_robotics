// `.miam` format specification:
// A `.miam` is a binary file containing both data (numbers) and messages (string).
// The file starts by the word 'miam', in ASCII, followed by one byte indicating
// version number (currently 1).
// The file is then composed of a serie of chunk ; each chunk starts with a 2-bytes int16
// indicating its size (no counting these two bytes), followed by a single byte indicating its type.
// The content then varies depending on the type.
//
// - Type 'n' (0x6E): name
//      Gives the name of a dataset. The next two bytes are the dataset ID. The rest is the name (ASCII string)
// - Type 'd' (0x64): data
//      Data timeserie. The first two bytes are the dataset ID. The rest is a serie of 2 double (time, value)
// - Type 's' (0x73): string
//      String corresponding to text messages (i.e. terminal output). The content is simply the string raw data.

#include "miam_utils/Logger.h"

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <iomanip>
#include <algorithm>
#include <fstream>

double const FLUSH_INTERVAL = 1.0;

#ifdef ENABLE_TELEPLOT
#include "Teleplot.h"
#endif

Logger::Logger():
    teleplotPrefix_(),
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
    cliPrefix_ = "";
    if (teleplotPrefix_.length() > 0)
    {
        cliPrefix_ = "[" + teleplotPrefix_ + "] ";
    }
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
    pthread_setname_np(pthread_self(), "miam_logger");
    std::ofstream file(filename, std::ios::out | std::ios::binary);
    char header[5] = {'m', 'i', 'a', 'm', 1};
    file.write(header, 5);
    file.flush();

    for (unsigned int i = 0; i < logger::MAX_VARIABLES; i++)
        variables_[i].init(&file, i);
    nRegisteredVariables_ = 0;

    double lastFlush = 0.0;

    while (!askForTerminate_ || queuedDatapoints_.size() > 0 || queuedText_.size() > 0)
    {
        std::vector<Datapoint> newDataPoints;
        std::vector<std::string> newStrings;
        std::vector<std::string> newNames;

        mutex_.lock();
        newDataPoints.swap(queuedDatapoints_);
        newStrings.swap(queuedText_);
        newNames.insert(newNames.begin(), names_.begin() + nRegisteredVariables_, names_.end());
        mutex_.unlock();

        // Write new variable names to file
        for (auto const& names : newNames)
        {
            uint16_t const size = 3 + names.size();
            char chunk[2 + size];
            memcpy(chunk, &size, sizeof(uint16_t));
            chunk[2] = 'n';
            memcpy(chunk + 3, &nRegisteredVariables_, sizeof(uint16_t));
            memcpy(chunk + 5, names.data(), names.size());
            file.write(chunk, size + 2);
            nRegisteredVariables_ ++;
        }

        // Process data points
        for (auto const& p : newDataPoints)
        {
            variables_[p.idx].addPoint(p.timestamp, p.value);
#ifdef ENABLE_TELEPLOT
            Teleplot::localhost().update(teleplotPrefix_ + names_.at(p.idx), p.value);
#endif
        }

        // Process strings
        for (auto const& s : newStrings)
        {
            uint16_t const size = 1 + s.size();

            char chunk[2 + size];
            memcpy(chunk, &size, sizeof(uint16_t));
            chunk[2] = 's';
            memcpy(chunk + 3, s.data(), s.size());
            file.write(chunk, size + 2);
        }

        // Flush everything at least at the specified time interval.
        if (getElapsedTime() - lastFlush > FLUSH_INTERVAL)
        {
            for (uint16_t i = 0; i < nRegisteredVariables_; i++)
                variables_[i].maybe_flush();
            file.flush();
            lastFlush = getElapsedTime();
        }
        usleep(1000);
    }
    for (uint16_t i = 0; i < nRegisteredVariables_; i++)
        variables_[i].flush();
    names_.clear();
    queuedDatapoints_.clear();
    queuedText_.clear();
    file.close();
}

uint16_t Logger::getVariableId(std::string const& varName)
{
    auto itr = std::find(names_.begin(), names_.end(), varName);
    if (itr == names_.end())
    {
        names_.push_back(varName);
        return names_.size() - 1;
    }
    else
     return itr - names_.begin();
}


void Logger::log(uint16_t const& idx, double const& time, double const& value)
{
    mutex_.lock();
    queuedDatapoints_.push_back(Datapoint{idx, time, value});
    mutex_.unlock();
}


void Logger::log(std::string const& name, double const& time, double const& value)
{
    log(getVariableId(name), time, value);
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
    std::cout << cliPrefix_ << time.str() << std::endl;
    textData_.str("");
    mutex_.lock();
    queuedText_.push_back(time.str());
    mutex_.unlock();
    return *this;
}