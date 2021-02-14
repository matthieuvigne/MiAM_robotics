/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/Logger.h"
#include <fstream>
#include <string>
#include <algorithm>

Logger::Logger(std::string const& filename,
               std::string const& logName,
               std::string const& description,
               std::string const& headerList,
               int const& minWriteSize):
    filename_(filename),
    isRunning_(false),
    bufferedData_(),
    currentData_()
{
    std::ofstream logfile;
    // Create CSV file.
    logfile.open(filename);

    if (!logfile.is_open())
    {
        #ifdef DEBUG
            std::cout << "Logger error when creating log file: " << filename << std::endl;
        #endif
        return;
    }
    // Write header.
    logfile << "Robot Log: " << logName << "," << description << std::endl;
    logfile << headerList << std::endl;

    // Determine number of elements from number of commas in header list.
    int nElement = std::count(headerList.begin(), headerList.end(), ',') + 1;
    currentData_ = std::vector<double>(nElement, 0.0);
}

Logger::Logger(Logger const& logger):
    filename_(logger.filename_),
    isRunning_(false),
    bufferedData_(logger.bufferedData_),
    currentData_(logger.currentData_)
{

}


void Logger::setData(unsigned int const& position, double const& data)
{
    if (position < currentData_.size())
        currentData_[position] = data;
}

void Logger::start()
{
    isRunning_ = true;
    // Start worker thread
    std::thread t(&Logger::run, this);
    t.detach();
}

void Logger::stop()
{
    isRunning_ = false;
    cond_.notify_one();
}

void Logger::run()
{
    std::ofstream logfile;
    logfile.open(filename_, std::ios_base::app);
    while(isRunning_)
    {
        std::vector<std::vector<double>> dataToLog;
        // Wait for data to be available.
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock);
        for (auto data : bufferedData_)
            dataToLog.push_back(data);
        bufferedData_.clear();
        lock.unlock();
        // Actually write to file.
        for (auto data : dataToLog)
        {
            std::string line;
            for(auto d : data)
                line +=  std::to_string(d) + ",";
            line +=  std::to_string(currentData_.back()) + "\n";
            logfile << line;
        }
        logfile.flush();
    }
}

void Logger::writeLine()
{
    // Ask worker thread to write data to file.
    mutex_.lock();
    bufferedData_.push_back(currentData_);
    mutex_.unlock();
    cond_.notify_one();
}
