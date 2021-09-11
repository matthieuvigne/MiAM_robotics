/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/Logger.h"
#include <iostream>
#include <string>
#include <algorithm>

Logger::Logger(std::string const& filename,
               std::string const& logName,
               std::string const& description,
               std::string const& headerList)
{
    // Create CSV file.
    logFile.open(filename);
    if (!logFile.is_open())
    {
        #ifdef DEBUG
            std::cout << "Logger error when creating log file: " << filename << std::endl;
        #endif
        return;
    }
    // Write header.
    logFile << "Robot Log: " << logName << "," << description << std::endl;
    logFile << headerList << std::endl;

    // Determine number of elements from number of commas in header list.
    int nElement = std::count(headerList.begin(), headerList.end(), ',') + 1;
    currentData_ = std::vector<double>(nElement, 0.0);
}

Logger::Logger():
    currentData_(std::vector<double>(0, 0.0))
{
}

void Logger::setData(unsigned int const& position, double const& data)
{
    if (position < currentData_.size())
        currentData_[position] = data;
}

void Logger::writeLine()
{
    // If file is not open, do nothing.
    if (!logFile.is_open())
        return;

    // Print data line.
    for(unsigned int i = 0; i < currentData_.size() - 1; i++)
        logFile << currentData_[i] << ",";
    // Last element: don't add trailing comma.
    logFile << currentData_.back() << std::endl;
}
