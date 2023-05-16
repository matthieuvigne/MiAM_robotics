/// \file TextLogger.h
/// \brief Log data to console, adding time reference in front of it.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef TEXT_LOGGER
#define TEXT_LOGGER
    #include <time.h>
    #include <iostream>
    #include <iomanip>
    namespace textlogger
    {
        extern timespec startTime;

        inline void setStartTime()
        {
            clock_gettime(CLOCK_MONOTONIC, &startTime);
        }

        inline double getTime()
        {
            struct timespec currentTime;
            clock_gettime(CLOCK_MONOTONIC, &currentTime);
            return currentTime.tv_sec - startTime.tv_sec + (currentTime.tv_nsec - startTime.tv_nsec) / 1e9;
        }

        inline std::ostream* log()
        {
            std::cout << "[" << std::fixed << std::setprecision(6) << getTime() << "] ";
            return &std::cout;
        }

        inline std::ostream* error()
        {
            std::cerr << "[" << std::fixed << std::setprecision(6) << getTime() << "] ";
            return &std::cerr;
        }

        #define textlog *textlogger::log()


        #define texterror *textlogger::error()
    }
#endif
