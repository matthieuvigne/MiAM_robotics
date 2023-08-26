/// \file Logger.h
/// \brief Implement the Logger class for logging robot telemetry data to a csv file.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef LOGGER
#define LOGGER
    #include <vector>
    #include <mutex>
    #include <thread>

    #include "Teleplot.h"

    #include "miam_utils/DatasetHandler.h"

    struct Datapoint{
        std::string name;
        double timestamp;
        double value;
    };

    /// \brief Telemetry class, for logging robot data to a csv file.
    /// \details The user specifies a list of headers and, at run time, a value for each element to log.
    ///          If no value is given, the previous value is kept.
    class Logger{
        public:
            /// \brief Create a logger.
            /// \details This function creates the log file, writes the header, and returns a logger struct, which can then
            ///          be used to add data to the log file.
            Logger();

            ~Logger();

            /// \brief Start logging thread.
            /// \param[in] filename Output filename
            /// \param[in] teleplotPrefix Optional prefix for variables in teleplot: used in simulation
            ///                           where several robots are logging.
            void start(std::string const& filename, std::string const& teleplotPrefix = "");

            /// @brief Set the origin of time, used on text log line.
            /// @param origin Time origin.
            void setTimeOrigin(timespec const& origin);

            /// @brief  Log a variable value
            /// @param name Variable name
            /// @param time Timestamp (s)
            /// @param value Value
            void log(std::string const& name, double const& time, double const& value);

            /// @brief Flush and close the file.
            ///
            /// @details This function is blocking and only returns once the write operation is complete.
            void close();

            /// @brief Log text data ; this data will also be printed in the terminal.
            template<typename T>
            Logger& operator<< (const T& data)
            {
                textData_ << data;
                return *this;
            }

            // Handling of std::endl, see https://stackoverflow.com/questions/1134388/stdendl-is-of-unknown-type-when-overloading-operator
            typedef std::basic_ostream<char, std::char_traits<char> > CoutType;
            typedef CoutType& (*StandardEndLine)(CoutType&);
            Logger& operator<<(StandardEndLine manip);

        private:
            void loggerThread(std::string const& filename);
            double getElapsedTime();

            Teleplot teleplot_;
            std::string teleplotPrefix_;
            std::vector<DatasetHandler> datasets_;
            std::vector<std::string> names_;
            std::vector<Datapoint> queuedDatapoints_;
            std::vector<std::string> queuedText_;

            std::mutex mutex_;
            std::thread thread_;
            bool askForTerminate_{false};

            std::stringstream textData_;

            timespec timeOrigin_;
    };
#endif
