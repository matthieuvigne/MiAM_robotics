/// \file Logger.h
/// \brief Implement the Logger class for logging robot telemetry data to a csv file.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef LOGGER
#define LOGGER
    #include <vector>
    #include <mutex>
    #include <thread>
    #include <sstream>

    #include "miam_utils/LogVariable.h"

    struct Datapoint{
        uint16_t idx;
        double timestamp;
        double value;
    };
    namespace logger{
        int const MAX_VARIABLES = 200;
    }

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
            /// \param[in] teleplotPrefix Optional prefix for variables in teleplot / stdout: used in simulation
            ///                           where several robots are logging.
            /// \param[in] silent If set, logger does not output to teleplot nor std::cout
            void start(std::string const& filename, std::string const& teleplotPrefix = "", bool const& silent = false);

            /// @brief Set the origin of time, used on text log line.
            /// @param origin Time origin.
            void setTimeOrigin(timespec const& origin);

            /// @brief Register a variable, obtaining an id that can be used for futur logging.
            /// @details Note that this function may be called several times safely, the same
            ///          id is returned.
            /// @param[in] name Variable name.
            /// @return Variable id, to be used when calling the log method.
            uint16_t registerVariable(std::string const& name)
            {
                return getVariableId(name);
            }

            /// @brief Log a variable value base on its id
            /// @param idx Variable id, as returned by registerVariable
            /// @param time Timestamp (s)
            /// @param value Value
            void log(uint16_t const& idx, double const& time, double const& value);

            /// @brief  Log a variable value
            /// @details This method performs a name lookup, and is thus slower than the id-based version.
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

            uint16_t getVariableId(std::string const& varName);

            std::string teleplotPrefix_;
            std::string cliPrefix_;

            LogVariable variables_[logger::MAX_VARIABLES];
            int nRegisteredVariables_ = 0;

            std::vector<std::string> names_;
            std::vector<Datapoint> queuedDatapoints_;
            std::vector<std::string> queuedText_;

            std::mutex mutex_;
            std::thread thread_;
            bool askForTerminate_{false};

            std::stringstream textData_;

            timespec timeOrigin_;

            bool silent_ = false;
            int nLines_ = 0;
    };
#endif
