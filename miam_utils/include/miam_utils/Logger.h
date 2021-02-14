/// \file Logger.h
/// \brief Implement the Logger class for logging robot telemetry data to a csv file.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef LOGGER
#define LOGGER
    #include <vector>
    #include <thread>
    #include <mutex>
    #include <condition_variable>

    /// \brief Telemetry class, for logging robot data to a csv file.
    /// \details The user specifies a list of headers and, at run time, a value for each element to log.
    ///          If no value is given, the previous value is kept.
    class Logger{
        public:
            /// \brief Create a logger.
            /// \details This function creates the log file, writes the header, and returns a logger struct, which can then
            ///          be used to add data to the log file.
            ///
            /// \param[in] filename Log filename.
            /// \param[in] logName Internal name of the log.
            /// \param[in] description Description string to add to the log first line.
            /// \param[in] headerList A comma-separated list of header. This line will be used directly for the header of the
            ///                       CSV file, and will also determine the number of elements.
            Logger(std::string const& filename,
                   std::string const& logName,
                   std::string const& description,
                   std::string const& headerList,
                   int const& minWriteSize = 1000);

            /// \brief Copy constructor
            Logger(Logger const & logger);

            /// \brief Start logger background thread
            void start();

            /// \brief Stop logger background thread, flushing content to file.
            void stop();

            /// \brief Set data value to log.
            ///
            /// \param[in] position Column number - this should be less than nElements, else this function has no effect.
            /// \param[in] data Data value to give.
            void setData(unsigned int const& position, double const& data);

            /// \brief Write last data sample (i.e. content of currentData) to the csv file.
            void writeLine();
        private:
            void run();

            std::string filename_; ///< Logfile
            bool isRunning_; ///< Boolean to start / stop background thread.

            std::mutex mutex_;  ///< Mutex and contionnal variable: logging is done in the background.
            std::condition_variable cond_;
            std::vector<std::vector<double>> bufferedData_; ///< Data to flush to the logfile.
            std::vector<double> currentData_; ///< Current data to log.
    };

#endif
