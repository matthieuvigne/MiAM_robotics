#ifndef LOG_VARIABLE
#define LOG_VARIABLE

    #include <fstream>

    // Handle logging of a single variable.
    class LogVariable{
        static int const BUFFER_SIZE = 500;
        public:
            // Create an extensible dataset to contain the data.
            LogVariable();

            void init(std::ofstream *file, uint16_t const& variableId);

            /// @brief Add a datapoint to the queue, potentially flushing it.
            /// @param time Timestamp
            /// @param value Data value
            void addPoint(double const& time, double const& value);

            /// @brief Gently ask for flush - flush only happens if 'enough' data is present.
            void maybe_flush();

            /// @brief Flush content to file
            void flush();
        private:
            std::ofstream *file_ = nullptr;
            char dataBuffer_[BUFFER_SIZE];
            uint16_t pos_ = 0;
    };

    // Specific handler for text log, a 1xN string array.
    // class TextLogHandler{
    //     static int const BUFFER_SIZE = 10;
    //     public:
    //         TextLogHandler(H5::H5File &file);

    //         /// @brief Add string to HDF5 file
    //         /// @param message String to add.
    //         void append(std::string const& message);

    //         /// @brief Flush content to HDF5 file
    //         void flush();

    //     private:
    //         H5::DataSet dataset_;
    //         unsigned int datasetOffset_ = 0;
    //         unsigned int pos_ = 0;
    //         std::string data_[BUFFER_SIZE];
    // };

#endif
