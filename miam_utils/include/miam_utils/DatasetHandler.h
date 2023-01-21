/// \file DatasetHandler.h
/// \brief Handler for HDF5 dataset for batch logging
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef DATASET_HANDLER
#define DATASET_HANDLER
    #include <H5Cpp.h>

    class DatasetHandler{
        static int const BUFFER_SIZE = 100;
        public:
            // Create an extensible dataset to contain the data.
            DatasetHandler(H5::H5File &file, std::string const& datasetName);

            /// @brief Add a datapoint to the queue, potentially flushing it.
            /// @param time Timestamp
            /// @param value Data value
            /// @return True if a flush was performed (buffer full)
            bool addPoint(double const& time, double const& value);

            /// @brief Flush content to HDF5 file
            void flush();
        private:
            H5::DataSet dataset_;
            unsigned int datasetOffset_ = 0;
            unsigned int pos_ = 0;
            double data_[2][BUFFER_SIZE];
    };

#endif
