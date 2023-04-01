/// \author MiAM Robotique
/// \copyright GNU GPLv3

#include "miam_utils/DatasetHandler.h"
using namespace H5;

#include <iostream>

DatasetHandler::DatasetHandler(H5File &file, std::string const& datasetName)
{
    // Create extensible dataset.
    hsize_t   dims[2]    = {2, 0}; // dataset dimensions at creation
    hsize_t   maxdims[2] = {2, H5S_UNLIMITED};
    DataSpace mspace1(2, dims, maxdims);

    DSetCreatPropList cparms;
    hsize_t chunk_dims[2] = {2, BUFFER_SIZE};
    cparms.setChunk(2, chunk_dims);

    dataset_ = file.createDataSet(datasetName, PredType::NATIVE_DOUBLE, mspace1, cparms);

    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        data_[0][i] = 0;
        data_[1][i] = 0;
    }
}

bool DatasetHandler::addPoint(double const& time, double const& value)
{
    data_[0][pos_] = time;
    data_[1][pos_] = value;
    pos_++;
    if (pos_ == BUFFER_SIZE)
    {
        flush();
        return true;
    }
    return false;
}

void DatasetHandler::flush()
{
    // Nothing to do on empty data.
    if (pos_ == 0)
        return;
    hsize_t size[2] = {2, datasetOffset_ + pos_};
    dataset_.extend(size);

    DataSpace fspace = dataset_.getSpace();
    hsize_t offset[2] = {0, datasetOffset_};
    hsize_t dims[2] = {2, pos_};
    fspace.selectHyperslab(H5S_SELECT_SET, dims, offset);

    DataSpace mspace2(2, dims);
    dataset_.write(data_, PredType::NATIVE_DOUBLE, mspace2, fspace);
    dataset_.flush(H5F_SCOPE_GLOBAL);

    datasetOffset_ += pos_;
    pos_ = 0;
}
