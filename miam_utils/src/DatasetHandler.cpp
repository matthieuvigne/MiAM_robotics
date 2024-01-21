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

void DatasetHandler::addPoint(double const& time, double const& value)
{
    data_[0][pos_] = time;
    data_[1][pos_] = value;
    pos_++;
    if (pos_ == BUFFER_SIZE)
        flush();
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
    double dataToWrite[2][pos_];
    for (int i = 0; i < pos_; i++)
    {
        dataToWrite[0][i] = data_[0][i];
        dataToWrite[1][i] = data_[1][i];
    }
    dataset_.write(dataToWrite, PredType::NATIVE_DOUBLE, mspace2, fspace);

    datasetOffset_ += pos_;
    pos_ = 0;
    dataset_.flush(H5F_SCOPE_LOCAL);
}

H5::StrType STRING_TYPE(H5::PredType::C_S1, H5T_VARIABLE);

TextLogHandler::TextLogHandler(H5File &file)
{
    // Create extensible dataset.
    hsize_t   dims[1]    = {0}; // dataset dimensions at creation
    hsize_t   maxdims[1] = {H5S_UNLIMITED};
    DataSpace mspace1(1, dims, maxdims);

    DSetCreatPropList cparms;
    hsize_t chunk_dims[1] = {BUFFER_SIZE};
    cparms.setChunk(1, chunk_dims);

    H5::Group group = file.createGroup("textLog");
    dataset_ = group.createDataSet("log", STRING_TYPE, mspace1, cparms);
}

void TextLogHandler::append(std::string const& message)
{
    data_[pos_] = message;
    pos_++;
    if (pos_ == BUFFER_SIZE)
        flush();
}

void TextLogHandler::flush()
{
    // Nothing to do on empty data.
    if (pos_ == 0)
        return;
    hsize_t size[1] = {datasetOffset_ + pos_};
    dataset_.extend(size);

    DataSpace fspace = dataset_.getSpace();
    hsize_t offset[1] = {datasetOffset_};
    hsize_t dims[1] = {1};
    DataSpace mspace2(1, dims);
    for (unsigned int i = 0; i < pos_; i++)
    {
        fspace.selectHyperslab(H5S_SELECT_SET, dims, offset);
        dataset_.write(&data_[i], STRING_TYPE, mspace2, fspace);
        offset[0] += 1;
    }

    datasetOffset_ += pos_;
    pos_ = 0;

    dataset_.flush(H5F_SCOPE_GLOBAL);
}
