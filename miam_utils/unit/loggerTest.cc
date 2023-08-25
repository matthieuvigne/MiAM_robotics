// Test logger class

#include "gtest/gtest.h"
#include <H5Cpp.h>

#include "miam_utils/Logger.h"

std::string const FILENAME = "test.hdf5";

TEST(Logger, LogData)
{
    Logger logger;
    logger.start(FILENAME);

    for (int i = 0; i < 100; i++)
    {
        logger.log("i", i, i);
        logger.log("iSquared", i, i * i);
    }
    logger.close();

    // Read and check file content
    H5::H5File file(FILENAME, H5F_ACC_RDONLY);

    H5::DataSet iData = file.openDataSet("i");
    H5::DataSet iSquaredData = file.openDataSet("iSquared");

    hsize_t dims_out[2];
    int ndims = iData.getSpace().getSimpleExtentDims(dims_out, NULL);
    ASSERT_EQ(ndims, 2);
    ASSERT_EQ(dims_out[0], 2);
    ASSERT_EQ(dims_out[1], 100);

    double dataOut[2][100];
    hsize_t dimsm[2] = {2, 100};
    H5::DataSpace memspace(2, dimsm);
    iData.read(dataOut, H5::PredType::NATIVE_DOUBLE, memspace, iData.getSpace());

    for (int i = 0; i < 100; i++)
    {
        ASSERT_FLOAT_EQ(dataOut[0][i], i);
        ASSERT_FLOAT_EQ(dataOut[1][i], i);
    }


    ndims = iSquaredData.getSpace().getSimpleExtentDims(dims_out, NULL);
    ASSERT_EQ(ndims, 2);
    ASSERT_EQ(dims_out[0], 2);
    ASSERT_EQ(dims_out[1], 100);

    iSquaredData.read(dataOut, H5::PredType::NATIVE_DOUBLE, memspace, iSquaredData.getSpace());
    for (int i = 0; i < 100; i++)
    {
        ASSERT_FLOAT_EQ(dataOut[0][i], i);
        ASSERT_FLOAT_EQ(dataOut[1][i], i*i);
    }

}
