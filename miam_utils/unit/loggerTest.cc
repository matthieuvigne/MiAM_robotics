// Test logger class

#include "gtest/gtest.h"
#include <H5Cpp.h>

#include "miam_utils/Logger.h"

#include <libgen.h>         // dirname
#include <unistd.h>         // readlink
#include <linux/limits.h>   // PATH_MAX


std::string const FILENAME = "test.miam";
std::string const FILENAME_HDF5 = "test.hdf5";

// Helper: get path of converter utility
std::string getConverterPath()
{
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    const char *path;
    if (count != -1) {
        path = dirname(result);
    }
    std::string fullPath(result);
    fullPath += "/../log_converter/miam_log_converter";
    return fullPath;
}

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
    std::string cmd =  getConverterPath() + " " + FILENAME;
    ASSERT_EQ(system(cmd.c_str()), 0);

    // Read and check file content
    H5::H5File file(FILENAME_HDF5, H5F_ACC_RDONLY);

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


TEST(Logger, TextLog)
{
    Logger logger;
    logger.start(FILENAME);
    logger << "Hello world !" << std::endl;
    logger << "Data: " << 42 << " " << 10.25 << std::endl;
    for (int i = 0; i < 20; i++)
    {
        usleep(1000);
        logger << "Data: " << i << std::endl;
    }
    logger.close();
    std::string cmd =  getConverterPath() + " " + FILENAME;
    ASSERT_EQ(system(cmd.c_str()), 0);

    // Read and check file content
    H5::H5File file(FILENAME_HDF5, H5F_ACC_RDONLY);

    H5::DataSet textData = file.openGroup("textLog").openDataSet("log");

    hsize_t dims_out[1];
    int ndims = textData.getSpace().getSimpleExtentDims(dims_out, NULL);
    ASSERT_EQ(ndims, 1);
    ASSERT_EQ(dims_out[0], 22);


    std::string dataOut[22];
    hsize_t dimsm[1] = {1};
    H5::DataSpace memspace(1, dimsm);
    H5::DataSpace fspace = textData.getSpace();
    hsize_t offset[1] = {0};
    for (int i = 0; i < 22; i++)
    {
        fspace.selectHyperslab(H5S_SELECT_SET, dimsm, offset);
        std::string str;
        textData.read(dataOut[i], textData.getStrType(), memspace, fspace);
        offset[0] += 1;
    }

    // Check content: all strings are formated [0.0TIMESTAMP] Data
    // Check that timestamp are increasing
    std::string expectedStrings[22] = {
        "Hello world !",
        "Data: 42 10.25",
        "Data: 0",
        "Data: 1",
        "Data: 2",
        "Data: 3",
        "Data: 4",
        "Data: 5",
        "Data: 6",
        "Data: 7",
        "Data: 8",
        "Data: 9",
        "Data: 10",
        "Data: 11",
        "Data: 12",
        "Data: 13",
        "Data: 14",
        "Data: 15",
        "Data: 16",
        "Data: 17",
        "Data: 18",
        "Data: 19",
    };

    double lastTime = 0;
    for (int i = 0; i < 22; i++)
    {
        ASSERT_EQ(dataOut[i].substr(0, 4), "[0.0");
        double time = std::stof(dataOut[i].substr(1, 8));
        ASSERT_GT(time, lastTime);
        lastTime = time;
        ASSERT_EQ(dataOut[i].substr(9), "] " + expectedStrings[i]);
    }
    ASSERT_GT(lastTime, 0.020);
}
