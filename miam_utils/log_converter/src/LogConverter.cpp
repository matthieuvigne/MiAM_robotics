
#include <iostream>
#include <fstream>
#include <vector>

#include <H5Cpp.h>


struct Variable{
    std::string name;
    std::vector<double> time;
    std::vector<double> value;
};


int main (int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cout << "Missing argument: input file name" << std::endl;
        return -1;
    }
    std::string const inputFilename = argv[1];
    std::string const outputFilename = inputFilename.substr(
        0,
        inputFilename.find_last_of('.')) + ".hdf5";

    // Read data.
    std::ifstream source(inputFilename, std::ifstream::binary);
    if (source.fail())
    {
        std::cout << "Failed to open file " << inputFilename << ". Does the file exist?" << std::endl;
        return -1;
    }
    char header[5];
    char supportedHeader[5] = {'m', 'i', 'a', 'm', 1};
    source.read(header, 5);
    bool isValid = source.good();
    for (int i = 0; i < 4; i++)
        isValid &= header[i] == supportedHeader[i];
    if (!isValid)
    {
        std::cout << "Invalid file provided, 'miam' header not detected." << std::endl;
        return -1;
    }
    if (header[4] != supportedHeader[4])
    {
        std::cout << "Unsupported file version: " << static_cast<int>(header[4]) << std::endl;
        return -1;
    }

    std::vector<Variable> variables_;
    std::vector<std::string> messages_;

    char chunkLength[2];
    while (source.good())
    {
        source.read(chunkLength, 2);
        if (!source.good())
            break;
        uint16_t const size = *reinterpret_cast<uint16_t*>(chunkLength);

        char chunk[size];
        source.read(chunk, size);

        if (chunk[0] == 'n' || chunk[0] == 'd')
        {
            uint16_t const idx = *reinterpret_cast<uint16_t*>(chunk + 1);
            while (variables_.size() < idx + 1)
            {
                variables_.push_back(Variable());
            }
            if (chunk[0] == 'n')
            {
                variables_[idx].name = std::string(chunk + 3, size - 3);
            }
            else if (chunk[0] == 'd')
            {
                uint16_t pos = 3;
                while (pos < size)
                {
                    variables_[idx].time.push_back(*reinterpret_cast<double*>(chunk + pos));
                    pos += sizeof(double);
                    variables_[idx].value.push_back(*reinterpret_cast<double*>(chunk + pos));
                    pos += sizeof(double);
                }
            }
        }
        else if (chunk[0] == 's')
        {
            messages_.push_back(std::string(chunk + 1, size - 1));
        }
        else
        {
            std::cout << "Warning: unknown chunk type: '" << chunk[0] << "'. Skipping." << std::endl;
        }
    }

    // Write output to HDF5

    // Write variables
    H5::H5File file(outputFilename, H5F_ACC_TRUNC);
    for (auto const& v : variables_)
    {
        hsize_t   dims[2]    = {2, v.time.size()};
        H5::DataSpace mspace(2, dims);
        H5::DataSet dataset = file.createDataSet(v.name, H5::PredType::NATIVE_DOUBLE, mspace);

        double dataBuffer[2][v.time.size()];
        for (unsigned int i = 0; i < v.time.size(); i++)
        {
            dataBuffer[0][i] = v.time.at(i);
            dataBuffer[1][i] = v.value.at(i);
        }
        dataset.write(dataBuffer, H5::PredType::NATIVE_DOUBLE, mspace);
    }

    // Write strings
    H5::StrType STRING_TYPE(H5::PredType::C_S1, H5T_VARIABLE);
    hsize_t   dims[1]    = {messages_.size()};
    H5::DataSpace mspace(1, dims);
    H5::Group group = file.createGroup("textLog");
    H5::DataSet dataset = group.createDataSet("log", STRING_TYPE, mspace);
    H5::DataSpace fspace = dataset.getSpace();

    hsize_t offset[1] = {0};
    dims[0] = 1;
    mspace = H5::DataSpace(1, dims);
    for (auto const& message : messages_)
    {
        fspace.selectHyperslab(H5S_SELECT_SET, dims, offset);
        dataset.write(&message, STRING_TYPE, mspace, fspace);
        offset[0]++;
    }

    file.close();
    std::cout << "Output written to " << outputFilename << std::endl;
    return 0;
}


