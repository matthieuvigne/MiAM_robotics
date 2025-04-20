#include "miam_utils/MathUtils.h"

std::vector<double> movingAverage(std::vector<double> const values, unsigned int const& windowSize)
{
    // Clamp window size for the computation to make sense.
    int winSize = std::min(windowSize, static_cast<unsigned int>((values.size() - 1) / 2));

    std::vector<double> outputValue;
    for (unsigned int i = 0; i < values.size(); i++)
    {
        unsigned int start = i - winSize;
        unsigned int end = i + winSize + 1;
        if (end > values.size())
        {
            int shift = end - values.size();
            end -= shift;
            start -= shift;
        }
        else if (start < 0)
        {
            end += -start;
            start = 0;
        }
        double value = 0;
        for (int j = start; j < end; j++)
            value += values.at(j);
        outputValue.push_back(value / (2 * winSize + 1));
    }
    return outputValue;
}