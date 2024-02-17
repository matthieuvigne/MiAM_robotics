#ifndef MIAM_MATH_UTILS
#define MIAM_MATH_UTILS

    #include <vector>

    /// @brief Compute centered moving average
    /// @param values Values to filter
    /// @param halfWindowSize Size of the moving average
    /// @return Filtered value
    std::vector<double> movingAverage(std::vector<double> const values, unsigned int const& halfWindowSize);
#endif
