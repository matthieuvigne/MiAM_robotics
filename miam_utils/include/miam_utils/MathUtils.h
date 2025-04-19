#ifndef MIAM_MATH_UTILS
#define MIAM_MATH_UTILS

    #include <vector>
    #include <cmath>

    /// @brief Compute centered moving average
    /// @param values Values to filter
    /// @param halfWindowSize Size of the moving average
    /// @return Filtered value
    std::vector<double> movingAverage(std::vector<double> const values, unsigned int const& halfWindowSize);

    template<typename T>
    inline T unwrap(T const& previous, T const& next, T const& period)
    {
        T diff = next - previous;
        if (diff > period / 2)
            diff -= period;
        if (diff < - period / 2)
            diff += period;
        return previous + diff;
    }

    inline double unwrap(double const& previous, double const& next)
    {
        return unwrap(previous, next, 2 * M_PI);
    }


#endif
