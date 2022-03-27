#ifndef COMMON_TIME_HPP
#define COMMON_TIME_HPP

#include <chrono>

namespace common {

//--------------------------------------------------------------------------------------------------
// Declaration
//--------------------------------------------------------------------------------------------------

typedef std::chrono::high_resolution_clock Time;
typedef Time::time_point TimePoint;
typedef Time::duration Duration;

inline int64_t convertToNanoseconds(TimePoint const& time_point);
inline int64_t convertToNanoseconds(Duration const& duration);
inline double convertToSeconds(TimePoint const& time_point);
inline double convertToSeconds(Duration const& duration);

//--------------------------------------------------------------------------------------------------
// Definition
//--------------------------------------------------------------------------------------------------

int64_t convertToNanoseconds(TimePoint const& time_point)
{
  return std::chrono::time_point_cast<std::chrono::nanoseconds>(time_point)
    .time_since_epoch().count();
}

//--------------------------------------------------------------------------------------------------

int64_t convertToNanoseconds(Duration const& duration)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

//--------------------------------------------------------------------------------------------------

double convertToSeconds(TimePoint const& time_point)
{
  return std::chrono::time_point_cast<std::chrono::seconds>(time_point)
    .time_since_epoch().count();
}

//--------------------------------------------------------------------------------------------------

double convertToSeconds(Duration const& duration)
{
  return std::chrono::duration_cast<std::chrono::seconds>(duration).count();
}

//--------------------------------------------------------------------------------------------------

} // namespace common

#endif // COMMON_TIME_HPP
