#include <iomanip>
#include <set>
#include <sstream>
#include <string.h>

#include <common/assert.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------

void check(
  bool condition,
  std::string const& error_message,
  std::string const& filename,
  int const line)
{
  std::string const prefix = getCurrentFileAndLine(filename, line);
  if(!condition)
  {
    throw std::runtime_error(prefix + error_message);
  }
}

//--------------------------------------------------------------------------------------------------

void check(
  bool condition,
  std::string const& filename,
  int const line)
{
  return check(condition, "Error !", filename, line);
}

//--------------------------------------------------------------------------------------------------

std::string getCurrentFileAndLine(
  std::string const& file,
  int const line)
{
  std::string const filename = strrchr(file.c_str(), '/') ? strrchr(file.c_str(), '/') + 1 : file;
  return filename + " [l." + std::to_string(line) + "] ";
}

//--------------------------------------------------------------------------------------------------

} // namespace common
