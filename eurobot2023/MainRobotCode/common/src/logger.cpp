#include <common/logger.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

FileLogger::FileLogger(std::string const& filename)
: filename_ (filename),
  logfile_  (filename, std::ofstream::out | std::ofstream::trunc)
{
  logfile_.close();
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void FileLogger::init(std::string const& filename)
{
  global_file_logger_ptr.reset(new FileLogger(filename));
}

//--------------------------------------------------------------------------------------------------

void ConsoleLogger::init()
{
  global_console_logger_ptr.reset(new ConsoleLogger);
}

//--------------------------------------------------------------------------------------------------
// Global variable definition
//--------------------------------------------------------------------------------------------------

FileLogger::UniquePtr global_file_logger_ptr = nullptr;
ConsoleLogger::UniquePtr global_console_logger_ptr = nullptr;

//--------------------------------------------------------------------------------------------------

} // namespace common
