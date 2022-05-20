#ifndef COMMON_LOGGER_HPP
#define COMMON_LOGGER_HPP

#include <fstream>
#include <iostream>
#include <mutex>

#include <common/macros.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class FileLogger {

public:
  POINTER_TYPEDEF(FileLogger);
  DISALLOW_EVIL_CONSTRUCTORS(FileLogger);
  FileLogger(std::string const& filename);
  virtual ~FileLogger(){}

public:

  // Guard class instanciated as rvalue in the stream
  // Necessary to move the state of being the last element along the chain
  class RvalLog {
    friend class FileLogger;
    public:
      RvalLog(std::ofstream* os, std::mutex* mtx) : os_(os), mutex_(mtx) {};
      RvalLog(RvalLog&& rhs) : os_(rhs.os_), mutex_(rhs.mutex_) { rhs.os_ = nullptr; rhs.mutex_ = nullptr; }
      ~RvalLog(){ if(os_){ *os_ << std::endl; os_->close(); } if(mutex_) mutex_->unlock(); }
    public:
      template<typename T>
      RvalLog operator <<(T const& t) && { *os_ << t; return std::move(*this); }
    private:
      std::ofstream* os_;
      std::mutex* mutex_;
  }; // class RvalLog

public:
  template<typename T>
  RvalLog operator <<(T const& message);
  static void init(std::string const& filename);

private:
  std::mutex mutex_;
  std::string filename_;
  std::ofstream logfile_;

}; // class FileLogger

//--------------------------------------------------------------------------------------------------

class ConsoleLogger {

public:
  POINTER_TYPEDEF(ConsoleLogger);
  DISALLOW_EVIL_CONSTRUCTORS(ConsoleLogger);
  ConsoleLogger() = default;
  virtual ~ConsoleLogger(){}

public:

  // Guard class instanciated as rvalue in the stream
  // Necessary to move the state of being the last element along the chain
  class RvalLog {
    friend class ConsoleLogger;
    public:
      RvalLog(std::ostream* os, std::mutex* mtx) : os_(os), mutex_(mtx) { CHECK_NOTNULL(os_); };
      RvalLog(RvalLog&& rhs) : os_(rhs.os_), mutex_(rhs.mutex_) { rhs.os_ = nullptr; rhs.mutex_ = nullptr; }
      ~RvalLog(){ if(os_) *os_ << std::endl; if(mutex_) mutex_->unlock(); }
    public:
      template<typename T>
      RvalLog operator <<(T const& t) && { *os_ << t; return std::move(*this); }
    private:
      std::ostream* os_;
      std::mutex* mutex_;
  }; // class RvalLog

public:
  template<typename T>
  RvalLog operator <<(T const& message);
  static void init();

private:
  std::mutex mutex_;

}; // class ConsoleLogger

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

template<typename T>
FileLogger::RvalLog FileLogger::operator <<(T const& message)
{
  mutex_.lock();
  logfile_.open(filename_, std::ofstream::app);
  logfile_ << message;
  return RvalLog(&logfile_, &mutex_);
}

//--------------------------------------------------------------------------------------------------

template<typename T>
ConsoleLogger::RvalLog ConsoleLogger::operator <<(T const& message)
{
  mutex_.lock();
  std::cout << message;
  return RvalLog(&std::cout, &mutex_);
}

//--------------------------------------------------------------------------------------------------
// Global variables
//--------------------------------------------------------------------------------------------------

extern FileLogger::UniquePtr global_file_logger_ptr;
extern ConsoleLogger::UniquePtr global_console_logger_ptr;

} // namespace console

//--------------------------------------------------------------------------------------------------
// Macros declaration
//--------------------------------------------------------------------------------------------------

#define INIT_LOGGERS(Filename)          \
  common::FileLogger::init(Filename);   \
  common::ConsoleLogger::init();

//--------------------------------------------------------------------------------------------------

// #define LOGFILE                                                      \
// if(common::global_file_logger_ptr == nullptr)                        \
//   throw std::runtime_error("File logger has not been initialized");  \
// (*common::global_file_logger_ptr) << "[" << __FILENAME__ << ": l."   \
//   << __LINE__ << "] "

#define LOGFILE                                                      \
if(common::global_file_logger_ptr == nullptr)                        \
  throw std::runtime_error("File logger has not been initialized");  \
(*common::global_file_logger_ptr)
//--------------------------------------------------------------------------------------------------

#define CONSOLE                                                         \
if(common::global_console_logger_ptr == nullptr)                        \
  throw std::runtime_error("Console logger has not been initialized");  \
(*common::global_console_logger_ptr) << "[" << __FILENAME__ << ": l."   \
  << __LINE__ << "] "

//--------------------------------------------------------------------------------------------------

#endif // COMMON_LOGGER_HPP
