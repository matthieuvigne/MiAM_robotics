#ifndef COMMON_LOGGER_HPP
#define COMMON_LOGGER_HPP

#include <fstream>
#include <iostream>
#include <mutex>

#include <common/macros.hpp>

namespace common {

// https://stackoverflow.com/questions/43105684/in-c-trying-to-write-a-newline-after-each-operator-overload-cascade

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class Logger {

public:
  POINTER_TYPEDEF(Logger);
  DISALLOW_EVIL_CONSTRUCTORS(Logger);
  Logger(std::string const& filename);
  virtual ~Logger(){}

public:

  // Guard class instanciated as rvalue in the stream
  // Necessary to move the state of being the last element along the chain
  class RvalLog {
    friend class Logger;
    public:
      RvalLog(std::ofstream* os, std::mutex* mtx) : os_(os), mutex_(mtx) {};
      RvalLog(RvalLog&& rhs) : os_(rhs.os_) { rhs.os_ = nullptr; rhs.mutex_ = nullptr; }
      ~RvalLog(){ if(os_){ *os_ << std::endl; os_->close(); mutex_->unlock(); }}
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

}; // class Logger

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

template<typename T>
Logger::RvalLog Logger::operator <<(T const& message)
{
  mutex_.lock();
  logfile_.open(filename_, std::ofstream::app);
  logfile_ << message;
  return RvalLog(&logfile_, &mutex_); // << message;
}

//--------------------------------------------------------------------------------------------------
// Global variables
//--------------------------------------------------------------------------------------------------

extern Logger::UniquePtr global_logger_ptr;

} // namespace common

//--------------------------------------------------------------------------------------------------
// Macros declaration
//--------------------------------------------------------------------------------------------------

#define LOGGER                                                    \
  if(common::global_logger_ptr == nullptr)                        \
    throw std::runtime_error("Logger has not been initialized");  \
  (*common::global_logger_ptr) << "[" << __FILENAME__ << ": l."   \
    << __LINE__ << "] "

//--------------------------------------------------------------------------------------------------

#endif // COMMON_LOGGER_HPP
