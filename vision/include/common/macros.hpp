#ifndef COMMON_MACROS_HPP
#define COMMON_MACROS_HPP

#include <iostream>
#include <memory>

#include <string.h>

#define POINTER_TYPEDEF(TypeName)                   \
  typedef std::shared_ptr<TypeName> Ptr;            \
  typedef std::shared_ptr<TypeName const> ConstPtr; \
  typedef std::unique_ptr<TypeName> UniquePtr

#define DISALLOW_EVIL_CONSTRUCTORS(TypeName)  \
  TypeName(TypeName const&) = delete;         \
  void operator=(TypeName const&) = delete

#define GET_AS_CASTER                                 \
  template<typename T>                                \
  T& getAs() {                                        \
    CHECK_NOTNULL(this);                              \
    T* result = dynamic_cast<T*>(this);               \
    CHECK_NOTNULL(result);                            \
    return *result;                                   \
  }                                                   \
  template<typename T>                                \
  T const& getAs() const {                            \
      CHECK_NOTULL(this);                             \
      T const* result = dynamic_cast<T const*>(this); \
      CHECK_NOTNULL(result);                          \
      return *this;                                   \
  }

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG(message)                                      \
  std::cout << "[" << __FILENAME__ << ": l."              \
    << __LINE__ << "] " << message << std::endl

#define CHECK(condition)                                  \
if(!(condition))                                          \
{                                                         \
  throw std::runtime_error(   std::string(__FILENAME__)   \
                            + std::string(" [")           \
                            + std::to_string(__LINE__)    \
                            + std::string("] ")           \
                            + std::string(#condition)   );\
}

#define CHECK_NOTNULL(pointer)                                \
if(pointer == 0)                                              \
{                                                             \
  throw std::runtime_error(   std::string(__FILENAME__)       \
                            + std::string("[")                \
                            + std::to_string(__LINE__)        \
                            + std::string("]")                \
                            + std::string("Invalid pointer"));\
}

#define CHECKLOG(condition, message)                      \
if(!(condition))                                          \
{                                                         \
  throw std::runtime_error(   std::string(__FILENAME__)   \
                            + std::string(" [")           \
                            + std::to_string(__LINE__)    \
                            + std::string("] ")           \
                            + std::string(#message));     \
}

#endif // COMMON_MACRO_HPP
