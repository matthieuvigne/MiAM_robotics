#ifndef APPLICATION_COMMON_HPP
#define APPLICATION_COMMON_HPP

#include <vector>
#include <set>
#include <string>

namespace common {

//--------------------------------------------------------------------------------------------------
// Function declarations
//--------------------------------------------------------------------------------------------------

/* Checks that the pointer is valid and returns it for further use.
 * Throws a runtime error if the pointers turns out to be invalid.
 */
template<typename T> T* checkNotNull(
  T* pointer,
  std::string const& filename,
  int const line);

/* Checks that the const pointer is valid and returns it for further use.
 * Throws a runtime error if the pointers turns out to be invalid.
 */
template<typename T> T const* checkNotNull(
  T const* pointer,
  std::string const& filename,
  int const line);

/* Checks that the given conditions is verified.
 * Throws a runtime error if not and prints the specified error message.
 */
void check(
  bool condition,
  std::string const& error_message,
  std::string const& filename,
  int const line);

/* Checks that the given conditions is verified.
 * Throws a runtime error if not.
 */
void check(
  bool condition,
  std::string const& filename,
  int const line);

// Macros to conveniently call the previous 'check' functions  
#define CHECK_NOTNULL(T) common::checkNotNull(T, __FILE__, __LINE__)  
#define CHECK(condition) common::check(condition, __FILE__, __LINE__)
#define CHECK_MSG(condition, errmsg) common::check(condition, errmsg, __FILE__, __LINE__)

/* Formats the current filename and line in the form "file.cpp [l.line]:"
 * File input must be __FILENAME__
 * Line input must be __LINE__
 */
std::string getCurrentFileAndLine(
  std::string const& file,
  int const line);

//--------------------------------------------------------------------------------------------------
// Inline function definitions
//--------------------------------------------------------------------------------------------------

template<typename T> T* checkNotNull(
  T* pointer,
  std::string const& filename,
  int const line)
{
  check(pointer != NULL, "Pointer is invalid", filename, line);
  return pointer;
}

//--------------------------------------------------------------------------------------------------

template<typename T> T const* checkNotNull(
  T const* pointer,
  std::string const& filename,
  int const line)
{
  check(pointer != NULL, "Pointer is invalid", filename, line);
  return pointer;
}

//--------------------------------------------------------------------------------------------------
  
} // namespace common

#endif // APPLICATION_COMMON_HPP
