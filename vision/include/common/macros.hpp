#ifndef COMMON_MACROS_HPP
#define COMMON_MACROS_HPP

#include <memory>

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

#endif // COMMON_MACRO_HPP
