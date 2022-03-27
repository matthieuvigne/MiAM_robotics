#ifndef COMMON_SIGINT_BREAKER_HPP
#define COMMON_SIGINT_BREAKER_HPP

namespace common {

// The code from this class is inspired from : 
// https://github.com/ethz-asl/maplab/common/maplab-common/include/maplab-common/sigint-breaker.h
class SigintBreaker {

public:
  SigintBreaker();
  ~SigintBreaker();

public:
  bool isBreakRequested() const;

private:
  static void handler(int signal);
  static bool is_instantiated_;
  static bool is_sigint_raised_;
  void (*previous_handler_)(int);

}; // class SigintBreaker

} // namespace common

#endif // COMMON_SIGINT_BREAKER_HPP
