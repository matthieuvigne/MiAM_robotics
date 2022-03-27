#include <cassert>
#include <csignal>

#include <common/sigint-breaker.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

SigintBreaker::SigintBreaker()
: previous_handler_   (signal(SIGINT, &SigintBreaker::handler))
{
  assert(!this->is_instantiated_);
  this->is_instantiated_ = true;
  this->is_sigint_raised_ = false;
}
  
//--------------------------------------------------------------------------------------------------

SigintBreaker::~SigintBreaker()
{
  assert(signal(SIGINT, this->previous_handler_) == &SigintBreaker::handler);
  this->is_instantiated_ = false;
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

bool SigintBreaker::isBreakRequested() const
{
  return this->is_sigint_raised_;
}

//--------------------------------------------------------------------------------------------------

void SigintBreaker::handler(int signal)
{
  assert(signal == SIGINT);
  SigintBreaker::is_sigint_raised_ = true;
}

//--------------------------------------------------------------------------------------------------
// Values
//--------------------------------------------------------------------------------------------------

bool SigintBreaker::is_instantiated_ = false;
bool SigintBreaker::is_sigint_raised_ = false;

//--------------------------------------------------------------------------------------------------

} // namespace common
