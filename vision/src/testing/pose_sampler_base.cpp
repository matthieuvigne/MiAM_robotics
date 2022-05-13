#include <testing/pose_sampler_base.hpp>

namespace testing {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

PoseSamplerBase::PoseSamplerBase(Type type)
: type_       (type),
  generator_  ()
{}

//--------------------------------------------------------------------------------------------------

} // namespace testing
