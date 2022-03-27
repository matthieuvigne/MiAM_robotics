#ifndef COMMON_TAGS_HPP
#define COMMON_TAGS_HPP

#include <map>

namespace common {

//--------------------------------------------------------------------------------------------------
// Structure/Enum declarations
//--------------------------------------------------------------------------------------------------

typedef uint8_t TagId;
enum class TagFamily {
  UNKNOWN,
  CENTRAL_MARKER,
  ROCK_SAMPLE,
  TREASURE_RED_SAMPLE,
  TREASURE_GREEN_SAMPLE,
  TREASURE_BLUE_SAMPLE,
  PURPLE_TEAM_ROBOT,
  YELLOW_TEAM_ROBOT,
  PURPLE_TEAM_MARKER,
  YELLOW_TEAM_MARKER
}; // enum class TagFamily

TagFamily getTagFamily(TagId id);

//--------------------------------------------------------------------------------------------------

} // namespace common

#endif // COMMON_TAGS_HPP
