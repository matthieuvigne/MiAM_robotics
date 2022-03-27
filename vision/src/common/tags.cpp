#include <common/tags.hpp>

namespace common {

TagFamily getTagFamily(TagId tag_id)
{
  // Initialize the correspondence table when the function is first called.
  typedef std::map<TagId,TagFamily> TagIdToFamily;
  static std::map<TagId,TagFamily> tag_id_to_family;
  if(tag_id_to_family.empty())
  {
    // Tags appended to the robots
    for(TagId tag_id=1u; tag_id<=5u; tag_id++)
      tag_id_to_family[tag_id] = TagFamily::PURPLE_TEAM_ROBOT;
    for(TagId tag_id=6u; tag_id<=10u; tag_id++)
      tag_id_to_family[tag_id] = TagFamily::YELLOW_TEAM_ROBOT;
      
    // Tags reserved for the board playing area
    tag_id_to_family[13u] = TagFamily::TREASURE_BLUE_SAMPLE;
    tag_id_to_family[17u] = TagFamily::ROCK_SAMPLE;
    tag_id_to_family[36u] = TagFamily::TREASURE_GREEN_SAMPLE;
    tag_id_to_family[42u] = TagFamily::CENTRAL_MARKER;
    tag_id_to_family[47u] = TagFamily::TREASURE_RED_SAMPLE;
    
    // Tags reserved for the purple team
    for(TagId tag_id=51u; tag_id<=70u; tag_id++)
      tag_id_to_family[tag_id] = TagFamily::PURPLE_TEAM_MARKER;
    for(TagId tag_id=71u; tag_id<=90u; tag_id++)
      tag_id_to_family[tag_id] = TagFamily::YELLOW_TEAM_MARKER;
  }
  
  // Return the requested tag's family
  TagIdToFamily::const_iterator it = tag_id_to_family.find(tag_id);
  TagFamily const tag_family = (it != tag_id_to_family.cend()) ? it->second : TagFamily::UNKNOWN;
  return tag_family;
}

} // namespace common
