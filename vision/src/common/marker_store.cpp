#include <common/marker_store.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------

void MarkerStore::addMarker(Marker::UniquePtr marker_ptr)
{
  CHECK(marker_ptr);
  CHECK(marker_ptr->isUnique());
  MarkerId const marker_id = marker_ptr->getId();
  unique_markers_.insert(std::make_pair(marker_id, std::move(marker_ptr)));
}

//--------------------------------------------------------------------------------------------------

void MarkerStore::addMarker(double azimuth_deg, double elevation_deg, Marker::UniquePtr marker_ptr)
{
  CHECK(marker_ptr);
  CHECK(!marker_ptr->isUnique());
  CHECK(std::fabs(azimuth_deg) < 180.);
  CHECK(std::fabs(elevation_deg) < 90.);
  int const new_marker_idx = static_cast<int>(multiple_markers_.size());
  azimuth_to_markers_.insert(std::make_pair(azimuth_deg, new_marker_idx));
  elevation_to_markers_.insert(std::make_pair(elevation_deg, new_marker_idx));
  multiple_markers_.insert(std::make_pair(new_marker_idx, std::move(marker_ptr)));
}

//--------------------------------------------------------------------------------------------------

void MarkerStore::forEachMarker(std::function<void(Marker const&)>& action) const
{
  // Iterate over the unique markers
  for(std::map<MarkerId,Marker::UniquePtr>::value_type const& pair : unique_markers_)
    action(*pair.second);
  
  // Iterate over the multiple markers
  for(std::map<int,Marker::UniquePtr>::value_type const& pair : multiple_markers_)
    action(*pair.second);
}

//--------------------------------------------------------------------------------------------------

bool MarkerStore::getUniqueMarker(MarkerId marker_id, Marker const** marker_ptr) const
{
  CHECK_NOTNULL(marker_ptr);
  if(!Marker::isUnique(marker_id)) return false;
  std::map<MarkerId,Marker::UniquePtr>::const_iterator it = unique_markers_.find(marker_id);
  if(it == unique_markers_.cend()) return false;
  *marker_ptr = it->second.get();
  return true;
}

//--------------------------------------------------------------------------------------------------

} // namespace common
