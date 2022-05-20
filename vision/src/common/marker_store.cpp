#include <common/marker_store.hpp>
#include <common/logger.hpp>

double const MARKER_MIN_DISTANCE = 0.010;

namespace common {

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------

void MarkerStore::addMarker(Marker::UniquePtr marker_ptr)
{
  CHECK(marker_ptr);
  MarkerId const marker_id = marker_ptr->getId();
  if(marker_ptr->isUnique())
    unique_markers_[marker_id] = std::move(marker_ptr);
  else
  {
    std::multimap<MarkerId,Marker::UniquePtr>::iterator it = multiple_markers_.lower_bound(marker_id);
    while(it != multiple_markers_.upper_bound(marker_id))
    {
      if ((it->second->getTWM()->translation() - marker_ptr->getTWM()->translation()).norm() < MARKER_MIN_DISTANCE)
      {
        it = multiple_markers_.erase(it);

        LOGFILE << "Removing marker too close to new marker:" << static_cast<int>(it->second->getId());
        LOGFILE << "    TWM" << it->second->getTWM()->translation().transpose();
      }
      else
      {
        it ++;
      }
    }
    multiple_markers_.insert(std::make_pair(marker_id, std::move(marker_ptr)));
  }
}

//--------------------------------------------------------------------------------------------------

void MarkerStore::forEachMarker(std::function<void(Marker const&)> const& action) const
{
  // Iterate over the unique markers
  for(std::map<MarkerId,Marker::UniquePtr>::value_type const& pair : unique_markers_)
    action(*pair.second);
  // Iterate over the multiple markers
  for(std::map<MarkerId,Marker::UniquePtr>::value_type const& pair : multiple_markers_)
    action(*pair.second);
}

//--------------------------------------------------------------------------------------------------

size_t MarkerStore::forEachMultipleMarkerRemoveIf(
  std::function<bool(Marker const&)> const& condition)
{
  size_t num_removed_markers = 0u;
  std::multimap<MarkerId,Marker::UniquePtr>::iterator it = multiple_markers_.begin();
  while(it != multiple_markers_.end())
  {
    if(condition(*it->second))
    {
      it = multiple_markers_.erase(it);
      num_removed_markers += 1u;
      continue;
    }
    it = std::next(it);
  }
  return num_removed_markers;
}

//--------------------------------------------------------------------------------------------------

Marker const* MarkerStore::getUniqueMarker(MarkerId marker_id) const
{
  if(!Marker::isUnique(marker_id)) return 0;
  std::map<MarkerId,Marker::UniquePtr>::const_iterator it = unique_markers_.find(marker_id);
  if(it == unique_markers_.cend()) return 0;
  return it->second.get();
}

//--------------------------------------------------------------------------------------------------

} // namespace common
