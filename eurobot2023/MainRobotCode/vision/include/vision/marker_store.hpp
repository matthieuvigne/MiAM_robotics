#ifndef VISION_MARKER_STORE_HPP
#define VISION_MARKER_STORE_HPP

#include <common/macros.hpp>
#include <vision/marker.hpp>

namespace vision {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class MarkerStore {

public:
  POINTER_TYPEDEF(MarkerStore);

public:
  MarkerStore() = default;
  virtual ~MarkerStore() = default;

public:

  void addMarker(Marker::UniquePtr marker);
  Marker const* getUniqueMarker(MarkerId marker_id) const;
  void forEachMarker(std::function<void(Marker const&)> const& action) const;
  size_t forEachMultipleMarkerRemoveIf(std::function<bool(Marker const&)> const& condition);
  inline size_t getNumMarkers() const;
  inline size_t getNumUniqueMarkers() const;
  inline size_t getNumMultipleMarkers() const;

private:
  std::map<MarkerId,Marker::UniquePtr> unique_markers_;
  std::multimap<MarkerId,Marker::UniquePtr> multiple_markers_;

}; // class MarkerStore

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

size_t MarkerStore::getNumMarkers() const
{
  return unique_markers_.size() + multiple_markers_.size();
}

//--------------------------------------------------------------------------------------------------

size_t MarkerStore::getNumUniqueMarkers() const
{
  return unique_markers_.size();
}

//--------------------------------------------------------------------------------------------------

size_t MarkerStore::getNumMultipleMarkers() const
{
  return multiple_markers_.size();
}

//--------------------------------------------------------------------------------------------------

} // namespace vision

#endif // VISION_MARKER_STORE_HPP
