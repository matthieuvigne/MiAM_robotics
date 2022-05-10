#ifndef COMMON_MARKER_STORE_HPP
#define COMMON_MARKER_STORE_HPP

#include <common/macros.hpp>
#include <common/marker.hpp>

namespace common {

class MarkerStore {

public:
  POINTER_TYPEDEF(MarkerStore);

public:
  MarkerStore() = default;
  virtual ~MarkerStore() = default;

public:

  // Add markers
  void addMarker(Marker::UniquePtr marker);
  void addMarker(double azimuth_deg, double elevation_deg, Marker::UniquePtr marker);

  // Getters
  bool getUniqueMarker(MarkerId marker_id, Marker const** marker_ptr) const;
  void forEachMarker(std::function<void(Marker const&)>& action) const;

private:

  // Unique marker
  std::map<MarkerId,Marker::UniquePtr> unique_markers_;

  // Multiple markers
  std::map<int,Marker::UniquePtr> multiple_markers_;
  std::map<double,int> azimuth_to_markers_;
  std::map<double,int> elevation_to_markers_;

}; // class MarkerStore

} // namespace common

#endif // COMMON_MARKER_STORE_HPP
