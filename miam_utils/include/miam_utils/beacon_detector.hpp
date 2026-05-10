#ifndef MIAM_UTILS_BEACON_DETECTOR_HPP
#define MIAM_UTILS_BEACON_DETECTOR_HPP

#include <deque>
#include <functional>

#include "miam_utils/lidar_point.hpp"
#include "miam_utils/Metronome.h"

struct Beacon {
    double x;   // x absolute position of the beacon
    double y;   // y absolute position of the beacon
};

struct DetectedBeacon {
    LidarPoint point;
    double addedTime = 0.0;
    int nPoints = 0;
    int ref_idx = -1;
};

typedef std::vector<LidarPoint> Blob;

class BeaconDetector {

    public:
        BeaconDetector();
        //template<typename... Args>
        //BeaconDetector(Args&&... beacons);

    public:
        bool detect(Blob const& blob);
        void remove_outdated_beacons(double timeout);
        void update_robot_position(
            double& x, double const sigma_x,
            double& y, double const sigma_y,
            double& theta, double const sigma_theta) const;
        void for_each_detected_beacon(
            std::function<void(DetectedBeacon const&)> action);

    protected:
        static double constexpr R_ = 58; // [mm] radius of the beacons
        std::vector<Beacon> reference_beacons_;
        std::deque<DetectedBeacon> detected_beacons_;
        Metronome timeHandler_;

}; // class BeaconDetector

//-------------------------------------------------------------------------------------------------

/*template<typename... Args>
inline BeaconDetector::BeaconDetector(Args&&... beacons)
{
    static_assert((std::is_convertible_v<Args,Beacon>&& ...));
    (reference_beacons_.push_back(beacons), ...);
    return;
}*/

//-------------------------------------------------------------------------------------------------

#endif // MIAM_UTILS_BEACON_DETECTOR_HPP