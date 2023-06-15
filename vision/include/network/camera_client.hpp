/// \file uCListener.h
/// \brief Communication with a slave micro-controller to access some sensors.
///
/// \details This uC constantly broadcast the status of the sensor. This file runs a thread that listen to the serial
///          port, update the value as needed and makes it available through a specific data structure.
///
///    \note     All functions in this header should be prefixed with uCListener_.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef NETWORK_CAMERA_CLIENT_HPP
#define NETWORK_CAMERA_CLIENT_HPP

#include <string>
#include <mutex>

#include <miam_utils/network/client_socket.hpp>
#include <common/marker.hpp>

namespace network {

class CameraClient {

  public:
    CameraClient();
    CameraClient(CameraClient const& c);

  public:
    /// Run the camera client thread.
    void run();
    /// Set the robot playing side.
    void updateRobotSide(bool const& isPlayingRightSide);
    void shutDown();
    int getNumberOfMarkersInExcavationSite();
    /// Access to the list of markers seen by the camera.
    std::vector<common::Marker> markers_;

  private:
    /// Init communication
    bool init(std::string const& host, int const& port);
    void processMarkers();
    int nMarkersInExcavationSite_ = 0;

  private:
    network::ClientSocket sock_;
    bool needColorUpdate_ = true;
    bool isPlayingRightSide_ = false;
    bool shut_down_ = false;
    std::mutex mutex_;
};

} // namespace network

#endif // NETWORK_CAMERA_CLIENT_HPP