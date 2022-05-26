/// \file CameraClient.c
/// \brief This file implements the sock_ for the camera
///
/// \author MiAM Robotique, Matthieu Vigne, Rodolphe Dubois
/// \copyright GNU GPLv3

#include <common/common.hpp>
#include <common/logger.hpp>
#include <common/marker.hpp>
#include <network/camera_client.hpp>
#include <network/client_request.hpp>
#include <network/server_response.hpp>
#include <network/socket_exception.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructors
//--------------------------------------------------------------------------------------------------

CameraClient::CameraClient()
{
  common::ConsoleLogger::init();
}

//--------------------------------------------------------------------------------------------------

CameraClient::CameraClient(CameraClient const& c):
  sock_(c.sock_)
{}

//--------------------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------------------

bool CameraClient::init(std::string const& host, int const& port)
{
  try
  {
      sock_.connect(host, port);
  }
  catch(network::SocketException const& e)
  {
      std::cout << "failed to connect" << std::endl;
      return false;
  }
  return true;
}

//--------------------------------------------------------------------------------------------------

void CameraClient::updateRobotSide(bool const& isPlayingRightSide)
{
  mutex_.lock();
  needColorUpdate_ = isPlayingRightSide_ != isPlayingRightSide;
  isPlayingRightSide_ = isPlayingRightSide;
  mutex_.unlock();
}

//--------------------------------------------------------------------------------------------------

void CameraClient::run()
{
  while(true)
  {
    try
    {
      CONSOLE << "Trying to connect";
      //~ sock_.connect("192.168.6.20", 30000);
      //~ sock_.connect("192.168.6.42", 30000);
      sock_.connect("192.168.7.2", 30000);
      CONSOLE << "Connected!";
      usleep(50000);
      break;
    }
    catch(network::SocketException const&)
    {
      usleep(1e6);
      CONSOLE << "Failed to connect, retyring...";
    }
  }
  std::cout << "created client" << std::endl;
  CONSOLE << "OK";
  // bool lastSide = !(*isPlayingRightSide);

  network::ClientRequest::UniquePtr request_ptr = nullptr;
  network::ServerResponse::UniquePtr response_ptr = nullptr;

  // Send messages to the server
  std::shared_ptr<common::MarkerList> markers = nullptr;
  while(true)
  {
    try
    {
      // Build the message to send to the server
      network::MessageType message_type;
      mutex_.lock();
      if (needColorUpdate_)
      {
        message_type = network::MessageType::INITIALIZATION;
        request_ptr.reset(new network::ClientRequest(message_type));
        if (isPlayingRightSide_)
            request_ptr->getParamsAs<common::Team>() = common::Team::PURPLE;
        else
            request_ptr->getParamsAs<common::Team>() = common::Team::YELLOW;
        std::cout << "Reset side" << isPlayingRightSide_ << std::endl;
        needColorUpdate_ = false;
      }
      else
      {
        message_type = network::MessageType::GET_MEASUREMENTS;
        request_ptr.reset(new network::ClientRequest(message_type));
      }
      mutex_.unlock();

      // Serialize and send the message to the server
      std::string request_str;
      request_ptr->serialize(&request_str);
      sock_ << request_str;

      // Get the response from the server
      try
      {
        std::string response_str;
        sock_ >> response_str;
        response_ptr.reset(new network::ServerResponse(response_str));
        response_ptr->deserialize(response_str);
        if(message_type == network::MessageType::GET_MEASUREMENTS)
        {
          common::MarkerList const& markers =
              response_ptr->getParamsAs<common::MarkerList>();
          markers_.clear();
          for (auto const& m : markers)
          {
            markers_.push_back(m);
          }
          std::cout << "current marker" << markers_.size() << std::endl;
          for (auto const& m : markers_)
          {
            std::cout << " Id " << std::to_string(m.getId()) << " pos "
              << m.getTWM()->translation().transpose() << std::endl;
          }
          processMarkers();
        }
      }
      catch (std::runtime_error const& err)
      {
        continue;
      }
      // Sleep before next iteration
      usleep(100000);
    }
    catch(network::SocketException const& e)
    {
      std::cout << "network error"  << std::endl;
      usleep(100000);
    }
  }
}

//--------------------------------------------------------------------------------------------------

void CameraClient::processMarkers()
{
  mutex_.lock();
  Eigen::Vector3d center;
  if (isPlayingRightSide_)
      center << 2.025, 0.525, 0;
  else
      center << 0.975, 0.525, 0;
  nMarkersInExcavationSite_ = 0;
  for (auto const& m : markers_)
  {
    // Excavation site diameter: 0.5m
    if ((m.getTWM()->translation() - center).norm() < 0.6)
        nMarkersInExcavationSite_++;
  }
  mutex_.unlock();
}

//--------------------------------------------------------------------------------------------------

int CameraClient::getNumberOfMarkersInExcavationSite()
{
  mutex_.lock();
  int res = nMarkersInExcavationSite_;
  mutex_.unlock();
  return res;
}

//--------------------------------------------------------------------------------------------------

} // namespace network
