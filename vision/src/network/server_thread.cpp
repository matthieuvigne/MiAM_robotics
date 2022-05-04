#include <iostream>

#include <network/client_request.hpp>
#include <network/server_response.hpp>
#include <network/server_thread.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructors and destructors
//--------------------------------------------------------------------------------------------------

ServerThread::ServerThread(int port)
: ServerSocket(port)
{
  this->thread_ptr_.reset(new std::thread([=](){this->serverThread();}));
}

//--------------------------------------------------------------------------------------------------

ServerThread::~ServerThread(){
  this->thread_ptr_->join();  
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void ServerThread::serverThread()
{
  // Wait for the client's connection
  LOG("Server thread is launched");
  ClientRequest::UniquePtr request_ptr;
  ServerResponse::UniquePtr response_ptr;
  while(true)
  {
    // Wait for the client's connection
    LOG("Waiting for the client's connection");
    ServerSocket client_sock;
    this->accept(client_sock);
    LOG("Accepted a client");

    // Wait for the client's requests
    while(true)
    {
      // Waiting for the client's request
      std::string received_message;
      client_sock >> received_message;

      // Deserialize the client's request and respond
      bool shut_down = false;
      void* response_params_ptr = NULL;
      //~ MessageType const request_type = ClientRequest::deserializeType(received_message);
      ClientRequest const client_request(received_message);
      common::MarkerIdToEstimate estimates;
      switch(client_request.getType())
      {
        case MessageType::INITIALIZATION:
        {
          ClientRequest::Initialization team =
            *static_cast<ClientRequest::Initialization const*>(client_request.getParams());
          switch(team)
          {
            case ClientRequest::Initialization::PURPLE_TEAM:
              camera_thread_ptr_->setTeam(camera::CameraThread::Team::PURPLE);
              break;
            case ClientRequest::Initialization::YELLOW_TEAM:
              camera_thread_ptr_->setTeam(camera::CameraThread::Team::YELLOW);
              break;
            case ClientRequest::Initialization::UNKNOWN:
            default:
              break;
          }
          break;
        }
        case MessageType::GET_MEASUREMENTS:
        {
          camera_thread_ptr_->getMarkers(&estimates);
          response_params_ptr = &estimates;
          break;
        }
        case MessageType::SHUT_DOWN:
        {
          shut_down = true;
          break;
        }
        case MessageType::UNKNOWN:
        default:
          break;
      }
      response_ptr.reset(new ServerResponse(client_request.getType(), response_params_ptr));

      // Send the response to the client
      std::string response_str;
      response_ptr->serialize(&response_str);
      client_sock << response_str;

      // Shut down if request
      if(shut_down) break;
    }
    
    break;
  }
}

//--------------------------------------------------------------------------------------------------

} // namespace network
