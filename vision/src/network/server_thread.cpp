#include <iostream>

#include <common/logger.hpp>
#include <network/client_request.hpp>
#include <network/server_response.hpp>
#include <network/server_thread.hpp>
#include <network/socket_exception.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructors and destructors
//--------------------------------------------------------------------------------------------------

ServerThread::ServerThread(int port)
: ServerSocket(port)
{
  thread_ptr_.reset(new std::thread([=](){serverThread();}));
}

//--------------------------------------------------------------------------------------------------

ServerThread::~ServerThread(){
  thread_ptr_->join();  
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void ServerThread::serverThread()
{
  // Wait for the client's connection
  CONSOLE << "Server thread is launched";
  ClientRequest::UniquePtr request_ptr;
  ServerResponse::UniquePtr response_ptr;
  while(true)
  {
    try
    {
      // Wait for the client's connection
      CONSOLE << "Waiting for the client's connection";
      ServerSocket client_sock;
      accept(client_sock);
      CONSOLE << "Accepted a client";

      // Wait for the client's requests
      while(true)
      {
        // Waiting for the client's request
        std::string received_message;
        client_sock >> received_message;

        // Deserialize the client's request and respond
        bool shut_down = false;
        std::shared_ptr<void> response_params_ptr = nullptr;
        ClientRequest const client_request(received_message);
        switch(client_request.getType())
        {
          case MessageType::INITIALIZATION:
          {
            common::Team const& team = client_request.getParamsAs<common::Team>();
            camera_thread_ptr_->setTeam(team);
            break;
          }
          case MessageType::GET_MEASUREMENTS:
          {
            std::shared_ptr<common::MarkerIdToEstimate> estimates(new common::MarkerIdToEstimate);
            camera_thread_ptr_->getMarkers(estimates.get());
            response_params_ptr = std::move(estimates);
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
    catch(SocketException const& e)
    {
      CONSOLE << "Lost connection with the client";
      CONSOLE << e.description();
    }
  }
}

//--------------------------------------------------------------------------------------------------

} // namespace network
