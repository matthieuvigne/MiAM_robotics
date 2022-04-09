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
  while(true)
  {
    // Wait for the client's connection
    ServerSocket client_sock;
    this->accept(client_sock);

    // Wait for the client's requests
    while(true)
    {
      // Waiting for the client's request
      std::string received_message;
      client_sock >> received_message;
      
      // Deserialize the client's request
      ClientRequest const client_request(received_message);
      MessageType const request_type = client_request.getType();

      // Respond according the client's request type
      bool shut_down = false;
      void* params_ptr = NULL;
      switch(request_type)
      {
        case MessageType::GET_MEASUREMENTS:
        {
          common::MarkerIdToEstimate estimates;
          this->camera_thread_ptr_->getMarkers(&estimates);
          params_ptr = &estimates;
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
      ServerResponse server_response(request_type, params_ptr);

      // Send the response to the client
      std::string response;
      server_response.serialize(&response);
      client_sock << response;

      // Shut down if request
      if(shut_down) break;
    }
    
    break;
  }
}

//--------------------------------------------------------------------------------------------------

} // namespace network
