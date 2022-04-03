#include <iostream>

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
    std::cout << "Waiting for the client's connection...";
    ServerSocket new_sock;
    this->accept(new_sock);
    std::cout << "ok" << std::endl;

    // Wait for the client's requests
    while(true)
    {
      // Waiting for the client's request
      std::string client_request;
      new_sock >> client_request;
      
      // Get all the measurements and serialize them
      common::MarkerIdToEstimate estimates;
      this->camera_thread_ptr_->getMarkers(&estimates);
      std::string answer;
      
      // Send data to the client
      if(client_request == "stop") break;
      new_sock << answer;
      

    }
    break;
  }
  
  // Shut the server down
  std::cout << "Shutting down the server thread." << std::endl;
}

//--------------------------------------------------------------------------------------------------

} // namespace network
