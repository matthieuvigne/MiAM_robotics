#include <iostream>

#include <network/client_request.hpp>
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
      std::string received_message;
      new_sock >> received_message;
      
      // Deserialize the client's request
      //~ ClientRequest const client_request(received_message);
      //~ RequestType const request_type = client_request.getType();

      // Respond according the client's request type
      bool shut_down = false;
      std::string answer;
      //~ switch(request_type)
      //~ {
        //~ case RequestType::GET_MEASUREMENTS:
        //~ {
          //~ common::MarkerIdToEstimate estimates;
          //~ this->camera_thread_ptr_->getMarkers(&estimates);
          //~ common::MarkerIdToEstimate::const_iterator it = estimates.cbegin();
          //~ for(it; it != estimates.cend(); ++it)
          //~ {
            //~ std::vector<char> marker_message;
            //~ common::Marker const& marker = it->second;
            //~ marker.serialize(&marker_message);
            //~ // [TODO] : add the server response
          //~ }
          //~ break;
        //~ }
        //~ case RequestType::SHUT_DOWN:
        //~ {
          //~ shut_down = true;
          //~ // [TODO] : serialize the answer
          //~ break;
        //~ }
        //~ case RequestType::UNKNOWN:
        //~ default:
          //~ break;
      //~ }

      // Shut down if request
      if(shut_down) break;
    }
    
    break;
  }
  
  // Shut the server down
  std::cout << "Shutting down the server thread." << std::endl;
}

//--------------------------------------------------------------------------------------------------

} // namespace network
