#include <iostream>

#include <common/marker.hpp>
#include <network/client.hpp>
#include <network/client_request.hpp>
#include <network/server_response.hpp>

// Global variables

// Dummy client main routine
int main(int argc, char* argv[])
{
  // Initialize the client
  std::cout << "Launching the dummy client" << std::endl;
  network::Client client("localhost", 30000);
  network::ClientRequest::UniquePtr request_ptr = nullptr;
  network::ServerResponse::UniquePtr response_ptr = nullptr;
  
  // Send messages to the client
  int request_idx = 0;
  int constexpr max_num_requests = 20;
  common::MarkerIdToEstimate markers;
  while(true)
  {
    // Build and send the request to the server
    std::string request_str;
    network::MessageType const message_type = network::MessageType::GET_MEASUREMENTS;
    request_ptr.reset(new network::ClientRequest(message_type));
    request_ptr->serialize(&request_str);
    client << request_str;

    // Get the response from the server
    std::string response_str;
    client >> response_str;
    response_ptr.reset(new network::ServerResponse(message_type, &markers));
    response_ptr->deserialize(response_str);
    
    // Prepare for next iteration
    request_idx += 1;
    if(request_idx >= max_num_requests) break;
  }
  
  // Send the shutdown request to the server
  std::string request_str;
  network::MessageType const message_type = network::MessageType::SHUT_DOWN;
  request_ptr.reset(new network::ClientRequest(message_type));
  request_ptr->serialize(&request_str);
  client << request_str;
  
  // Get the response from the server
  std::string response_str;
  client >> response_str;
  response_ptr.reset(new network::ServerResponse(message_type));
  response_ptr->deserialize(response_str);
  std::cout << "Shutting down the client" << std::endl;
}
