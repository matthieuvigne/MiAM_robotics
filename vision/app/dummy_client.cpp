#include <iostream>
#include <stdlib.h>

#include <common/common.hpp>
#include <common/logger.hpp>
#include <common/marker.hpp>
#include <network/client.hpp>
#include <network/client_request.hpp>
#include <network/server_response.hpp>
#include <network/socket_exception.hpp>

// Dummy client main routine
int main(int argc, char* argv[])
{
  // Initialize the logger
  //~ std::string const filename = "dummy_client_logs.txt";
  //~ common::FileLogger::init(filename);
  //~ LOGFILE << "Initialized the logger";
  common::ConsoleLogger::init();

  while(true)
  {
    try
    {
      // Initialize the client
      network::Client client("localhost", 30000);
      network::ClientRequest::UniquePtr request_ptr = nullptr;
      network::ServerResponse::UniquePtr response_ptr = nullptr;
      CONSOLE << "Initialized the client";
      
      // Send messages to the server
      int request_idx = 0;
      int constexpr max_num_requests = 10;
      std::shared_ptr<common::MarkerIdToEstimate> markers = nullptr;
      while(true)
      {    
        // Build and send the request to the server
        std::string request_str;
        network::MessageType message_type;
        if(request_idx>0)
        {
          message_type = network::MessageType::GET_MEASUREMENTS;
          request_ptr.reset(new network::ClientRequest(message_type));
        }
        else
        {
          message_type = network::MessageType::INITIALIZATION;          
          std::shared_ptr<common::Team> team_ptr(new common::Team);
          *team_ptr = common::Team::PURPLE;
          request_ptr.reset(new network::ClientRequest(message_type, team_ptr));
        }
        CONSOLE << "Message pointer initialized";
        request_ptr->serialize(&request_str);
        CONSOLE << "Message serialized";
        client << request_str;
        CONSOLE << "Sent request to the server";

        // Get the response from the server
        std::string response_str;
        client >> response_str;
        markers.reset(new common::MarkerIdToEstimate);
        response_ptr.reset(new network::ServerResponse(message_type, markers));
        response_ptr->deserialize(response_str);
        CONSOLE << "Received and deserialized response from the server";
        
        // Prepare for next iteration
        request_idx += 1;
        if(request_idx >= max_num_requests) break;
        std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
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
    }
    catch(network::SocketException const& e)
    {
      CONSOLE << "Client lost the connection with the server";
      CONSOLE << e.description();
    }
  }
}
