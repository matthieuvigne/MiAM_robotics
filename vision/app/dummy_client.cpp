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
  common::ConsoleLogger::init();

  while(true)
  {
    try
    {
      // Initialize the client
      network::Client client("localhost", 30000);
      network::ClientRequest::UniquePtr request_ptr = nullptr;
      network::ServerResponse::UniquePtr response_ptr = nullptr;
      
      // Send messages to the server
      int request_idx = 0;
      int constexpr max_num_requests = 50;

      while(true)
      {
        // Build the message to send to the server
        network::MessageType message_type;
        if(request_idx>0)
        {
          message_type = network::MessageType::GET_MEASUREMENTS;
          request_ptr.reset(new network::ClientRequest(message_type));
        }
        else
        {
          message_type = network::MessageType::INITIALIZATION;
          request_ptr.reset(new network::ClientRequest(message_type));
          request_ptr->getParamsAs<common::Team>() = common::Team::PURPLE;
        }

        // Serialize and send the message to the server
        std::string request_str;
        request_ptr->serialize(&request_str);
        client << request_str;

        // Get the response from the server
        std::string response_str;
        client >> response_str;
        response_ptr.reset(new network::ServerResponse(response_str));
        response_ptr->deserialize(response_str);
        CONSOLE << "Received message type: " << network::print(message_type);

        if(message_type == network::MessageType::GET_MEASUREMENTS)
        {
          common::MarkerList const& markers =
            response_ptr->getParamsAs<common::MarkerList>();
          CONSOLE << "Received " << markers.size() << " markers";
        }

        // Prepare for next iteration
        request_idx += 1;
        if(request_idx >= max_num_requests) break;
        std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
        request_ptr = nullptr;
        response_ptr = nullptr;
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
