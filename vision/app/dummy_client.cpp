#include <iostream>

#include <network/client.hpp>

// Global variables

// Dummy client main routine
int main(int argc, char* argv[])
{
  // Initialize the client
  std::cout << "Launching the dummy client" << std::endl;
  network::Client client("localhost", 30000);
  
  // Send messages to the client
  while(true)
  {
    // Write the request
    std::string request;
    std::cout << "Write the request to the server:" << std::endl;
    std::cin >> request;
    client << request;
    
    // Get the answer from the server
    if(request == "stop") break;
    std::string answer;
    client >> answer;
    std::cout << "Received response from the server: " << answer << std::endl;
  }
}
