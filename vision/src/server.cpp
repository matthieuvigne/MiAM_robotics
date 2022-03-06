#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>

#include <network/server.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructors and destructors
//--------------------------------------------------------------------------------------------------

Server::Server()
{
  // TODO
}

//--------------------------------------------------------------------------------------------------

Server::~Server()
{
  // TODO
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void Server::serverThread()
{
  // Launch the server

	// Creating socket file descriptor
  int server_fd = socket(AF_INET, SOCK_STREAM, 0);
  assert( server_fd != 0);
	// Forcefully attaching socket to the port 8080
  int opt = 1;
  int const PORT = 8080;
  bool success = setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
  assert(success);
  sockaddr_in address;
	int addrlen = sizeof(address);
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(PORT);  
  // Forcefully attaching socket to the port 8080
  success &= bind(server_fd, (sockaddr*) &address, sizeof(address));
  assert(success);
  // Listen to the port binded to the socket
  success &= listen(server_fd, 3);
  assert(success);
  // Empty the current messages
  while(!this->server_buffer_.empty())
    this->server_buffer_.pop();

  // Accept the first communication to get the socket from the robot
  // TODO

  // Send the messages
  while(true)
  {
    // Wait condition (unbusy waiting)
    std::unique_lock<std::mutex> server_locker(this->server_mtx_);
    this->server_con_.wait(server_locker,
      [&](){ return !this->server_buffer_.empty() or this->shut_down_; });
    if(this->shut_down_) break;
    Message new_message = std::move(this->server_buffer_.front());
    this->server_buffer_.pop();
    this->server_mtx_.unlock();
  
    // Send the message
    //~ send(new_socket , hello , strlen(hello) , 0 );
    // TODO
  }
  
  // Shut the server down
  // TODO
}

  //~ std::cout << "running....\n";
  //~ int count = 0;

  //~ try
  //~ {
    //~ // Create the socket
    //~ ServerSocket server ( 30000 );
    //~ std::cout << "[simple_server] Created the server socket.\n";
    //~ while ( true )
    //~ {
      //~ ServerSocket new_sock;
      //~ std::cout << "[simple_server] Created the new_sock.\n";
      //~ server.accept ( new_sock ); // Creates a waiting point
      //~ std::cout << "[simple_server] Accepted the new_sock.\n";
      //~ try
      //~ {
        //~ while ( true )
        //~ {
          //~ std::string data;
          //~ std::cout << "[simple_server] Waiting for the client's message.\n";
          //~ new_sock >> data;
          //~ std::cout << "[simple_server] We received the following message from the client " << data << std::endl;
          //~ new_sock << data + std::to_string(count);
          //~ if(++count>5)
          //~ {
            //~ std::cout << "More than 5 connections" << std::endl;
            //~ break;
          //~ }
        //~ }
        //~ break;
      //~ }
      //~ catch ( SocketException& e)
      //~ {
        //~ std::cout << "Exception was caught: " << e.description() << "\n";
      //~ }
    //~ }
  //~ }

  //~ catch ( SocketException& e )
  //~ {
    //~ std::cout << "Exception was caught: " << e.description() << "\nExiting.\n";
  //~ }

  //~ return 0;

//--------------------------------------------------------------------------------------------------

} // namespace network
