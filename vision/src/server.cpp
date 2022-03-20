#include <iostream>

#include <network/server.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructors and destructors
//--------------------------------------------------------------------------------------------------

Server::Server(int port)
: ServerSocket(port)
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
  // Wait for the client's connection
  // TODO

  // Wait for the client's requests
  while(true)
  {
    // Wait condition (unbusy waiting)
    std::unique_lock<std::mutex> server_locker(this->thread_mtx_);
    this->thread_con_.wait(server_locker,
      [&](){ return !this->server_buffer_.empty() or this->abort_thread_; });
    if(this->abort_thread_) break;
    Message new_message = std::move(this->server_buffer_.front());
    this->server_buffer_.pop();
    this->thread_mtx_.unlock();
  
    // Send the message
    //~ send(new_socket , hello , strlen(hello) , 0 );
    // TODO
  }
  
  // Shut the server down
  std::cout << "Shutting down the server thread." << std::endl;
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
