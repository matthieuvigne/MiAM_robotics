#include <network/client.hpp>

namespace network {

//--------------------------------------------------------------------------------------------------
// Constructors and destructors
//--------------------------------------------------------------------------------------------------

Client::Client(
    std::mutex* mtx_ptr,
    std::condition_variable* con_ptr,
    std::queue<Message>* buffer_ptr)
: client_mtx_       (mtx_ptr),
  client_con_       (con_ptr),
  client_buffer_    (buffer_ptr)
{
  // TODO
}

//--------------------------------------------------------------------------------------------------

Client::~Client()
{
  // TODO
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

void Client::clientThread()
{
  
}


//~ int main ( int argc, char* argv[] )
//~ {
  //~ try
  //~ {
    //~ ClientSocket client_socket ( "localhost", 30000 );
    //~ std::cout << "[simple_client] Initialized the client\n";
    
    //~ std::string reply;
    //~ try
    //~ {
      //~ for(int i=0; i<2; ++i)
      //~ {
        //~ std::cout << "We send a 'Test message [" << i << "]' to the server" << std::endl;
        //~ client_socket << "Test message.";
        //~ client_socket >> reply;
        //~ std::cout << "We received this response from the server:\n\"" << reply << "\"\n";
      //~ }
    //~ }
    //~ catch (SocketException& e)
    //~ {
      //~ std::cout << "Exception was caught: " << e.description() << "\n";
    //~ }
  //~ }
  //~ catch (SocketException& e )
  //~ {
    //~ std::cout << "Exception was caught: " << e.description() << "\n";
  //~ }
  //~ return 0;
//~ }

//--------------------------------------------------------------------------------------------------

} // namespace network