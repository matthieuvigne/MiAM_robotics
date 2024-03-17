#include <cstring> 
#include <iostream> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <unistd.h> 
#include <arpa/inet.h>

#include <MessageSender.h>

namespace message_sender
{
    void send_message(float* message, int message_size_in_float_number, const char* str_ip_addr)
    {
        // creating socket 
        int clientSocket = socket(AF_INET, SOCK_STREAM, 0); 

        // specifying address 
        sockaddr_in serverAddress; 
        serverAddress.sin_family = AF_INET; 
        serverAddress.sin_port = htons(778); 
        serverAddress.sin_addr.s_addr = inet_addr(str_ip_addr);
    
        // sending connection request 
        std::cout << "Connect: " << connect(clientSocket, (struct sockaddr*)&serverAddress, 
                sizeof(serverAddress)) << std::endl; 
    
        // sending data 
        std::cout << "Send: " << send(clientSocket, message, message_size_in_float_number*4, 0) << std::endl; 
    
        // closing socket 
        close(clientSocket); 
    }

    void send_message_udp(float* message, int message_size_in_float_number, const char* str_ip_addr)
    {
        // creating socket 
        int clientSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); 
        int broadcast=1;

        setsockopt(clientSocket, SOL_SOCKET, SO_BROADCAST,
                    &broadcast, sizeof broadcast);

        // specifying address 
        sockaddr_in serverAddress; 
        serverAddress.sin_family = AF_INET; 
        serverAddress.sin_port = htons(779); 
        serverAddress.sin_addr.s_addr = inet_addr(str_ip_addr);
    
        // sending connection request 
        std::cout << "Connect: " << connect(clientSocket, (struct sockaddr*)&serverAddress, 
                sizeof(serverAddress)) << std::endl; 
    
        // sending data 
        std::cout << "Send: " << send(clientSocket, message, message_size_in_float_number*4, 0) << std::endl; 
    
        // closing socket 
        close(clientSocket); 
    }
}