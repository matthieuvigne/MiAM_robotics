// Client side C/C++ program to demonstrate Socket programming
#include <iostream>

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

#define PORT 8080

int main(int argc, char const *argv[])
{
  // Create the socket for the client
  int sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0)
	{
		printf("\n Socket creation error \n");
		return -1;
	}

  // Get the server adress
  sockaddr_in serv_addr;
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);
	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0)
	{
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}
  std::cout << "Get the server address" << std::endl;
  std::cout << "address.sin_family: " << serv_addr.sin_family << std::endl;
  std::cout << "address.sin_addr.s_addr: " << serv_addr.sin_addr.s_addr << std::endl;
  std::cout << "address.sin_port: " << serv_addr.sin_port << std::endl;

  // Connect to the server
	if (connect(sock, (sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		printf("\nConnection Failed \n");
		return -1;
	}
  
  // Send the message to the server
	char *hello = "Hello from client";
  send(sock , hello , strlen(hello) , 0 );
	printf("Hello message sent\n");
  
  // Read the message from the server
	char buffer[1024] = {0};
  int valread = read( sock , buffer, 1024);
	printf("%s\n",buffer );

	return 0;
}
