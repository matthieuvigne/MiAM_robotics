// Server side C/C++ program to demonstrate Socket programming
#include <iostream>

#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>

#define PORT 8080

int main(int argc, char const *argv[])
{	
	// Creating socket file descriptor
  int server_fd;
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
	{
		perror("socket failed");
		exit(EXIT_FAILURE);
	}
  std::cout << "Server handle: " << server_fd << std::endl;
  std::cout << "AF_INET: " << AF_INET << std::endl;
  std::cout << "SOCK_STREAM: " << SOCK_STREAM << std::endl;
	
	// Forcefully attaching socket to the port 8080
  int opt = 1;
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
	{
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}
	/*struct sockaddr_in address; (C style struct declaration)*/ 
  sockaddr_in address;
	int addrlen = sizeof(address);
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(PORT);
  std::cout << "Server address" << std::endl;
  std::cout << "address.sin_family: " << address.sin_family << std::endl;
  std::cout << "address.sin_addr.s_addr: " << address.sin_addr.s_addr << std::endl;
  std::cout << "address.sin_port: " << address.sin_port << std::endl;
	
	// Forcefully attaching socket to the port 8080
  // sockaddr est une structure plus générale décrivant une adresse réseau
  // Elle est moins parlante que sockaddr_in mais des fonctions l'utilisent donc besoin d'un cast
  // Les deux fonctions bind et connect associent le descripteur de fichier de socket à une adresse
  // réseau (combinaison IP/port). Leurs signatures sont les suivantes :
  // - int connect(int sockfd, sockaddr const* addr, socklen_t addrlen);
  // - int bind(int sockfd, sockaddr const* addr, socklen_t addrlen);
  // Bind lie le serveur à son adresse locale afin que les clients puissent utiliser cette adresse
  // pour se connecter au serveur avec la fonction connect().
	if( bind(server_fd, (sockaddr*) &address, sizeof(address)) < 0 )
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
  
  // On écoute maintenant le port associé (binded) à la socket
	if (listen(server_fd, 3) < 0)
	{
		perror("listen");
		exit(EXIT_FAILURE);
	}
  
  // On récupère l'identifiant de la socket de laquelle on reçoit un message
	int new_socket;
  new_socket = accept(server_fd, (sockaddr *)&address, (socklen_t*)&addrlen);
  if (new_socket < 0)
	{
		perror("accept");
		exit(EXIT_FAILURE);
	}

  // Display the address of the incoming message
  std::cout << "Incoming message" << std::endl;
  std::cout << "address.sin_family: " << address.sin_family << std::endl;
  std::cout << "address.sin_addr.s_addr: " << address.sin_addr.s_addr << std::endl;
  std::cout << "address.sin_port: " << address.sin_port << std::endl;

  // Read the incoming message
  char buffer[1024] = {0};
	int valread = read( new_socket , buffer, 1024);
	printf("%s\n",buffer );
  
  // Send a response message
	char *hello = "Hello from server";
	send(new_socket , hello , strlen(hello) , 0 );
	printf("Hello message sent\n");
	return 0;
}
