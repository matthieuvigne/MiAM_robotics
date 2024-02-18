// C++ program to show the example of server application in 
// socket programming 
#include <cstring> 
#include <iostream> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <unistd.h> 
#include <miam_utils/trajectory/Trajectory.h>
#include <miam_utils/trajectory/SampledTrajectory.h>
  
using namespace std; 
using namespace miam::trajectory; 

#define SIZE_OF_BUFFER 512

enum MessageType
{
    NEW_TRAJECTORY = 0,
    SET_ID = 1,
    ERROR = 99
};
  
int main() 
{ 
    // creating socket 
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0); 
  
    // specifying the address 
    sockaddr_in serverAddress; 
    serverAddress.sin_family = AF_INET; 
    serverAddress.sin_port = htons(778); 
    serverAddress.sin_addr.s_addr = INADDR_ANY; 
  
    // binding socket. 
    bind(serverSocket, (struct sockaddr*)&serverAddress, 
         sizeof(serverAddress)); 
  
    // listening to the assigned socket 
    listen(serverSocket, 5); 
  
    // accepting connection request 
    int clientSocket 
        = accept(serverSocket, nullptr, nullptr); 
  
    // recieving data 
    std::vector<float > tmpvec;
    float* buffer = new float[SIZE_OF_BUFFER](); 

    int sizeofreceiveddata;

    while((sizeofreceiveddata = recv(clientSocket, buffer, SIZE_OF_BUFFER*4, 0)) > 0)
    {
        cout << "Receiving: " << sizeofreceiveddata << std::endl; 
        // cout << "Message from client: " << buffer 
        //         << endl; 
        for (int i = 0; i < SIZE_OF_BUFFER; i++)
        {
            float f = buffer[i];
            // cout << f << endl;
            tmpvec.push_back(f);
        }
    }

    // for (int i=0; i < tmpvec.size(); i++)
    // {
    //     std::cout << tmpvec.at(i);
    //     if (i > 0 && i % 5 == 0)
    //         std::cout << std::endl;
    // }
    
    MessageType mt = MessageType::ERROR;
    if (tmpvec.at(0) == 0)
    {
        mt = MessageType::NEW_TRAJECTORY;
    }
    cout << "MessageType : " << mt << endl;

    if (mt == MessageType::NEW_TRAJECTORY)
    {
        int size_of_trajectory = tmpvec.at(1);
        float duration_of_trajectory = tmpvec.at(2);

        cout << "Size of trajectory: " << size_of_trajectory << endl;
        cout << "Duration of trajectory: " << duration_of_trajectory << endl;

        std::vector<TrajectoryPoint > trajectoryPoints;
        // int serializationIndex = 2;
        for (int i = 0; i < size_of_trajectory; i++)
        {
            TrajectoryPoint tp;
            tp.position.x = tmpvec.at(3 + 5*i);
            tp.position.y = tmpvec.at(3 + 5*i + 1);
            tp.position.theta = tmpvec.at(3 + 5*i + 2);
            tp.linearVelocity = tmpvec.at(3 + 5*i + 3);
            tp.angularVelocity = tmpvec.at(3 + 5*i + 4);
            trajectoryPoints.push_back(tp);
        }

        for (auto& tp : trajectoryPoints)
        {
            std::cout << tp << std::endl;
        }
    }

  
    // closing the socket. 
    close(serverSocket); 
  
    return 0; 
}