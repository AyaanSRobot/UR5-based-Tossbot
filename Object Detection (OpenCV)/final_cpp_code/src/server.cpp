#include "server.h"
#include <netdb.h>
#include <arpa/inet.h>
#include <iostream> // For cout
#include <unistd.h>

using namespace std;

Server::Server(int port, string addr)
{
    //Create a socket
    listening = socket(AF_INET, SOCK_STREAM, 0);
    if(listening == -1)
    {
        cerr << "Can't create a socket!" << endl;
    }
    else
    {
        cout << "Socket has been created" << endl;
    }

    //Bind the socket to a IP / port
    sockaddr_in saddr;
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(port);
    inet_pton(AF_INET, addr.c_str(), &saddr.sin_addr);

    if(bind(listening, (sockaddr*)&saddr, sizeof(saddr)) == -1)
    {
        cerr << "Can't bind to IP/port " << endl;
    }
    else
    {
        cout << "The socket has been bound to a IP / port" << endl;
        cout << "Now waiting for client connection" << endl;
    }

    //Mark the socket for listening
    if(listen(listening, SOMAXCONN) == -1)
    {
        cerr << "Can't listen!" << endl;
    }

    //Accept a call
    sockaddr_in client;
    socklen_t clientSize = sizeof(client);
    char host[NI_MAXHOST];
    char svc[NI_MAXSERV];

    clientSocket = accept(listening, (sockaddr*)&client, &clientSize);

    if(clientSocket == -1)
    {
        cerr << "Problem with client connecting!" << endl;
    }

    //Close the listening socket
    close(listening);

    memset(host, 0, NI_MAXHOST);
    memset(svc, 0, NI_MAXSERV);

    int result = getnameinfo((sockaddr*)&client, sizeof(client), host, NI_MAXHOST, svc, NI_MAXSERV, 0);

    if(result)
    {
        std::cout << host << " connected on " << svc << endl;
    }
    else
    {
        inet_ntop(AF_INET, &client.sin_addr,host,NI_MAXHOST);
        std::cout << host << " connected on " << ntohs(client.sin_port) << endl;
    }
}

void Server::sendMessage(string message)
{
    //clear the buffer
    memset(buf, 0, 4096);

    //Send message
    char const *testMsg = message.c_str();
    send(clientSocket, testMsg, strlen(testMsg), 0);
}

string Server::waitForMessage()
{
    while(true)
    {
        //clear the buffer
        memset(buf, 0, 4096);

        cout << "Waiting for message" << endl;

        //wait for message
        int bytesRecv = recv(clientSocket, buf, 4096, 0);
        if(bytesRecv == -1)
        {
            cerr << "There was a connection issuse" << endl;
        }
        else if(bytesRecv == 0)
        {
            std::cout << "The client disconnected" << endl;
        }

        //display message
        std::cout << "Received: " << string(buf, 0, bytesRecv) << endl;
        return string(buf, 0, bytesRecv);
    }

}

void Server::closeSocket()
{
    //Close socket
    close(clientSocket);
}
