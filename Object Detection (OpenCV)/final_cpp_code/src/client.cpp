#include "client.h"


using namespace std;

client::client()
{

}

void client::setUpSocket(int port, string ip)
{
    //Create a socket
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == -1)
    {
        cerr << "Can't create a socket!" << endl;
    }
    else
    {
        cout << "Socket has been created" << endl;
    }

    //	Create a hint structure for the server we're connecting with

    sockaddr_in hint;
    hint.sin_family = AF_INET;
    hint.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &hint.sin_addr);

    //	Connect to the server on the socket
    int connectRes = connect(sock, (sockaddr*)&hint, sizeof(hint));
    if (connectRes == -1)
    {
       cerr << "Can't connect to server" << endl;
    }
    else
    {
        cout << "Connected to server"    << endl;
    }
}

client::client(int port, string ip)
{
    setUpSocket(port, ip);
}

void client::sendMessage(string message)
{

    cout << "Sending " << message << endl;

    int sendRes = send(sock, message.c_str(), message.size(), 0);
    if (sendRes == -1)
    {
        cout << "Could not send to server! Whoops!\r\n" << endl;
    }
}

string client::recieveMessage()
{
    //clear the buffer
    memset(buf, 0, 4096);

    //Wait for response
    int bytesReceived = recv(sock, buf, 4096, 0);
    if (bytesReceived == -1)
    {
        cout << "There was an error getting response from server\r\n" <<endl;
        return "No message received";
    }
    cout << string(buf, bytesReceived) << endl;

    return string(buf, bytesReceived);
}

void client::closesocket()
{
    close(sock);
}
