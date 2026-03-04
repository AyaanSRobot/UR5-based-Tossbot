#ifndef CLIENT_H
#define CLIENT_H
#include <string.h>
#include <string>
#include <netdb.h>
#include <arpa/inet.h>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>

using namespace std;

class client
{
public:
    client();

    void setUpSocket(int port, string ip);

    client(int port, string ip);

    void sendMessage(string message);

    string recieveMessage();

    void closesocket();

private:
char buf[4096];
int sock;

};

#endif // CLIENT_H
