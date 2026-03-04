#ifndef SERVER_H
#define SERVER_H
#include <string.h>
#include <string>

using namespace std;

class Server
{
public:
    Server(int port, string addr);

    void sendMessage(string message);

    string waitForMessage();

    void closeSocket();

private:
    int clientSocket;
    char buf[4096];
    int listening;
};

#endif // SERVER_H
