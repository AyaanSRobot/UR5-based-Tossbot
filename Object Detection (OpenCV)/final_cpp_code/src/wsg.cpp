#include "wsg.h"

WSG::WSG(int port, string ip)
{
    c.setUpSocket(port, ip);

    c.sendMessage("home()\n");
    c.recieveMessage();
    c.recieveMessage();
}

void WSG::grip()
{
    c.sendMessage("grip()\n");
    c.recieveMessage();
    c.recieveMessage();
}

void WSG::release()
{
    c.sendMessage("release(10)\n");

}

void WSG::home()
{
    c.sendMessage("home()\n");
    c.recieveMessage();
    c.recieveMessage();
}

void WSG::bye()
{
    c.sendMessage("bye()\n");
    c.recieveMessage();
    c.closesocket();
}
