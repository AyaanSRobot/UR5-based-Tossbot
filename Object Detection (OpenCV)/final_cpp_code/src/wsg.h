#ifndef WSG_H
#define WSG_H
#include "client.h"


class WSG
{
public:
    WSG(int port, string ip);

    void grip();

    void release();

    void home();

    void bye();

private:
client c;
};

#endif // WSG_H
