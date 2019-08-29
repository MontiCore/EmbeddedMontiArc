/* (c) https://github.com/MontiCore/monticore */

#ifndef CallbackZ_hpp
#define CallbackZ_hpp

#include "tests_a_compA"
#include "mqtt/client.h"
using namespace std;
using namespace mqtt;

class CallbackZ : public virtual callback
{
    client& cli_;

public:
    CallbackZ(client& cli, double* port);

    void connected(const string& cause);

    void connection_lost(const string& cause);

    void message_arrived(const_message_ptr msg);

private:
    int* port_;

};
#endif /* CallbackZ_hpp */
