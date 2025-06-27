/* (c) https://github.com/MontiCore/monticore */

#ifndef CallbackN_hpp
#define CallbackN_hpp

#include "tests_a_compA.h"
#include "mqtt/client.h"
using namespace std;
using namespace mqtt;

class CallbackN : public virtual callback
{
    client& cli_;

public:
    CallbackN(client& cli, int* port);

    void connected(const string& cause);

    void connection_lost(const string& cause);

    void message_arrived(const_message_ptr msg);

private:
    int* port_;

};
#endif /* CallbackN_hpp */
