/* (c) https://github.com/MontiCore/monticore */

#ifndef CallbackB_hpp
#define CallbackB_hpp

#include "tests_a_compA.h"
#include "mqtt/client.h"
using namespace std;
using namespace mqtt;

class CallbackB : public virtual callback
{
    client& cli_;

public:
    CallbackB(client& cli, bool* port);

    void connected(const string& cause);

    void connection_lost(const string& cause);

    void message_arrived(const_message_ptr msg);

private:
    bool* port_;

};
#endif /* CallbackB_hpp */
