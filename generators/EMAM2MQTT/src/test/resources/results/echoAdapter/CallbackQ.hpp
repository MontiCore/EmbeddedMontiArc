/* (c) https://github.com/MontiCore/monticore */

#ifndef CallbackQ_hpp
#define CallbackQ_hpp

#include "tests_a_compA.h"
#include "mqtt/client.h"
using namespace std;
using namespace mqtt;

class CallbackQ : public virtual callback
{
    client& cli_;

public:
    CallbackQ(client& cli, double* port);

    void connected(const string& cause);

    void connection_lost(const string& cause);

    void message_arrived(const_message_ptr msg);

private:
    double* port_;

};
#endif /* CallbackQ_hpp */
