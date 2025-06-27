/* (c) https://github.com/MontiCore/monticore */
#ifndef CallbackQ_hpp
#define CallbackQ_hpp

#include "tests_a_compA.h"
#include "mqtt/client.h"

using namespace std;

class CallbackQ : public virtual mqtt::callback
{
    mqtt::client& cli_;

public:
    CallbackQ(mqtt::client& cli, double* port);

    void connected(const std::string& cause);

    void connection_lost(const std::string& cause);

    void message_arrived(mqtt::const_message_ptr msg);

private:
    double* port_;

};

#endif /* CallbackQ_hpp */
