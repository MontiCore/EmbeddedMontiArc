/* (c) https://github.com/MontiCore/monticore */
#ifndef CallbackN_hpp
#define CallbackN_hpp

#include "tests_a_compA.h"
#include "mqtt/client.h"

using namespace std;

class CallbackN : public virtual mqtt::callback
{
    mqtt::client& cli_;

public:
    CallbackN(mqtt::client& cli, int* port);

    void connected(const std::string& cause);

    void connection_lost(const std::string& cause);

    void message_arrived(mqtt::const_message_ptr msg);

private:
    int* port_;

};

#endif /* CallbackN_hpp */
