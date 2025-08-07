/* (c) https://github.com/MontiCore/monticore */
#ifndef CallbackZ_hpp
#define CallbackZ_hpp

#include "tests_a_compA.h"
#include "mqtt/client.h"

using namespace std;

class CallbackZ : public virtual mqtt::callback
{
    mqtt::client& cli_;

public:
    CallbackZ(mqtt::client& cli, int* port);

    void connected(const std::string& cause);

    void connection_lost(const std::string& cause);

    void message_arrived(mqtt::const_message_ptr msg);

private:
    int* port_;

};

#endif /* CallbackZ_hpp */
