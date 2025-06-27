/* (c) https://github.com/MontiCore/monticore */
#ifndef CallbackB_hpp
#define CallbackB_hpp

#include "tests_a_compA.h"
#include "mqtt/client.h"

using namespace std;

class CallbackB : public virtual mqtt::callback
{
    mqtt::client& cli_;

public:
    CallbackB(mqtt::client& cli, bool* port);

    void connected(const std::string& cause);

    void connection_lost(const std::string& cause);

    void message_arrived(mqtt::const_message_ptr msg);

private:
    bool* port_;

};

#endif /* CallbackB_hpp */
