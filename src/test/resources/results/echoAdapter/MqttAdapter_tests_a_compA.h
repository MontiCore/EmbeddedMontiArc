
#pragma once

#include "tests_a_compA.h"
#include "mqtt/client.h"
#include "Callback.hpp"

using namespace std;
using namespace mqtt;

class MqttAdapter_tests_a_compA {
    
public:
    
    MqttAdapter_tests_a_compA();
    
    void init(tests_a_compA* comp);
    
        void publish_echo_portC();
    
    
    void tick();
    
private:
    const string SERVER_ADDRESS = "tcp://localhost:1883";
    const string PUB_ID = "publisher_cpp";
    const string SUB_ID = "subscriber_cpp";
    const string TOPIC = "/clock";
    
    tests_a_compA* component = nullptr;
    
    // Callbacks, subscribers
        Callback* _callback_portA = nullptr;
        client* _sub_portA = nullptr; 
    // Publishers
        client* _pub_portC = nullptr; 
};
