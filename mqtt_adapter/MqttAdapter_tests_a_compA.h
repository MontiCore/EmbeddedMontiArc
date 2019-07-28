#pragma once
#include "tests_a_compA.h"
#include "Callback.hpp"
#include "mqtt/client.h"
#include <iostream>

using namespace std;

class MqttAdapter_tests_a_compA {
    
public:
    
    MqttAdapter_tests_a_compA();
    
    void init(tests_a_compA* comp);
    
    void publish_echoPublisher();
    
    void tick();
    
private:
    const string SERVER_ADDRESS = "tcp://localhost:1883";
    const string PUB_ID = "publisher_cpp";
    const string SUB_ID = "subscriber_cpp";
    const string TOPIC = "/clock";
    
    tests_a_compA* component = nullptr;
    Callback* _callback = nullptr;
    mqtt::client* _clockSubscriber = nullptr;
    mqtt::client* _echoPublisher = nullptr;
};
