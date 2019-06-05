//  MqttAdapter
//
//  Created by Georg Vinogradov on 05.06.19.
//  Copyright © 2019 Georg Vinogradov. All rights reserved.
//

#pragma once
#include "tests_a_compA.h"
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
    mqtt::client* _clockSubscriber = nullptr;
    mqtt::client* _echoPublisher = nullptr;
};

class callback : public virtual mqtt::callback
{
    mqtt::client& cli_;
    
    // Callback for when connected
    void connected(const std::string& cause) {}
    
    // Callback for when the connection is lost.
    void connection_lost(const std::string& cause) {
        cout << "\nConnection lost";
        if (!cause.empty())
            cout << ": " << cause << endl;
    }
    
    // Callback for when message is received
    void message_arrived(mqtt::const_message_ptr msg) {
        cout << "Message received "<< msg->get_topic() << ": " << msg->get_payload_str() << endl;
    }
    
public:
    callback(mqtt::client& cli) : cli_(cli) {}
};
