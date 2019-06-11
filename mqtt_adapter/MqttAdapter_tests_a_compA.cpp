//  MqttAdapter
//
//  Created by Georg Vinogradov on 05.06.19.
//  Copyright © 2019 Georg Vinogradov. All rights reserved.
//

#include "MqttAdapter_tests_a_compA.h"

MqttAdapter_tests_a_compA::MqttAdapter_tests_a_compA() {}

void MqttAdapter_tests_a_compA::init(tests_a_compA *comp)
{
    this->component = comp;
    
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);

    _clockSubscriber = new mqtt::client(SERVER_ADDRESS, SUB_ID);
    _echoPublisher = new mqtt::client(SERVER_ADDRESS, PUB_ID);
    
    try {
        _clockSubscriber->connect(connOpts);
        callback cb(*_clockSubscriber);
        _clockSubscriber->set_callback(cb);
        _clockSubscriber->subscribe(TOPIC, 1);
        
    } catch (const mqtt::exception& exc) {
        cerr << exc.what() << endl;
    }
    
}

void MqttAdapter_tests_a_compA::publish_echoPublisher()
{
    mqtt::message tmpMsg;
    
    double value = component->rosOut;
    uint8_t *buf = (uint8_t*)&value;
    
    tmpMsg.set_payload(buf, 8); // pointer to byte buffer of length 8
    
    try {
        _echoPublisher->publish(tmpMsg);
        
    } catch (const mqtt::exception& exc) {
        cerr << exc.what() << endl;
    }
}

void MqttAdapter_tests_a_compA::tick()
{
    publish_echoPublisher();
}

