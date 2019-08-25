/* (c) https://github.com/MontiCore/monticore */

#pragma once

#include "tests_a_compA.h"
#include "IAdapter_tests_a_compA.h"
#include "mqtt/client.h"
#include "Callback.hpp"

using namespace std;
using namespace mqtt;

class MqttAdapter_tests_a_compA : public IAdapter_tests_a_compA {

public:

    MqttAdapter_tests_a_compA();

    void init(tests_a_compA* comp);

    void publish_echo_portC();
    void publish_echo_portD();


    void tick();

private:
    const string SERVER_ADDRESS = "tcp://localhost:1883";

    tests_a_compA* component = nullptr;

    // Callbacks, subscribers
    Callback* _callback_portA = nullptr;
    client* _sub_portA = nullptr;
    Callback* _callback_portB = nullptr;
    client* _sub_portB = nullptr;
    // Publishers
    client* _pub_portC = nullptr;
    client* _pub_portD = nullptr;
};
