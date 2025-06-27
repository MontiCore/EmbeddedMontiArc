/* (c) https://github.com/MontiCore/monticore */

#pragma once

#include "tests_a_compA.h"
#include "IAdapter_tests_a_compA.h"
#include "mqtt/client.h"
#include "CallbackQ.hpp"
#include "CallbackN.hpp"
#include "CallbackZ.hpp"
#include "CallbackB.hpp"

using namespace std;
using namespace mqtt;

class MqttAdapter_tests_a_compA : public IAdapter_tests_a_compA {

public:

    MqttAdapter_tests_a_compA();

    void init(tests_a_compA* comp);

    void publish_echo_portC();
    void publish_echo_portD();
    void publish_echo_portF();
    void publish_echo_portH();
    void publish_echo_portJ();

    void tick();
    bool hasReceivedNewData();  

private:
    const string SERVER_ADDRESS = "tcp://localhost:1883";

    tests_a_compA* component = nullptr;

    // Callbacks, subscribers
          CallbackQ* _callback_portA = nullptr;
    client* _sub_portA = nullptr;
          CallbackQ* _callback_portB = nullptr;
    client* _sub_portB = nullptr;
          CallbackN* _callback_portE = nullptr;
    client* _sub_portE = nullptr;
          CallbackZ* _callback_portG = nullptr;
    client* _sub_portG = nullptr;
          CallbackB* _callback_portI = nullptr;
    client* _sub_portI = nullptr;
    // Publishers
    client* _pub_portC = nullptr;
    client* _pub_portD = nullptr;
    client* _pub_portF = nullptr;
    client* _pub_portH = nullptr;
    client* _pub_portJ = nullptr;
};
