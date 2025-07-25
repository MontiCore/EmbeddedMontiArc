/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "tests_a_compA.h"
#include "CallbackQ.hpp"
#include "CallbackN.hpp"
#include "CallbackZ.hpp"
#include "CallbackB.hpp"
#include "mqtt/client.h"
#include <iostream>

using namespace std;

class MqttAdapter_tests_a_compA {

public:

    MqttAdapter_tests_a_compA();

    void init(tests_a_compA* comp);

    void publish_echoPublisherQ();
    void publish_echoPublisherN();
    void publish_echoPublisherZ();
    void publish_echoPublisherB();

    void tick();
    bool hasReceivedNewData();

private:
    const string SERVER_ADDRESS = "tcp://localhost:1883";
    const string PUB_IDQ = "publisher_cppQ";
    const string SUB_IDQ = "subscriber_cppQ";
    const string PUB_IDN = "publisher_cppN";
    const string SUB_IDN = "subscriber_cppN";
    const string PUB_IDZ = "publisher_cppZ";
    const string SUB_IDZ = "subscriber_cppZ";
    const string PUB_IDB = "publisher_cppB";
    const string SUB_IDB = "subscriber_cppB";
    const string TOPICQ = "/clock";
    const string TOPICN = "/clockN";
    const string TOPICZ = "/clockZ";
    const string TOPICB = "/clockB";

    tests_a_compA* component = nullptr;
    CallbackQ* _callbackQ = nullptr;
    CallbackN* _callbackN = nullptr;
    CallbackZ* _callbackZ = nullptr;
    CallbackB* _callbackB = nullptr;
    mqtt::client* _clockSubscriberQ = nullptr;
    mqtt::client* _echoPublisherQ = nullptr;
    mqtt::client* _clockSubscriberN = nullptr;
    mqtt::client* _echoPublisherN = nullptr;
    mqtt::client* _clockSubscriberZ = nullptr;
    mqtt::client* _echoPublisherZ = nullptr;
    mqtt::client* _clockSubscriberB = nullptr;
    mqtt::client* _echoPublisherB = nullptr;
};
