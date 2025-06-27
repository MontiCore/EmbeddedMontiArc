/* (c) https://github.com/MontiCore/monticore */
#include "MqttAdapter_tests_a_compA.h"

using namespace std;

MqttAdapter_tests_a_compA::MqttAdapter_tests_a_compA()
{

}

void MqttAdapter_tests_a_compA::init(tests_a_compA *comp)
{
    // Initialize component
    this->component = comp;

    // Initialize connect options
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);

    // Intitialize callback, subscriber and publisher
    _clockSubscriberQ = new mqtt::client(SERVER_ADDRESS, SUB_IDQ);
    _callbackQ = new CallbackQ(*_clockSubscriberQ, &(comp->mqttInQ));
    _echoPublisherQ = new mqtt::client(SERVER_ADDRESS, PUB_IDQ);
    _clockSubscriberN = new mqtt::client(SERVER_ADDRESS, SUB_IDN);
    _callbackN = new CallbackN(*_clockSubscriberN, &(comp->mqttInN));
    _echoPublisherN = new mqtt::client(SERVER_ADDRESS, PUB_IDN);
    _clockSubscriberZ = new mqtt::client(SERVER_ADDRESS, SUB_IDZ);
    _callbackZ = new CallbackZ(*_clockSubscriberZ, &(comp->mqttInZ));
    _echoPublisherZ = new mqtt::client(SERVER_ADDRESS, PUB_IDZ);
    _clockSubscriberB = new mqtt::client(SERVER_ADDRESS, SUB_IDB);
    _callbackB = new CallbackB(*_clockSubscriberB, &(comp->mqttInB));
    _echoPublisherB = new mqtt::client(SERVER_ADDRESS, PUB_IDB);

    // Connect subscriber, publisher and subscribe to the topic
    try {
        _clockSubscriberQ->set_callback(*_callbackQ);
        _clockSubscriberQ->connect(connOpts);
        _echoPublisherQ->connect(connOpts);
        _clockSubscriberQ->subscribe(TOPICQ, 1);
        _clockSubscriberN->set_callback(*_callbackN);
        _clockSubscriberN->connect(connOpts);
        _echoPublisherN->connect(connOpts);
        _clockSubscriberN->subscribe(TOPICN, 1);
        _clockSubscriberZ->set_callback(*_callbackZ);
        _clockSubscriberZ->connect(connOpts);
        _echoPublisherZ->connect(connOpts);
        _clockSubscriberZ->subscribe(TOPICZ, 1);
        _clockSubscriberB->set_callback(*_callbackB);
        _clockSubscriberB->connect(connOpts);
        _echoPublisherB->connect(connOpts);
        _clockSubscriberB->subscribe(TOPICB, 1);
    } catch (const mqtt::exception& exc) {
        cerr << exc.what() << endl;
    }

}

void MqttAdapter_tests_a_compA::publish_echoPublisherQ()
{

    string value = to_string(component->mqttOutQ);
    auto pubmsg = mqtt::make_message(TOPICQ, value);

    try {
        _echoPublisherQ->publish(pubmsg);

    } catch (const mqtt::exception& exc) {
        cerr << exc.to_string() << endl;
    }
}

void MqttAdapter_tests_a_compA::publish_echoPublisherN()
{

    string value = to_string(component->mqttOutN);
    auto pubmsg = mqtt::make_message(TOPICN, value);

    try {
        _echoPublisherN->publish(pubmsg);

    } catch (const mqtt::exception& exc) {
        cerr << exc.to_string() << endl;
    }
}

void MqttAdapter_tests_a_compA::publish_echoPublisherZ()
{

    string value = to_string(component->mqttOutZ);
    auto pubmsg = mqtt::make_message(TOPICZ, value);

    try {
        _echoPublisherZ->publish(pubmsg);

    } catch (const mqtt::exception& exc) {
        cerr << exc.to_string() << endl;
    }
}

void MqttAdapter_tests_a_compA::publish_echoPublisherB()
{

    string value = to_string(component->mqttOutB ? 1.0 : 0.0);
    auto pubmsg = mqtt::make_message(TOPICB, value);

    try {
        _echoPublisherB->publish(pubmsg);

    } catch (const mqtt::exception& exc) {
        cerr << exc.to_string() << endl;
    }
}

void MqttAdapter_tests_a_compA::tick()
{
    publish_echoPublisherQ();
    publish_echoPublisherN();
    publish_echoPublisherZ();
    publish_echoPublisherB();
}

bool MqttAdapter_tests_a_compA::hasReceivedNewData() {
    return true;
}

