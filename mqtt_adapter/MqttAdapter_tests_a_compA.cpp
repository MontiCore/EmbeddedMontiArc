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
    _clockSubscriber = new mqtt::client(SERVER_ADDRESS, SUB_ID);
    _callback = new Callback(*_clockSubscriber, comp);
    _echoPublisher = new mqtt::client(SERVER_ADDRESS, PUB_ID);
    
    // Connect subscriber, publisher and subscribe to the topic
    try {
        _clockSubscriber->set_callback(*_callback);
        _clockSubscriber->connect(connOpts);
        _echoPublisher->connect(connOpts);
        _clockSubscriber->subscribe(TOPIC, 1);
        
    } catch (const mqtt::exception& exc) {
        cerr << exc.what() << endl;
    }
    
}

void MqttAdapter_tests_a_compA::publish_echoPublisher()
{
    
    string value = to_string(component->mqttOut);
    auto pubmsg = mqtt::make_message(TOPIC, value);
    
    try {
        _echoPublisher->publish(pubmsg);
        
    } catch (const mqtt::exception& exc) {
        cerr << exc.to_string() << endl;
    }
}

void MqttAdapter_tests_a_compA::tick()
{
    publish_echoPublisher();
}

