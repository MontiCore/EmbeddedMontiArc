
#include "MqttAdapter_tests_a_compA.h"

MqttAdapter_tests_a_compA::MqttAdapter_tests_a_compA()
{

}

void MqttAdapter_tests_a_compA::init(tests_a_compA *comp)
{
    // Initialize component
    this->component = comp;

    // Initialize connect options
    connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);

    // Intitialize callbacks, subscribers and publishers
    	_sub_portA = new client(SERVER_ADDRESS, portA);
        _callback_portA = new Callback(*_sub_portA, comp, portA);

        _pub_portC = new client(SERVER_ADDRESS, portC);

    // Connect subscribers, publishers and subscribe to the topics
    try {
    	_sub_portA->set_callback(*_callback_portA);
    	_sub_portA->connect(connOpts);
    	_sub_portA->subscribe("/clock", 1);
        _pub_portC->connect(connOpts);

    } catch (const mqtt::exception& exc) {
        cerr << exc.what() << endl;
    }

}

void MqttAdapter_tests_a_compA::publish_echo_portC()
{
	string value = to_string(component->portC);
	auto pubmsg = make_message("/clock", value);

	try {
		_pub_portC->publish(pubmsg);

	}
	catch (const exception& exc) {
	    cerr << exc.to_string() << endl;
	}
}


void MqttAdapter_tests_a_compA::tick()
{
        publish_echo_portC();
}
