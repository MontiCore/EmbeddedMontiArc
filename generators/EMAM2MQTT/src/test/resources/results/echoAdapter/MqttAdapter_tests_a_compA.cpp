/* (c) https://github.com/MontiCore/monticore */

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
          _sub_portA = new client(SERVER_ADDRESS, "portA");
          _callback_portA = new CallbackQ(*_sub_portA, &(component->portA));
          _sub_portB = new client(SERVER_ADDRESS, "portB");
          _callback_portB = new CallbackQ(*_sub_portB, &(component->portB));
          _sub_portE = new client(SERVER_ADDRESS, "portE");
          _callback_portE = new CallbackN(*_sub_portE, &(component->portE));
          _sub_portG = new client(SERVER_ADDRESS, "portG");
          _callback_portG = new CallbackZ(*_sub_portG, &(component->portG));
          _sub_portI = new client(SERVER_ADDRESS, "portI");
          _callback_portI = new CallbackB(*_sub_portI, &(component->portI));

    _pub_portC = new client(SERVER_ADDRESS, "portC");
    _pub_portD = new client(SERVER_ADDRESS, "portD");
    _pub_portF = new client(SERVER_ADDRESS, "portF");
    _pub_portH = new client(SERVER_ADDRESS, "portH");
    _pub_portJ = new client(SERVER_ADDRESS, "portJ");

    // Connect subscribers, publishers and subscribe to the topics
    try {
    	_sub_portA->set_callback(*_callback_portA);
    	_sub_portA->connect(connOpts);
    	_sub_portA->subscribe("/clock", 1);
    	_sub_portB->set_callback(*_callback_portB);
    	_sub_portB->connect(connOpts);
    	_sub_portB->subscribe("/clock", 1);
    	_sub_portE->set_callback(*_callback_portE);
    	_sub_portE->connect(connOpts);
    	_sub_portE->subscribe("/clockN", 1);
    	_sub_portG->set_callback(*_callback_portG);
    	_sub_portG->connect(connOpts);
    	_sub_portG->subscribe("/clockZ", 1);
    	_sub_portI->set_callback(*_callback_portI);
    	_sub_portI->connect(connOpts);
    	_sub_portI->subscribe("/clockB", 1);
        _pub_portC->connect(connOpts);
        _pub_portD->connect(connOpts);
        _pub_portF->connect(connOpts);
        _pub_portH->connect(connOpts);
        _pub_portJ->connect(connOpts);

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
	catch (const mqtt::exception& exc) {
	    cerr << exc.to_string() << endl;
	}
}
void MqttAdapter_tests_a_compA::publish_echo_portD()
{
      string value = to_string(component->portD);
	auto pubmsg = make_message("/clock", value);

	try {
		_pub_portD->publish(pubmsg);

	}
	catch (const mqtt::exception& exc) {
	    cerr << exc.to_string() << endl;
	}
}
void MqttAdapter_tests_a_compA::publish_echo_portF()
{
      string value = to_string(component->portF);
	auto pubmsg = make_message("/clockN", value);

	try {
		_pub_portF->publish(pubmsg);

	}
	catch (const mqtt::exception& exc) {
	    cerr << exc.to_string() << endl;
	}
}
void MqttAdapter_tests_a_compA::publish_echo_portH()
{
      string value = to_string(component->portH);
	auto pubmsg = make_message("/clockZ", value);

	try {
		_pub_portH->publish(pubmsg);

	}
	catch (const mqtt::exception& exc) {
	    cerr << exc.to_string() << endl;
	}
}
void MqttAdapter_tests_a_compA::publish_echo_portJ()
{
      string value = to_string(component->portJ ? 1.0 : 0.0);
	auto pubmsg = make_message("/clockB", value);

	try {
		_pub_portJ->publish(pubmsg);

	}
	catch (const mqtt::exception& exc) {
	    cerr << exc.to_string() << endl;
	}
}


void MqttAdapter_tests_a_compA::tick()
{
    publish_echo_portC();
    publish_echo_portD();
    publish_echo_portF();
    publish_echo_portH();
    publish_echo_portJ();
}

bool MqttAdapter_tests_a_compA::hasReceivedNewData()
{
  return true;
}
