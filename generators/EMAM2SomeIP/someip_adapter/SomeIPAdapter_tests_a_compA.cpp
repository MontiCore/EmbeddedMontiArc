/* (c) https://github.com/MontiCore/monticore */
#include "SomeIPAdapter_tests_a_compA.h"

SomeIPAdapter_tests_a_compA::SomeIPAdapter_tests_a_compA() {
	//choose random ids
    in1_service_id = 11;
	in1_instance_id = 12;
	in1_method_id = 13;
	in1_event_id = 14;
	in1_eventgroup_id = 15;

	in2_service_id = 21;
	in2_instance_id = 22;
	in2_method_id = 23;
	in2_event_id = 24;
	in2_eventgroup_id = 25;

    out1_service_id = 111;
	out1_instance_id = 112;
	out1_method_id = 113;
	out1_event_id = 114;
	out1_eventgroup_id = 115;

	out2_service_id = 121;
	out2_instance_id = 122;
	out2_method_id = 123;
	out2_event_id = 124;
	out2_eventgroup_id = 125;
}

void SomeIPAdapter_tests_a_compA::init(tests_a_compA *comp) {
    // Initialize component
	this->component = comp;

	// Intitialize subscriber for in1
    in1_Subscriber = vsomeip::runtime::get()->create_application("Subscriber");
    in1_Subscriber->init();
    in1_Subscriber->request_service(in1_service_id, in1_instance_id);
    in1_Subscriber->register_message_handler(in1_service_id, in1_instance_id, in1_method_id, std::bind(&SomeIPAdapter_tests_a_compA::on_message_in1, this, std::placeholders::_1));

	// Subscribe
  	std::set<vsomeip::eventgroup_t> in1_event_group;
  	in1_event_group.insert(in1_eventgroup_id);
  	in1_Subscriber->request_event(in1_service_id, in1_instance_id, in1_event_id, in1_event_group, true);
  	in1_Subscriber->subscribe(in1_service_id, in1_instance_id, in1_eventgroup_id);
    in1_Subscriber->start();


	// Intitialize subscriber for in2
    in2_Subscriber = vsomeip::runtime::get()->create_application("Subscriber");
    in2_Subscriber->init();
    in2_Subscriber->request_service(in2_service_id, in2_instance_id);
    in2_Subscriber->register_message_handler(in2_service_id, in2_instance_id, in2_method_id, std::bind(&SomeIPAdapter_tests_a_compA::on_message_in2, this, std::placeholders::_1));

	// Subscribe
  	std::set<vsomeip::eventgroup_t> in2_event_group;
  	in2_event_group.insert(in2_eventgroup_id);
  	in2_Subscriber->request_event(in2_service_id, in2_instance_id, in2_event_id, in2_event_group, true);
  	in2_Subscriber->subscribe(in2_service_id, in2_instance_id, in2_eventgroup_id);
    in2_Subscriber->start();


	// Intitialize Publisher for out1
    out1_Publisher = vsomeip::runtime::get()->create_application("Publisher");
    out1_Publisher->init();
    out1_Publisher->offer_service(out1_service_id, out1_instance_id);
    out1_Publisher->start();

    // Intitialize Publisher for out2
    out2_Publisher = vsomeip::runtime::get()->create_application("Publisher");
    out2_Publisher->init();
    out2_Publisher->offer_service(out2_service_id, out2_instance_id);
    out2_Publisher->start();
}

void SomeIPAdapter_tests_a_compA::on_message_in1(const std::shared_ptr<vsomeip::message> &_request) {
	//read received message
    std::shared_ptr<vsomeip::payload> its_payload = _request->get_payload();
    vsomeip::length_t l = its_payload->get_length();
    double dataFromMessage = *((double*)its_payload->get_data());
    component->in1 = dataFromMessage;
	//print data to std out
    std::cout << "SERVICE: Received message from ["
        << std::setw(4) << std::setfill('0') << std::hex << _request->get_client() << "/"
        << std::setw(4) << std::setfill('0') << std::hex << _request->get_session() << "]: "
        << dataFromMessage << std::endl;
}

void SomeIPAdapter_tests_a_compA::on_message_in2(const std::shared_ptr<vsomeip::message> &_request) {
	//read received message
    std::shared_ptr<vsomeip::payload> its_payload = _request->get_payload();
    vsomeip::length_t l = its_payload->get_length();
    double dataFromMessage = *((double*)its_payload->get_data());
    component->in2 = dataFromMessage;
	//print data to std out
    std::cout << "SERVICE: Received message from ["
        << std::setw(4) << std::setfill('0') << std::hex << _request->get_client() << "/"
        << std::setw(4) << std::setfill('0') << std::hex << _request->get_session() << "]: "
        << dataFromMessage << std::endl;
}

void SomeIPAdapter_tests_a_compA::publishout1_Publisher()
{
    //Read data from component
    double d = component->out1;

    //Create message
   	uint8_t *byteArray = (uint8_t*)&d;
   	vsomeip::byte_t *p;
   	p = byteArray;
   	std::shared_ptr< vsomeip::payload > payload = vsomeip::runtime::get()->create_payload(p,8);

	//Publish
	std::set<vsomeip::eventgroup_t> out1_event_group;
	out1_event_group.insert(out1_eventgroup_id);
	out1_Publisher->offer_event(out1_service_id, out1_instance_id, out1_event_id, out1_event_group, true);
	out1_Publisher->notify(out1_service_id, out1_instance_id, out1_event_id, payload);
}

void SomeIPAdapter_tests_a_compA::publishout2_Publisher()
{
	//Read data from component
    double d = component->out2;

    //Create message
   	uint8_t *byteArray = (uint8_t*)&d;
   	vsomeip::byte_t *p;
   	p = byteArray;
   	std::shared_ptr< vsomeip::payload > payload = vsomeip::runtime::get()->create_payload(p,8);

	//Publish
	std::set<vsomeip::eventgroup_t> out2_event_group;
	out2_event_group.insert(out2_eventgroup_id);
	out2_Publisher->offer_event(out2_service_id, out2_instance_id, out2_event_id, out2_event_group, true);
	out2_Publisher->notify(out2_service_id, out2_instance_id, out2_event_id, payload);
}

void SomeIPAdapter_tests_a_compA::tick()
{
    publishout1_Publisher();
    publishout2_Publisher();
}

bool SomeIPAdapter_tests_a_compA::hasReceivedNewData()
{
  return true;
}


