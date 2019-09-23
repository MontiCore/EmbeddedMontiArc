/* (c) https://github.com/MontiCore/monticore */
#include "SomeIPAdapter_tests_a_compA.h"

SomeIPAdapter_tests_a_compA::SomeIPAdapter_tests_a_compA() : public IAdapter_tests_a_compA {

}

void SomeIPAdapter_tests_a_compA::init(tests_a_compA *comp) {
    // Initialize component
	this->component = comp;


	// Intitialize Subscriber
		in1_Subscriber = vsomeip::runtime::get()->create_application("Subscriber in1");
		if (!in1_Subscriber->init()) {
            std::cerr << "Couldn't initialize Subscriber in1" << std::endl;
        }
		
		in1_Subscriber->register_availability_handler(11, 12, std::bind(&SomeIPAdapter_tests_a_compA::on_availability_in1, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
		in1_Subscriber->request_service(11, 12);
		in1_Subscriber->register_message_handler(11, 12, 0, std::bind(&SomeIPAdapter_tests_a_compA::on_message_in1, this, std::placeholders::_1));

		std::thread sender_in1(std::bind(&SomeIPAdapter_tests_a_compA::run_in1, this));
		in1_Subscriber->start();
		in2_Subscriber = vsomeip::runtime::get()->create_application("Subscriber in2");
		if (!in2_Subscriber->init()) {
            std::cerr << "Couldn't initialize Subscriber in2" << std::endl;
        }
		
		in2_Subscriber->register_availability_handler(21, 22, std::bind(&SomeIPAdapter_tests_a_compA::on_availability_in2, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
		in2_Subscriber->request_service(21, 22);
		in2_Subscriber->register_message_handler(21, 22, 0, std::bind(&SomeIPAdapter_tests_a_compA::on_message_in2, this, std::placeholders::_1));

		std::thread sender_in2(std::bind(&SomeIPAdapter_tests_a_compA::run_in2, this));
		in2_Subscriber->start();


	// Intitialize Publisher
    	out1_Publisher = vsomeip::runtime::get()->create_application("Publisher out1");
		if (!out1_Publisher->init()) {
            std::cerr << "Couldn't initialize Publisher out1" << std::endl;
        }
    	out1_Publisher->offer_service(111, 112);
		out1_Publisher->start();
    	out2_Publisher = vsomeip::runtime::get()->create_application("Publisher out2");
		if (!out2_Publisher->init()) {
            std::cerr << "Couldn't initialize Publisher out2" << std::endl;
        }
    	out2_Publisher->offer_service(121, 122);
		out2_Publisher->start();
}

void SomeIPAdapter_tests_a_compA::run_in1() {
	std::unique_lock<std::mutex> its_lock(mutex_in1);
	condition_in1.wait(its_lock);

	std::set<vsomeip::eventgroup_t> event_group;
	event_group.insert(13);
	in1_Subscriber->request_event(11, 12, 0, event_group, true);
	in1_Subscriber->subscribe(11, 12, 13);
}

void SomeIPAdapter_tests_a_compA::on_message_in1(const std::shared_ptr<vsomeip::message> &_message) {
	//read received message
    std::shared_ptr<vsomeip::payload> its_payload = _message->get_payload();
    vsomeip::length_t l = its_payload->get_length();
    double dataFromMessage = *((double*)its_payload->get_data());
    component->in1 = dataFromMessage;
	//print data to std out
    std::cout << "SERVICE in1: Received message from ["
			<< std::setw(4) << std::setfill('0') << std::hex << _message->get_client() << "/"
			<< std::setw(4) << std::setfill('0') << std::hex << _message->get_session() << "]: "
			<< dataFromMessage << std::endl;
}

void SomeIPAdapter_tests_a_compA::on_availability_in1(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available) {
	std::cout << "Service ["
		<< std::setw(4) << std::setfill('0') << std::hex << _service << "." << _instance
		<< "] is "
		<< (_is_available ? "available." : "NOT available.")
		<< std::endl;
	condition_in1.notify_one();
}
void SomeIPAdapter_tests_a_compA::run_in2() {
	std::unique_lock<std::mutex> its_lock(mutex_in2);
	condition_in2.wait(its_lock);

	std::set<vsomeip::eventgroup_t> event_group;
	event_group.insert(23);
	in2_Subscriber->request_event(21, 22, 0, event_group, true);
	in2_Subscriber->subscribe(21, 22, 23);
}

void SomeIPAdapter_tests_a_compA::on_message_in2(const std::shared_ptr<vsomeip::message> &_message) {
	//read received message
    std::shared_ptr<vsomeip::payload> its_payload = _message->get_payload();
    vsomeip::length_t l = its_payload->get_length();
    double dataFromMessage = *((double*)its_payload->get_data());
    component->in2 = dataFromMessage;
	//print data to std out
    std::cout << "SERVICE in2: Received message from ["
			<< std::setw(4) << std::setfill('0') << std::hex << _message->get_client() << "/"
			<< std::setw(4) << std::setfill('0') << std::hex << _message->get_session() << "]: "
			<< dataFromMessage << std::endl;
}

void SomeIPAdapter_tests_a_compA::on_availability_in2(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available) {
	std::cout << "Service ["
		<< std::setw(4) << std::setfill('0') << std::hex << _service << "." << _instance
		<< "] is "
		<< (_is_available ? "available." : "NOT available.")
		<< std::endl;
	condition_in2.notify_one();
}


void SomeIPAdapter_tests_a_compA::publishout1_Publisher()
{
    //Read data from component
    double d = component->out1;

	const vsomeip::byte_t its_data[] = { (uint8_t) d };
	std::shared_ptr<vsomeip::payload> payload = vsomeip::runtime::get()->create_payload();
	payload->set_data(its_data, sizeof(its_data));

	//Publish
	std::set<vsomeip::eventgroup_t> event_group;
	event_group.insert(113);
	out1_Publisher->offer_event(111, 112, 0, event_group, true);
	out1_Publisher->notify(111, 112, 0, payload);
}
void SomeIPAdapter_tests_a_compA::publishout2_Publisher()
{
    //Read data from component
    double d = component->out2;

	const vsomeip::byte_t its_data[] = { (uint8_t) d };
	std::shared_ptr<vsomeip::payload> payload = vsomeip::runtime::get()->create_payload();
	payload->set_data(its_data, sizeof(its_data));

	//Publish
	std::set<vsomeip::eventgroup_t> event_group;
	event_group.insert(123);
	out2_Publisher->offer_event(121, 122, 0, event_group, true);
	out2_Publisher->notify(121, 122, 0, payload);
}


void SomeIPAdapter_tests_a_compA::tick()
{
    	publishout1_Publisher();
    	publishout2_Publisher();
}
