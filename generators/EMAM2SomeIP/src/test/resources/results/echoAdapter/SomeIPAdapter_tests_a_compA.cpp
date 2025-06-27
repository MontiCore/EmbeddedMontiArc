#include "SomeIPAdapter_tests_a_compA.h"

SomeIPAdapter_tests_a_compA::SomeIPAdapter_tests_a_compA() {

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
		in3_Subscriber = vsomeip::runtime::get()->create_application("Subscriber in3");
		if (!in3_Subscriber->init()) {
            std::cerr << "Couldn't initialize Subscriber in3" << std::endl;
        }
		
		in3_Subscriber->register_availability_handler(31, 32, std::bind(&SomeIPAdapter_tests_a_compA::on_availability_in3, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
		in3_Subscriber->request_service(31, 32);
		in3_Subscriber->register_message_handler(31, 32, 0, std::bind(&SomeIPAdapter_tests_a_compA::on_message_in3, this, std::placeholders::_1));

		std::thread sender_in3(std::bind(&SomeIPAdapter_tests_a_compA::run_in3, this));
		in3_Subscriber->start();
		in4_Subscriber = vsomeip::runtime::get()->create_application("Subscriber in4");
		if (!in4_Subscriber->init()) {
            std::cerr << "Couldn't initialize Subscriber in4" << std::endl;
        }
		
		in4_Subscriber->register_availability_handler(41, 42, std::bind(&SomeIPAdapter_tests_a_compA::on_availability_in4, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
		in4_Subscriber->request_service(41, 42);
		in4_Subscriber->register_message_handler(41, 42, 0, std::bind(&SomeIPAdapter_tests_a_compA::on_message_in4, this, std::placeholders::_1));

		std::thread sender_in4(std::bind(&SomeIPAdapter_tests_a_compA::run_in4, this));
		in4_Subscriber->start();
		in5_Subscriber = vsomeip::runtime::get()->create_application("Subscriber in5");
		if (!in5_Subscriber->init()) {
            std::cerr << "Couldn't initialize Subscriber in5" << std::endl;
        }
		
		in5_Subscriber->register_availability_handler(51, 52, std::bind(&SomeIPAdapter_tests_a_compA::on_availability_in5, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
		in5_Subscriber->request_service(51, 52);
		in5_Subscriber->register_message_handler(51, 52, 0, std::bind(&SomeIPAdapter_tests_a_compA::on_message_in5, this, std::placeholders::_1));

		std::thread sender_in5(std::bind(&SomeIPAdapter_tests_a_compA::run_in5, this));
		in5_Subscriber->start();


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
    	out3_Publisher = vsomeip::runtime::get()->create_application("Publisher out3");
		if (!out3_Publisher->init()) {
            std::cerr << "Couldn't initialize Publisher out3" << std::endl;
        }
    	out3_Publisher->offer_service(131, 132);
		out3_Publisher->start();
    	out4_Publisher = vsomeip::runtime::get()->create_application("Publisher out4");
		if (!out4_Publisher->init()) {
            std::cerr << "Couldn't initialize Publisher out4" << std::endl;
        }
    	out4_Publisher->offer_service(141, 142);
		out4_Publisher->start();
    	out5_Publisher = vsomeip::runtime::get()->create_application("Publisher out5");
		if (!out5_Publisher->init()) {
            std::cerr << "Couldn't initialize Publisher out5" << std::endl;
        }
    	out5_Publisher->offer_service(151, 152);
		out5_Publisher->start();
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
	
        // double
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
	
        // double
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
void SomeIPAdapter_tests_a_compA::run_in3() {
	std::unique_lock<std::mutex> its_lock(mutex_in3);
	condition_in3.wait(its_lock);

	std::set<vsomeip::eventgroup_t> event_group;
	event_group.insert(33);
	in3_Subscriber->request_event(31, 32, 0, event_group, true);
	in3_Subscriber->subscribe(31, 32, 33);
}

void SomeIPAdapter_tests_a_compA::on_message_in3(const std::shared_ptr<vsomeip::message> &_message) {
	//read received message
    std::shared_ptr<vsomeip::payload> its_payload = _message->get_payload();
    vsomeip::length_t l = its_payload->get_length();
    double dataFromMessage = *((double*)its_payload->get_data());
	
        //int
		component->in3 = (int) round(dataFromMessage);

	//print data to std out
    std::cout << "SERVICE in3: Received message from ["
			<< std::setw(4) << std::setfill('0') << std::hex << _message->get_client() << "/"
			<< std::setw(4) << std::setfill('0') << std::hex << _message->get_session() << "]: "
			<< dataFromMessage << std::endl;
}

void SomeIPAdapter_tests_a_compA::on_availability_in3(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available) {
	std::cout << "Service ["
		<< std::setw(4) << std::setfill('0') << std::hex << _service << "." << _instance
		<< "] is "
		<< (_is_available ? "available." : "NOT available.")
		<< std::endl;
	condition_in3.notify_one();
}
void SomeIPAdapter_tests_a_compA::run_in4() {
	std::unique_lock<std::mutex> its_lock(mutex_in4);
	condition_in4.wait(its_lock);

	std::set<vsomeip::eventgroup_t> event_group;
	event_group.insert(43);
	in4_Subscriber->request_event(41, 42, 0, event_group, true);
	in4_Subscriber->subscribe(41, 42, 43);
}

void SomeIPAdapter_tests_a_compA::on_message_in4(const std::shared_ptr<vsomeip::message> &_message) {
	//read received message
    std::shared_ptr<vsomeip::payload> its_payload = _message->get_payload();
    vsomeip::length_t l = its_payload->get_length();
    double dataFromMessage = *((double*)its_payload->get_data());
	
        //int
		component->in4 = (int) round(dataFromMessage);

	//print data to std out
    std::cout << "SERVICE in4: Received message from ["
			<< std::setw(4) << std::setfill('0') << std::hex << _message->get_client() << "/"
			<< std::setw(4) << std::setfill('0') << std::hex << _message->get_session() << "]: "
			<< dataFromMessage << std::endl;
}

void SomeIPAdapter_tests_a_compA::on_availability_in4(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available) {
	std::cout << "Service ["
		<< std::setw(4) << std::setfill('0') << std::hex << _service << "." << _instance
		<< "] is "
		<< (_is_available ? "available." : "NOT available.")
		<< std::endl;
	condition_in4.notify_one();
}
void SomeIPAdapter_tests_a_compA::run_in5() {
	std::unique_lock<std::mutex> its_lock(mutex_in5);
	condition_in5.wait(its_lock);

	std::set<vsomeip::eventgroup_t> event_group;
	event_group.insert(53);
	in5_Subscriber->request_event(51, 52, 0, event_group, true);
	in5_Subscriber->subscribe(51, 52, 53);
}

void SomeIPAdapter_tests_a_compA::on_message_in5(const std::shared_ptr<vsomeip::message> &_message) {
	//read received message
    std::shared_ptr<vsomeip::payload> its_payload = _message->get_payload();
    vsomeip::length_t l = its_payload->get_length();
    double dataFromMessage = *((double*)its_payload->get_data());
	
        //bool
		component->in5 = (dataFromMessage > 1.0e-10);

	//print data to std out
    std::cout << "SERVICE in5: Received message from ["
			<< std::setw(4) << std::setfill('0') << std::hex << _message->get_client() << "/"
			<< std::setw(4) << std::setfill('0') << std::hex << _message->get_session() << "]: "
			<< dataFromMessage << std::endl;
}

void SomeIPAdapter_tests_a_compA::on_availability_in5(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available) {
	std::cout << "Service ["
		<< std::setw(4) << std::setfill('0') << std::hex << _service << "." << _instance
		<< "] is "
		<< (_is_available ? "available." : "NOT available.")
		<< std::endl;
	condition_in5.notify_one();
}


void SomeIPAdapter_tests_a_compA::publishout1_Publisher()
{
    //Read data from component
        // double
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
        // double
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
void SomeIPAdapter_tests_a_compA::publishout3_Publisher()
{
    //Read data from component
        //int
    	double d = 1.0 * component->out3;

	const vsomeip::byte_t its_data[] = { (uint8_t) d };
	std::shared_ptr<vsomeip::payload> payload = vsomeip::runtime::get()->create_payload();
	payload->set_data(its_data, sizeof(its_data));

	//Publish
	std::set<vsomeip::eventgroup_t> event_group;
	event_group.insert(133);
	out3_Publisher->offer_event(131, 132, 0, event_group, true);
	out3_Publisher->notify(131, 132, 0, payload);
}
void SomeIPAdapter_tests_a_compA::publishout4_Publisher()
{
    //Read data from component
        //int
    	double d = 1.0 * component->out4;

	const vsomeip::byte_t its_data[] = { (uint8_t) d };
	std::shared_ptr<vsomeip::payload> payload = vsomeip::runtime::get()->create_payload();
	payload->set_data(its_data, sizeof(its_data));

	//Publish
	std::set<vsomeip::eventgroup_t> event_group;
	event_group.insert(143);
	out4_Publisher->offer_event(141, 142, 0, event_group, true);
	out4_Publisher->notify(141, 142, 0, payload);
}
void SomeIPAdapter_tests_a_compA::publishout5_Publisher()
{
    //Read data from component
        //bool
    	double d = component->out5 ? 1.0 : 0.0;

	const vsomeip::byte_t its_data[] = { (uint8_t) d };
	std::shared_ptr<vsomeip::payload> payload = vsomeip::runtime::get()->create_payload();
	payload->set_data(its_data, sizeof(its_data));

	//Publish
	std::set<vsomeip::eventgroup_t> event_group;
	event_group.insert(153);
	out5_Publisher->offer_event(151, 152, 0, event_group, true);
	out5_Publisher->notify(151, 152, 0, payload);
}


void SomeIPAdapter_tests_a_compA::tick()
{
    	publishout1_Publisher();
    	publishout2_Publisher();
    	publishout3_Publisher();
    	publishout4_Publisher();
    	publishout5_Publisher();
}

bool SomeIPAdapter_tests_a_compA::hasReceivedNewData()
{
  return true;
}
