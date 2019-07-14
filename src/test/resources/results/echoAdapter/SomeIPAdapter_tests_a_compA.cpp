#include "SomeIPAdapter_tests_a_compA.h"

SomeIPAdapter_tests_a_compA::SomeIPAdapter_tests_a_compA() {
	//choose random ids
    service_id = 1;
	instance_id = 2;
	method_id = 3;
	event_id = 4;
	eventgroup_id = 5;
}

SomeIPAdapter_tests_a_compA::SomeIPAdapter_tests_a_compA(int service_id, int instance_id, int method_id, int event_id, int eventgroup_id) {
    this->service_id = service_id;
	this->instance_id = instance_id;
	this->method_id = method_id;
	this->event_id = event_id;
	this->eventgroup_id = eventgroup_id;
}

void SomeIPAdapter_tests_a_compA::init(tests_a_compA *comp) {
	// Initialize component
	this->component = comp;

	// Intitialize subscriber
    _clockSubscriber = vsomeip::runtime::get()->create_application("Subscriber");
    _clockSubscriber->init();
    _clockSubscriber->request_service(service_id, instance_id);
    _clockSubscriber->register_message_handler(service_id, instance_id, method_id, std::bind(&SomeIPAdapter_tests_a_compA::on_message, this, std::placeholders::_1));

	// Subscribe
  	std::set<vsomeip::eventgroup_t> event_group;
  	event_group.insert(eventgroup_id);
  	_clockSubscriber->request_event(service_id, instance_id, event_id, event_group, true);
  	_clockSubscriber->subscribe(service_id, instance_id, eventgroup_id);
    _clockSubscriber->start();

	// Intitialize Publisher
    _echoPublisher = vsomeip::runtime::get()->create_application("Publisher");
    _echoPublisher->init();
    _echoPublisher->offer_service(service_id, instance_id);
    _echoPublisher->start();
}

void SomeIPAdapter_tests_a_compA::on_message(const std::shared_ptr<vsomeip::message> &_request) {
	//read received message
    std::shared_ptr<vsomeip::payload> its_payload = _request->get_payload();
    vsomeip::length_t l = its_payload->get_length();
    double dataFromMessage = *((double*)its_payload->get_data());
    component->someIPIn = dataFromMessage;
	//print data to std out
    std::cout << "SERVICE: Received message from ["
        << std::setw(4) << std::setfill('0') << std::hex << _request->get_client() << "/"
        << std::setw(4) << std::setfill('0') << std::hex << _request->get_session() << "]: "
        << dataFromMessage << std::endl;
}

void SomeIPAdapter_tests_a_compA::publish_echoPublisher()
{
	//Read data from component
    double d = component->someIPOut;

    //Create message
   	uint8_t *byteArray = (uint8_t*)&d;
   	vsomeip::byte_t *p;
   	p = byteArray;
   	std::shared_ptr< vsomeip::payload > payload = vsomeip::runtime::get()->create_payload(p,8);

	//Publish
	std::set<vsomeip::eventgroup_t> event_group;
	event_group.insert(eventgroup_id);
	_echoPublisher->offer_event(service_id, instance_id, event_id, event_group, true);
	_echoPublisher->notify(service_id, instance_id, event_id, payload);
}

void SomeIPAdapter_tests_a_compA::tick()
{
    publish_echoPublisher();
}
