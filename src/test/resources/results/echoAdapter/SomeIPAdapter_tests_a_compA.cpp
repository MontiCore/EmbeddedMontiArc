#include "SomeIPAdapter_tests_a_compA.h"

#define SAMPLE_SERVICE_ID 0x1234
#define SAMPLE_INSTANCE_ID 0x5678
#define SAMPLE_METHOD_ID 0x0421
#define SAMPLE_EVENT_ID 0x2345
#define SAMPLE_EVENTGROUP_ID 0x1456

SomeIPAdapter_tests_a_compA::SomeIPAdapter_tests_a_compA() {}

void SomeIPAdapter_tests_a_compA::init(tests_a_compA *comp) {
	// Initialize component
	this->component = comp;

	// Intitialize subscriber
    _clockSubscriber = vsomeip::runtime::get()->create_application("Subscriber");
    _clockSubscriber->init();
    _clockSubscriber->request_service(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID);
    _clockSubscriber->register_message_handler(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_METHOD_ID, std::bind(&SomeIPAdapter_tests_a_compA::on_message, this, 		std::placeholders::_1));

	// Subscribe
  	std::set<vsomeip::eventgroup_t> event_group;
  	event_group.insert(SAMPLE_EVENTGROUP_ID);
  	_clockSubscriber->request_event(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENT_ID, event_group, true);
  	_clockSubscriber->subscribe(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENTGROUP_ID);
    _clockSubscriber->start();

	// Intitialize Publisher
    _echoPublisher = vsomeip::runtime::get()->create_application("Publisher");
    _echoPublisher->init();
    _echoPublisher->offer_service(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID);
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
	event_group.insert(SAMPLE_EVENTGROUP_ID);
	_echoPublisher->offer_event(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENT_ID, event_group, true);
	_echoPublisher->notify(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENT_ID, payload);
}

void SomeIPAdapter_tests_a_compA::tick()
{
    publish_echoPublisher();
}
