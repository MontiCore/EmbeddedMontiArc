#include "SomeipAdapter_tests_a_compA.h"

#define SAMPLE_SERVICE_ID 0x1234
#define SAMPLE_INSTANCE_ID 0x5678
#define SAMPLE_METHOD_ID 0x0421
#define SAMPLE_EVENT_ID 0x2345
#define SAMPLE_EVENTGROUP_ID 0x1456

tests_a_compA* component;

std::shared_ptr<vsomeip::application> _clockSubscriber;

std::shared_ptr<vsomeip::application> _echoPublisher;

SomeipAdapter_tests_a_compA::SomeipAdapter_tests_a_compA() {}

void SomeipAdapter_tests_a_compA::init(tests_a_compA *comp)
{
	this->component = comp;
	
    _clockSubscriber = vsomeip::runtime::get()->create_application("Subscriber");
    _clockSubscriber->init();
    //_clockSubscriber->register_availability_handler(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, on_availability);
    _clockSubscriber->request_service(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID);

    _clockSubscriber->register_message_handler(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_METHOD_ID, on_message);

  	std::unique_lock<std::mutex> its_lock(mutex);
	
  	std::set<vsomeip::eventgroup_t> its_groups;
  	its_groups.insert(SAMPLE_EVENTGROUP_ID);
  	_clockSubscriber->request_event(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENT_ID, its_groups, true);
  	_clockSubscriber->subscribe(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENTGROUP_ID);
    _clockSubscriber->start();
    
}

void SomeipAdapter_tests_a_compA::on_message(const std::shared_ptr<vsomeip::message> &_request) {

    std::shared_ptr<vsomeip::payload> its_payload = _request->get_payload();
    vsomeip::length_t l = its_payload->get_length();
    
    double dataFromMessage = *((double*)its_payload->get_data());
    
    component->rosIn = dataFromMessage;

    std::cout << "SERVICE: Received message from ["
        << std::setw(4) << std::setfill('0') << std::hex << _request->get_client() << "/"
        << std::setw(4) << std::setfill('0') << std::hex << _request->get_session() << "]: "
        << final << std::endl;
}

void SomeipAdapter_tests_a_compA::publish_echoPublisher()
{
   _echoPublisher = vsomeip::runtime::get()->create_application("Publisher");
   _echoPublisher->init();
   //app->register_message_handler(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_METHOD_ID, on_message);
    
   double d = component->rosOut;
   uint8_t *byteArray = (uint8_t*)&d;
  
   vsomeip::byte_t *p;
   p = byteArray; 
  
   std::shared_ptr< vsomeip::payload > its_payload = vsomeip::runtime::get()->create_payload(p,8);
	
   std::set<vsomeip::eventgroup_t> its_groups;
   its_groups.insert(SAMPLE_EVENTGROUP_ID);
   _echoPublisher->offer_event(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENT_ID, its_groups, true);
   _echoPublisher->notify(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENT_ID, its_payload);
   _echoPublisher->offer_service(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID);
   _echoPublisher->start();
}

void SomeipAdapter_tests_a_compA::tick()
{
    publish_echoPublisher();
}

