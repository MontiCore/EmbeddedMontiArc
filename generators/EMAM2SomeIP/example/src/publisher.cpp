/* (c) https://github.com/MontiCore/monticore */
#include <iomanip>
#include <iostream>
#include <sstream>

#include <vsomeip/vsomeip.hpp>

#define SAMPLE_SERVICE_ID 0x1234
#define SAMPLE_INSTANCE_ID 0x5678
#define SAMPLE_METHOD_ID 0x0421
#define SAMPLE_EVENT_ID 0x2345
#define SAMPLE_EVENTGROUP_ID 0x1456

std::shared_ptr<vsomeip::application> app;

void on_message(const std::shared_ptr<vsomeip::message> &_request) {

    std::shared_ptr<vsomeip::payload> its_payload = _request->get_payload();
    vsomeip::length_t l = its_payload->get_length();
    
    double final = *((double*)its_payload->get_data());

    std::cout << "SERVICE: Received message from ["
        << std::setw(4) << std::setfill('0') << std::hex << _request->get_client() << "/"
        << std::setw(4) << std::setfill('0') << std::hex << _request->get_session() << "]: "
        << final << std::endl;
}

int main() {

   app = vsomeip::runtime::get()->create_application("Publisher");
   app->init();
   app->register_message_handler(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_METHOD_ID, on_message);
    
   double d = 2.5;
   uint8_t *byteArray = (uint8_t*)&d;
  
   vsomeip::byte_t *p;
   p = byteArray; 
  
   std::shared_ptr< vsomeip::payload > its_payload = vsomeip::runtime::get()->create_payload(p,8);
	
   std::set<vsomeip::eventgroup_t> its_groups;
   its_groups.insert(SAMPLE_EVENTGROUP_ID);
   app->offer_event(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENT_ID, its_groups, true);
   app->notify(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENT_ID, its_payload);
   app->offer_service(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID);
   app->start();
   

}
