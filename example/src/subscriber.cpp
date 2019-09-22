/* (c) https://github.com/MontiCore/monticore */
#include <iomanip>
#include <iostream>
#include <sstream>

#include <condition_variable>
#include <thread>

#include <vsomeip/vsomeip.hpp>

#define SAMPLE_SERVICE_ID 0x1234
#define SAMPLE_INSTANCE_ID 0x5678
#define SAMPLE_METHOD_ID 0x0421
#define SAMPLE_EVENT_ID 0x2345
#define SAMPLE_EVENTGROUP_ID 0x1456

std::shared_ptr< vsomeip::application > app;
std::mutex mutex;
std::condition_variable condition;

void run() {
  std::unique_lock<std::mutex> its_lock(mutex);
  condition.wait(its_lock);

  std::set<vsomeip::eventgroup_t> its_groups;
  its_groups.insert(SAMPLE_EVENTGROUP_ID);
  app->request_event(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENT_ID, its_groups, true);
  app->subscribe(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, SAMPLE_EVENTGROUP_ID);
 
}

void on_message(const std::shared_ptr<vsomeip::message> &_request) {

    std::shared_ptr<vsomeip::payload> its_payload = _request->get_payload();
    vsomeip::length_t l = its_payload->get_length();
    
    double final = *((double*)its_payload->get_data());

    std::cout << "SERVICE: Received message from ["
        << std::setw(4) << std::setfill('0') << std::hex << _request->get_client() << "/"
        << std::setw(4) << std::setfill('0') << std::hex << _request->get_session() << "]: "
        << final << std::endl;
}

void on_availability(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available) {
    std::cout << "CLIENT: Service ["
            << std::setw(4) << std::setfill('0') << std::hex << _service << "." << _instance
            << "] is "
            << (_is_available ? "available." : "NOT available.")
            << std::endl;
    condition.notify_one();
}

int main() {


    app = vsomeip::runtime::get()->create_application("Hello");
    app->init();
    app->register_availability_handler(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID, on_availability);
    app->request_service(SAMPLE_SERVICE_ID, SAMPLE_INSTANCE_ID);

    app->register_message_handler(vsomeip::ANY_SERVICE, vsomeip::ANY_INSTANCE, vsomeip::ANY_METHOD, on_message);

    std::thread sender(run);
    app->start();
}
