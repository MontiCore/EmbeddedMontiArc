#pragma once
#include "tests_a_compA.h"
#include <iomanip>
#include <iostream>
#include <sstream>

#include <condition_variable>
#include <thread>

#include <vsomeip/vsomeip.hpp>

using namespace std;

class SomeipAdapter_tests_a_compA {
    
public:
    
    SomeipAdapter_tests_a_compA();
    
    void init(tests_a_compA* comp);
    
    void publish_echoPublisher();
    
    void tick();
    
    static void on_message(const std::shared_ptr<vsomeip::message> &_response);
    
    static void on_availability(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available);
    
private:
    
    tests_a_compA* component = nullptr;
};

