#pragma once
#include "tests_a_compA.h"
#include <iomanip>
#include <iostream>
#include <sstream>

#include <condition_variable>
#include <thread>

#include <vsomeip/vsomeip.hpp>

using namespace std;

class SomeIPAdapter_tests_a_compA {

public:

    SomeIPAdapter_tests_a_compA();

    SomeIPAdapter_tests_a_compA(int service_id, int instance_id, int method_id, int event_id, int eventgroup_id);

    void init(tests_a_compA* comp);

    void publish_echoPublisher();

    void tick();

    void on_message(const std::shared_ptr<vsomeip::message> &_response);

private:

    tests_a_compA* component = nullptr;

	std::shared_ptr<vsomeip::application> _clockSubscriber;

	std::shared_ptr<vsomeip::application> _echoPublisher;

	int service_id;
	int instance_id;
	int method_id;
	int event_id;
	int eventgroup_id;
};
