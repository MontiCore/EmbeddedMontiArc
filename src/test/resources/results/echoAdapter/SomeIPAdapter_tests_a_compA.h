/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include "tests_a_compA.h"
#include "IAdapter_tests_a_compA.h"
#include <iomanip>
#include <iostream>
#include <sstream>

#include <condition_variable>
#include <thread>

#include <vsomeip/vsomeip.hpp>

using namespace std;

class SomeIPAdapter_tests_a_compA : public IAdapter_tests_a_compA {

public:

    SomeIPAdapter_tests_a_compA();

    void init(tests_a_compA *comp);

    void tick();


private:

    tests_a_compA* component = nullptr;

		std::shared_ptr<vsomeip::application> in1_Subscriber;
		std::mutex mutex_in1;
		std::condition_variable condition_in1;

		void run_in1();
		void on_message_in1(const std::shared_ptr<vsomeip::message> &_message);
		void on_availability_in1(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available);
		std::shared_ptr<vsomeip::application> in2_Subscriber;
		std::mutex mutex_in2;
		std::condition_variable condition_in2;

		void run_in2();
		void on_message_in2(const std::shared_ptr<vsomeip::message> &_message);
		void on_availability_in2(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available);

		std::shared_ptr<vsomeip::application> out1_Publisher;

		void publishout1_Publisher();
		std::shared_ptr<vsomeip::application> out2_Publisher;

		void publishout2_Publisher();
};
