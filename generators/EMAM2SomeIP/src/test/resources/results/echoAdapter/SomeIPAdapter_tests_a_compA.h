#pragma once
#include "tests_a_compA.h"
#include "IAdapter_tests_a_compA.h"
#include <iomanip>
#include <iostream>
#include <sstream>

#include <condition_variable>
#include <thread>

#include <vsomeip/vsomeip.hpp>
#include <math.h>

using namespace std;

class SomeIPAdapter_tests_a_compA : public IAdapter_tests_a_compA {

public:

    SomeIPAdapter_tests_a_compA();

    void init(tests_a_compA *comp);

    void tick();
	bool hasReceivedNewData();  


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
		std::shared_ptr<vsomeip::application> in3_Subscriber;
		std::mutex mutex_in3;
		std::condition_variable condition_in3;

		void run_in3();
		void on_message_in3(const std::shared_ptr<vsomeip::message> &_message);
		void on_availability_in3(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available);
		std::shared_ptr<vsomeip::application> in4_Subscriber;
		std::mutex mutex_in4;
		std::condition_variable condition_in4;

		void run_in4();
		void on_message_in4(const std::shared_ptr<vsomeip::message> &_message);
		void on_availability_in4(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available);
		std::shared_ptr<vsomeip::application> in5_Subscriber;
		std::mutex mutex_in5;
		std::condition_variable condition_in5;

		void run_in5();
		void on_message_in5(const std::shared_ptr<vsomeip::message> &_message);
		void on_availability_in5(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available);

		std::shared_ptr<vsomeip::application> out1_Publisher;

		void publishout1_Publisher();
		std::shared_ptr<vsomeip::application> out2_Publisher;

		void publishout2_Publisher();
		std::shared_ptr<vsomeip::application> out3_Publisher;

		void publishout3_Publisher();
		std::shared_ptr<vsomeip::application> out4_Publisher;

		void publishout4_Publisher();
		std::shared_ptr<vsomeip::application> out5_Publisher;

		void publishout5_Publisher();
};
