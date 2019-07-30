<#import "SomeIPMacros.ftl" as m>
#pragma once
#include "${model.getEscapedCompName()}.h"
#include "IAdapter_${model.getEscapedCompName()}.h"

#include <iomanip>
#include <iostream>
#include <sstream>

#include <condition_variable>
#include <thread>

#include <vsomeip/vsomeip.hpp>

using namespace std;

class <@m.mwIdent/>Adapter_${model.getEscapedCompName()} : public IAdapter_${model.getEscapedCompName()} {

public:

    <@m.mwIdent/>Adapter_${model.getEscapedCompName()}();

    void init(${model.getEscapedCompName()} *comp);

    void on_message_in1(const std::shared_ptr<vsomeip::message> &_request);

    void on_message_in2(const std::shared_ptr<vsomeip::message> &_request);

    void publishout1_Publisher();

	void publishout2_Publisher();

    void tick();


private:

    ${model.getEscapedCompName()}* component = nullptr;

	std::shared_ptr<vsomeip::application> in1_Subscriber;

	std::shared_ptr<vsomeip::application> in2_Subscriber;

	std::shared_ptr<vsomeip::application> out1_Publisher;

	std::shared_ptr<vsomeip::application> out2_Publisher;

    int in1_service_id;
	int in1_instance_id;
	int in1_method_id;
	int in1_event_id;
	int in1_eventgroup_id;

	int in2_service_id;
	int in2_instance_id;
	int in2_method_id;
	int in2_event_id;
	int in2_eventgroup_id;

	int out1_service_id;
	int out1_instance_id;
	int out1_method_id;
	int out1_event_id;
	int out1_eventgroup_id;

	int out2_service_id;
	int out2_instance_id;
	int out2_method_id;
	int out2_event_id;
	int out2_eventgroup_id;
};
