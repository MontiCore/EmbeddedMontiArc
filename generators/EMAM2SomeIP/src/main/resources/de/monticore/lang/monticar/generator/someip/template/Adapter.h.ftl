<#-- (c) https://github.com/MontiCore/monticore -->
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
#include <math.h>

using namespace std;

class <@m.mwIdent/>Adapter_${model.getEscapedCompName()} : public IAdapter_${model.getEscapedCompName()} {

public:

    <@m.mwIdent/>Adapter_${model.getEscapedCompName()}();

    void init(${model.getEscapedCompName()} *comp);

    void tick();
	bool hasReceivedNewData();  


private:

    ${model.getEscapedCompName()}* component = nullptr;

	<#list model.getIncomingPorts() as sub>
		std::shared_ptr<vsomeip::application> ${sub.getName()}_Subscriber;
		std::mutex mutex_${sub.getName()};
		std::condition_variable condition_${sub.getName()};

		void run_${sub.getName()}();
		void on_message_${sub.getName()}(const std::shared_ptr<vsomeip::message> &_message);
		void on_availability_${sub.getName()}(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available);
    </#list>

	<#list model.getOutgoingPorts() as pub>
		std::shared_ptr<vsomeip::application> ${pub.getName()}_Publisher;

		void publish${pub.getName()}_Publisher();
    </#list>
};
