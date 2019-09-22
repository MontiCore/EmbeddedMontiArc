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

using namespace std;

class <@m.mwIdent/>Adapter_${model.getEscapedCompName()} : public IAdapter_${model.getEscapedCompName()} {

public:

    <@m.mwIdent/>Adapter_${model.getEscapedCompName()}();

    void init(${model.getEscapedCompName()} *comp);

    void tick();


private:

    ${model.getEscapedCompName()}* component = nullptr;

	<#list model.getIncomingPorts() as sub>
		std::shared_ptr<vsomeip::application> ${sub.getName()}_Subscriber;

		void on_message_${sub.getName()}(const std::shared_ptr<vsomeip::message> &_request);
		void on_state_${sub.getName()}(vsomeip::state_type_e _state);
    </#list>

	<#list model.getOutgoingPorts() as pub>
		std::shared_ptr<vsomeip::application> ${pub.getName()}_Publisher;

		void publish${pub.getName()}_Publisher()();
    </#list>

	void on_availability(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available);
};
