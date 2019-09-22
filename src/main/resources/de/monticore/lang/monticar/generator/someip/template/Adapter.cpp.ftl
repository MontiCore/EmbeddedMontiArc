<#-- (c) https://github.com/MontiCore/monticore -->
<#import "SomeIPMacros.ftl" as m>
#include "<@m.mwIdent/>Adapter_${model.getEscapedCompName()}.h"

<@m.mwIdent/>Adapter_${model.getEscapedCompName()}::<@m.mwIdent/>Adapter_${model.getEscapedCompName()}() : public IAdapter_${model.getEscapedCompName()} {

}

void <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::init(${model.getEscapedCompName()} *comp) {
    // Initialize component
	this->component = comp;


	// Intitialize Subscriber
	<#list model.getIncomingPorts() as sub>
		${sub.getName()}_Subscriber = vsomeip::runtime::get()->create_application("Subscriber");
		if (!${sub.getName()}_Subscriber->init()) {
            std::cerr << "Couldn't initialize Subscriber ${sub.getName()}" << std::endl;
            return false;
        }
		
        ${sub.getName()}_Subscriber->register_state_handler(std::bind(&SomeIPAdapter_${model.getEscapedCompName()}::on_state, this, std::placeholders::_1);

		${sub.getName()}_Subscriber->register_message_handler(${model.getServiceId(sub)}, ${model.getInstanceId(sub)}, ${model.getMethodId(sub)}, std::bind(&SomeIPAdapter_${model.getEscapedCompName()}::on_message_${sub.getName()}, this, std::placeholders::_1));
		${sub.getName()}_Subscriber->register_availability_handler(${model.getServiceId(sub)}, ${model.getInstanceId(sub)}, std::bind(&SomeIPAdapter_${model.getEscapedCompName()}::on_availability, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

		// Subscribe
		std::set<vsomeip::eventgroup_t> event_group;
		event_group.insert(${model.getEventGroupId(sub)});
		${sub.getName()}_Subscriber->request_event(${model.getServiceId(sub)}, ${model.getInstanceId(sub)}, ${model.getEventId(sub)}, event_group, true);
		${sub.getName()}_Subscriber->subscribe(${model.getServiceId(sub)}, ${model.getInstanceId(sub)}, ${model.getEventGroupId(sub)});
    </#list>


	// Intitialize Publisher
	<#list model.getOutgoingPorts() as pub>
    	${pub.getName()}_Publisher = vsomeip::runtime::get()->create_application("Publisher");
		if (!${pub.getName()}_Publisher->init()) {
            std::cerr << "Couldn't initialize Publisher ${pub.getName()}" << std::endl;
            return false;
        }
    	${pub.getName()}_Publisher->offer_service(${model.getServiceId(pub)}, ${model.getInstanceId(pub)});
    </#list>

	// Start Subscriber
	<#list model.getIncomingPorts() as sub>
		${sub.getName()}_Subscriber->start();
    </#list>

	// Start Publisher
	<#list model.getOutgoingPorts() as pub>
		${pub.getName()}_Publisher->start();
    </#list>
}

<#list model.getIncomingPorts() as sub>
void <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::on_message_${sub.getName()}(const std::shared_ptr<vsomeip::message> &_request) {
	//read received message
    std::shared_ptr<vsomeip::payload> its_payload = _request->get_payload();
    vsomeip::length_t l = its_payload->get_length();
    double dataFromMessage = *((double*)its_payload->get_data());
    component->${sub.getName()} = dataFromMessage;
	//print data to std out
    std::cout << "SERVICE ${sub.getName()}: Received message from ["
			<< std::setw(4) << std::setfill('0') << std::hex << _request->get_client() << "/"
			<< std::setw(4) << std::setfill('0') << std::hex << _request->get_session() << "]: "
			<< dataFromMessage << std::endl;
}
</#list>


<#list model.getOutgoingPorts() as pub>
void <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::publish${pub.getName()}_Publisher()
{
    //Read data from component
    double d = component->${pub.getName()};

    //Create message
   	uint8_t *byteArray = (uint8_t*)&d;
   	vsomeip::byte_t *p;
   	p = byteArray;
   	std::shared_ptr< vsomeip::payload > payload = vsomeip::runtime::get()->create_payload(p,8);

	//Publish
	std::set<vsomeip::eventgroup_t> event_group;
	event_group.insert(${model.getEventGroupId(pub)});
	${pub.getName()}_Publisher->offer_event(${model.getServiceId(pub)}, ${model.getInstanceId(pub)}, ${model.getEventId(pub)}, event_group, true);
	${pub.getName()}_Publisher->notify(${model.getServiceId(pub)}, ${model.getInstanceId(pub)}, ${model.getEventId(pub)}, payload);
}
</#list>

<#list model.getIncomingPorts() as sub>
void <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::on_state_${sub.getName()}(vsomeip::state_type_e _state) {
	if (_state == vsomeip::state_type_e::ST_REGISTERED) {
		${sub.getName()}_Subscriber->request_service(${model.getServiceId(sub)}, ${model.getInstanceId(sub)});
	}
}
</#list>


void <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::on_availability(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available) {
	std::cout << "Service ["
		<< std::setw(4) << std::setfill('0') << std::hex << _service << "." << _instance
		<< "] is "
		<< (_is_available ? "available." : "NOT available.")
		<< std::endl;
}


void <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::tick()
{
	<#list model.getOutgoingPorts() as pub>
    	publish${pub.getName()}_Publisher();
    </#list>
}
