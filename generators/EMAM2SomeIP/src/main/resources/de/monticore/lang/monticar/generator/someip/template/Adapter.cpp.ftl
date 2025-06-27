<#-- (c) https://github.com/MontiCore/monticore -->
<#import "SomeIPMacros.ftl" as m>
#include "<@m.mwIdent/>Adapter_${model.getEscapedCompName()}.h"

<@m.mwIdent/>Adapter_${model.getEscapedCompName()}::<@m.mwIdent/>Adapter_${model.getEscapedCompName()}() {

}

void <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::init(${model.getEscapedCompName()} *comp) {
    // Initialize component
	this->component = comp;


	// Intitialize Subscriber
	<#list model.getIncomingPorts() as sub>
		${sub.getName()}_Subscriber = vsomeip::runtime::get()->create_application("Subscriber ${sub.getName()}");
		if (!${sub.getName()}_Subscriber->init()) {
            std::cerr << "Couldn't initialize Subscriber ${sub.getName()}" << std::endl;
        }
		
		${sub.getName()}_Subscriber->register_availability_handler(${model.getServiceId(sub)}, ${model.getInstanceId(sub)}, std::bind(&SomeIPAdapter_${model.getEscapedCompName()}::on_availability_${sub.getName()}, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
		${sub.getName()}_Subscriber->request_service(${model.getServiceId(sub)}, ${model.getInstanceId(sub)});
		${sub.getName()}_Subscriber->register_message_handler(${model.getServiceId(sub)}, ${model.getInstanceId(sub)}, ${model.getMethodId(sub)}, std::bind(&SomeIPAdapter_${model.getEscapedCompName()}::on_message_${sub.getName()}, this, std::placeholders::_1));

		std::thread sender_${sub.getName()}(std::bind(&SomeIPAdapter_${model.getEscapedCompName()}::run_${sub.getName()}, this));
		${sub.getName()}_Subscriber->start();
    </#list>


	// Intitialize Publisher
	<#list model.getOutgoingPorts() as pub>
    	${pub.getName()}_Publisher = vsomeip::runtime::get()->create_application("Publisher ${pub.getName()}");
		if (!${pub.getName()}_Publisher->init()) {
            std::cerr << "Couldn't initialize Publisher ${pub.getName()}" << std::endl;
        }
    	${pub.getName()}_Publisher->offer_service(${model.getServiceId(pub)}, ${model.getInstanceId(pub)});
		${pub.getName()}_Publisher->start();
    </#list>
}

<#list model.getIncomingPorts() as sub>
void <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::run_${sub.getName()}() {
	std::unique_lock<std::mutex> its_lock(mutex_${sub.getName()});
	condition_${sub.getName()}.wait(its_lock);

	std::set<vsomeip::eventgroup_t> event_group;
	event_group.insert(${model.getEventGroupId(sub)});
	${sub.getName()}_Subscriber->request_event(${model.getServiceId(sub)}, ${model.getInstanceId(sub)}, ${model.getEventId(sub)}, event_group, true);
	${sub.getName()}_Subscriber->subscribe(${model.getServiceId(sub)}, ${model.getInstanceId(sub)}, ${model.getEventGroupId(sub)});
}

void <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::on_message_${sub.getName()}(const std::shared_ptr<vsomeip::message> &_message) {
	//read received message
    std::shared_ptr<vsomeip::payload> its_payload = _message->get_payload();
    vsomeip::length_t l = its_payload->get_length();
    double dataFromMessage = *((double*)its_payload->get_data());
	
	<#switch sub.getTypeReference().getName()>
		<#case "Q">
        // double
		component->${sub.getName()} = dataFromMessage;
        <#break>
        <#case "N">
        //int
		component->${sub.getName()} = (int) round(dataFromMessage);
        <#break>
        <#case "Z">
        //int
		component->${sub.getName()} = (int) round(dataFromMessage);
        <#break>
        <#case "B">
        //bool
		component->${sub.getName()} = (dataFromMessage > 1.0e-10);
        <#break>
        <#default>
        //error
    </#switch>

	//print data to std out
    std::cout << "SERVICE ${sub.getName()}: Received message from ["
			<< std::setw(4) << std::setfill('0') << std::hex << _message->get_client() << "/"
			<< std::setw(4) << std::setfill('0') << std::hex << _message->get_session() << "]: "
			<< dataFromMessage << std::endl;
}

void <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::on_availability_${sub.getName()}(vsomeip::service_t _service, vsomeip::instance_t _instance, bool _is_available) {
	std::cout << "Service ["
		<< std::setw(4) << std::setfill('0') << std::hex << _service << "." << _instance
		<< "] is "
		<< (_is_available ? "available." : "NOT available.")
		<< std::endl;
	condition_${sub.getName()}.notify_one();
}
</#list>


<#list model.getOutgoingPorts() as pub>
void <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::publish${pub.getName()}_Publisher()
{
    //Read data from component
	<#switch pub.getTypeReference().getName()>
        <#case "Q">
        // double
    	double d = component->${pub.getName()};
        <#break>
        <#case "N">
        //int
    	double d = 1.0 * component->${pub.getName()};
        <#break>
        <#case "Z">
        //int
    	double d = 1.0 * component->${pub.getName()};
        <#break>
        <#case "B">
        //bool
    	double d = component->${pub.getName()} ? 1.0 : 0.0;
        <#break>
        <#default>
        //error
    </#switch>

	const vsomeip::byte_t its_data[] = { (uint8_t) d };
	std::shared_ptr<vsomeip::payload> payload = vsomeip::runtime::get()->create_payload();
	payload->set_data(its_data, sizeof(its_data));

	//Publish
	std::set<vsomeip::eventgroup_t> event_group;
	event_group.insert(${model.getEventGroupId(pub)});
	${pub.getName()}_Publisher->offer_event(${model.getServiceId(pub)}, ${model.getInstanceId(pub)}, ${model.getEventId(pub)}, event_group, true);
	${pub.getName()}_Publisher->notify(${model.getServiceId(pub)}, ${model.getInstanceId(pub)}, ${model.getEventId(pub)}, payload);
}
</#list>


void <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::tick()
{
	<#list model.getOutgoingPorts() as pub>
    	publish${pub.getName()}_Publisher();
    </#list>
}

bool <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::hasReceivedNewData()
{
  return true;
}
