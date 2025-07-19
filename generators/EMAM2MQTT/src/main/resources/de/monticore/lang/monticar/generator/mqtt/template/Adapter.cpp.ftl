<#-- (c) https://github.com/MontiCore/monticore -->
<#import "MqttMacros.ftl" as m>
/* (c) https://github.com/MontiCore/monticore */

#include "<@m.mwIdent/>Adapter_${model.getEscapedCompName()}.h"

<@m.mwIdent/>Adapter_${model.getEscapedCompName()}::<@m.mwIdent/>Adapter_${model.getEscapedCompName()}()
{

}

void <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::init(${model.getEscapedCompName()} *comp)
{
    // Initialize component
    this->component = comp;

    // Initialize connect options
    connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);

    // Intitialize callbacks, subscribers and publishers
    <#list model.getIncomingPorts() as sub>
      <#switch sub.getTypeReference().getName()>
        <#case "Q">
          _sub_${sub.getName()} = new client(SERVER_ADDRESS, "${sub.getName()}");
          _callback_${sub.getName()} = new CallbackQ(*_sub_${sub.getName()}, &(component->${sub.getName()}));
        <#break>
        <#case "N">
          _sub_${sub.getName()} = new client(SERVER_ADDRESS, "${sub.getName()}");
          _callback_${sub.getName()} = new CallbackN(*_sub_${sub.getName()}, &(component->${sub.getName()}));
        <#break>
        <#case "Z">
          _sub_${sub.getName()} = new client(SERVER_ADDRESS, "${sub.getName()}");
          _callback_${sub.getName()} = new CallbackZ(*_sub_${sub.getName()}, &(component->${sub.getName()}));
        <#break>
        <#case "B">
          _sub_${sub.getName()} = new client(SERVER_ADDRESS, "${sub.getName()}");
          _callback_${sub.getName()} = new CallbackB(*_sub_${sub.getName()}, &(component->${sub.getName()}));
        <#break>
        <#default>
          cerr << "Not a valid input port type" << endl;
      </#switch>
    </#list>

	<#list model.getOutgoingPorts() as pub>
    _pub_${pub.getName()} = new client(SERVER_ADDRESS, "${pub.getName()}");
    </#list>

    // Connect subscribers, publishers and subscribe to the topics
    try {
    	 <#list model.getIncomingPorts() as sub>
    	_sub_${sub.getName()}->set_callback(*_callback_${sub.getName()});
    	_sub_${sub.getName()}->connect(connOpts);
    	_sub_${sub.getName()}->subscribe("${model.getTopic(sub)}", 1);
    	</#list>
    	<#list model.getOutgoingPorts() as pub>
        _pub_${pub.getName()}->connect(connOpts);
   		</#list>

    } catch (const mqtt::exception& exc) {
        cerr << exc.what() << endl;
    }

}

<#list model.getOutgoingPorts() as pub>
void <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::publish_echo_${pub.getName()}()
{
  <#switch pub.getTypeReference().getName()>
    <#case "Q">
      string value = to_string(component->${pub.getName()});
    <#break>
    <#case "N">
      string value = to_string(component->${pub.getName()});
    <#break>
    <#case "Z">
      string value = to_string(component->${pub.getName()});
    <#break>
    <#case "B">
      string value = to_string(component->${pub.getName()} ? 1.0 : 0.0);
    <#break>
    <#default>
      cerr << "Not a valid output port type" << endl;
  </#switch>
	auto pubmsg = make_message("${model.getTopic(pub)}", value);

	try {
		_pub_${pub.getName()}->publish(pubmsg);

	}
	catch (const mqtt::exception& exc) {
	    cerr << exc.to_string() << endl;
	}
}
</#list>


void <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::tick()
{
	<#list model.getOutgoingPorts() as pub>
    publish_echo_${pub.getName()}();
    </#list>
}

bool <@m.mwIdent/>Adapter_${model.getEscapedCompName()}::hasReceivedNewData()
{
  return true;
}
