<#import "MqttMacros.ftl" as m>

#pragma once

#include "${model.getEscapedCompName()}.h"
<@m.mwDefaultIncludes/>

using namespace std;
using namespace <@m.smallIdent/>;

class <@m.mwIdent/>Adapter_${model.getEscapedCompName()} {
    
public:
    
    <@m.mwIdent/>Adapter_${model.getEscapedCompName()}();
    
    void init(${model.getEscapedCompName()}* comp);
    
    <#list model.getOutgoingPorts() as pub>
        void publish_echo_${pub.getName()}();
    </#list>
    
    
    void tick();
    
private:
    const string SERVER_ADDRESS = "tcp://localhost:1883";
    const string PUB_ID = "publisher_cpp";
    const string SUB_ID = "subscriber_cpp";
    const string TOPIC = "/clock";
    
    ${model.getEscapedCompName()}* component = nullptr;
    
    // Callbacks, subscribers
    <#list model.getIncomingPorts() as sub>
        Callback* _callback_${sub.getName()} = nullptr;
        client* _sub_${sub.getName()} = nullptr; 
    </#list>
    // Publishers
	<#list model.getOutgoingPorts() as pub>
        client* _pub_${pub.getName()} = nullptr; 
    </#list>
};
