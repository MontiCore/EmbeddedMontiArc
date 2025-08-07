<#-- (c) https://github.com/MontiCore/monticore -->
<#import "MqttMacros.ftl" as m>
/* (c) https://github.com/MontiCore/monticore */

#pragma once

#include "${model.getEscapedCompName()}.h"
#include "IAdapter_${model.getEscapedCompName()}.h"
<@m.mwDefaultIncludes/>

using namespace std;
using namespace <@m.smallIdent/>;

class <@m.mwIdent/>Adapter_${model.getEscapedCompName()} : public IAdapter_${model.getEscapedCompName()} {

public:

    <@m.mwIdent/>Adapter_${model.getEscapedCompName()}();

    void init(${model.getEscapedCompName()}* comp);

    <#list model.getOutgoingPorts() as pub>
    void publish_echo_${pub.getName()}();
    </#list>

    void tick();
    bool hasReceivedNewData();  

private:
    const string SERVER_ADDRESS = "tcp://localhost:1883";

    ${model.getEscapedCompName()}* component = nullptr;

    // Callbacks, subscribers
    <#list model.getIncomingPorts() as sub>
      <#switch  sub.getTypeReference().getName()>
        <#case "Q">
          CallbackQ* _callback_${sub.getName()} = nullptr;
        <#break>
        <#case "N">
          CallbackN* _callback_${sub.getName()} = nullptr;
        <#break>
        <#case "Z">
          CallbackZ* _callback_${sub.getName()} = nullptr;
        <#break>
        <#case "B">
          CallbackB* _callback_${sub.getName()} = nullptr;
        <#break>
      </#switch>
    client* _sub_${sub.getName()} = nullptr;
    </#list>
    // Publishers
	<#list model.getOutgoingPorts() as pub>
    client* _pub_${pub.getName()} = nullptr;
    </#list>
};
