<#-- Creating header file for the adapter -->

<#-- We dont need if else case as with ROS/ROS2 - import MqttMacros.ftl directly -->

<#import "MqttMacros.ftl" as m>

#pragma once

<#-- Include IAdapter of component and component, that was set in a model -->

#include "IAdapter_${model.getCompName()}.h" 
#include "${model.getCompName()}.h"

<#-- Import the list of addtional includes from model -->

<@m.mwDefaultIncludes/>
<#list model.getIncludes() as incl>
    <@m.mwMsgInclude incl=incl/>
</#list>

<#-- Construct class name and generics from model -->

class <@m.mwIdent/>Adapter_${model.getCompName()}: public IAdapter_${model.getCompName()}{
<#list model.getGenerics() as gen>
${gen}
</#list>

<#-- Create pointers to component, subscribers and publishers instances -->

${model.getCompName()}* component;
<#list model.getSubscribers() as sub>
<@m.fieldSubscriber type="${sub.getTypeNameInTargetLanguage()}" name="${sub.getNameInTargetLanguage()}"/>
</#list>
<#list model.getPublishers() as pub>
<@m.fieldPublisher type="${pub.getTypeNameInTargetLanguage()}" name="${pub.getNameInTargetLanguage()}"/>
</#list>

<#-- Describing public methods, attributes -->
<#-- Since our implementations are not in header files, we also need to generate a .cpp -->

public:

<#-- To be continued -->

};
