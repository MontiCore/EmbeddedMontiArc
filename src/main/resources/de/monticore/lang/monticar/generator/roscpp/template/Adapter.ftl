<#--@formatter:off-->
<#if model.getMw() == "ROS2">
    <#import "Ros2Macros.ftl" as m>
<#elseif model.getMw() == "ROS">
    <#import "RosMacros.ftl" as m>
</#if>
#pragma once
#include "IAdapter_${model.getCompName()}.h"
#include "${model.getCompName()}.h"
<@m.mwDefaultIncludes/>
<#list model.getIncludes() as incl>
    <@m.mwMsgInclude incl=incl/>
</#list>

class <@m.mwIdent/>Adapter_${model.getCompName()}: public IAdapter_${model.getCompName()}{
<#list model.getGenerics() as gen>
${gen}
</#list>

${model.getCompName()}* component;
<#list model.getSubscribers() as sub>
<@m.fieldSubscriber type="${sub.getTypeNameInTargetLanguage()}" name="${sub.getNameInTargetLanguage()}"/>
</#list>
<#list model.getPublishers() as pub>
<@m.fieldPublisher type="${pub.getTypeNameInTargetLanguage()}" name="${pub.getNameInTargetLanguage()}"/>
</#list>

public:
<@m.mwIdent/>Adapter_${model.getCompName()}(){

}

void init(${model.getCompName()}* comp){
    this->component = comp;
     <@m.mwinit compname="${model.getCompName()}"/>

    <#list model.getSubscribers() as sub>
        <@m.initSubscriber sub=sub compName=model.getCompName()/>
    </#list>

    <#list model.getPublishers() as pub>
        <@m.initPublisher pub=pub />
    </#list>

    <@m.mwstart/>
}

<#list model.getSubscribers() as sub>
    <@m.callback sub=sub/>
</#list>

<#list model.getPublishers() as pub>
    <@m.publish pub=pub/>
</#list>

void tick(){
<#list model.getPublishers() as pub>
    ${pub.getMethodName()}();
</#list>
}

};