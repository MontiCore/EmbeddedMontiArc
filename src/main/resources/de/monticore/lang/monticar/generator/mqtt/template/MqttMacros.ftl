<#macro mwIdent>Mqtt</#macro>

<#macro smallIdent>mqtt</#macro>

<#macro mwDefaultIncludes>
#include "mqtt/client.h"
</#macro>

<#macro mwMsgInclude incl>
#include <${incl}>
</#macro>

<#macro fieldSubscriber name>
mqtt::client* ${name};
</#macro>

<#macro fieldPublisher name>
mqtt::client* ${name};
</#macro>




<#macro mwinit compname>
    char* tmp = strdup("");
    int i = 0;
    ros::init(i, &tmp, "RosAdapter_${compname}_node");
    ros::NodeHandle node_handle = ros::NodeHandle();
</#macro>

<#macro mwstart>
    ros::spin();
</#macro>

<#macro initSubscriber sub compName>
    ${sub.getNameInTargetLanguage()} = node_handle.subscribe("${sub.getTopicName()}", 5, &RosAdapter_${compName}::${sub.getMethodName()}, this, ros::TransportHints().tcpNoDelay());
</#macro>

<#macro initPublisher pub>
    ${pub.getNameInTargetLanguage()} = node_handle.advertise<${pub.getTypeNameInTargetLanguage()}>("${pub.getTopicName()}",5);
</#macro>

<#macro callback sub>
void ${sub.getMethodName()}(const ${sub.getTypeNameInTargetLanguage()}::ConstPtr& msg){
    <#if !sub.isStructInterface() && sub.getRosConnectionSymbol().getMsgField().isPresent()>
    component->${sub.getPortNameInTargetLanguage()} = msg->${sub.getRosConnectionSymbol().getMsgField().get()};
    <#else>
    ${sub.getRosSetStructInstruction()}
    </#if>
}
</#macro>


<#macro publish pub>
void publish${pub.getNameInTargetLanguage()}(){
    ${pub.getTypeNameInTargetLanguage()} tmpMsg;
    <#if !pub.isStructInterface() &&  pub.getRosConnectionSymbol().getMsgField().isPresent()>
    tmpMsg.${pub.getRosConnectionSymbol().getMsgField().get()} = component->${pub.getPortNameInTargetLanguage()};
    <#else>
    ${pub.getRosSetStructInstruction()}
    </#if>
    ${pub.getNameInTargetLanguage()}.publish(tmpMsg);
}
</#macro>
