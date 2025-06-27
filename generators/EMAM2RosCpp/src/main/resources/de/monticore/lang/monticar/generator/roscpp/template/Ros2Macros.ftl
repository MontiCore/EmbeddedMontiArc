<#-- (c) https://github.com/MontiCore/monticore -->
/* (c) https://github.com/MontiCore/monticore */
<#macro mwIdent>Ros</#macro>
<#macro mwDefaultIncludes>
#include <rclcpp/rclcpp.hpp>
</#macro>

<#macro mwMsgInclude incl>
#include <${incl}>
</#macro>

<#macro fieldSubscriber type name>
    rclcpp::Subscription<${type}>::SharedPtr ${name};
</#macro>

<#macro fieldPublisher type name>
    rclcpp::Publisher<${type}>::SharedPtr ${name};
</#macro>

<#macro mwinit compname>
    char* tmp = strdup("");
    int i = 1;
    rclcpp::init(i, &tmp);
    auto node_handle = rclcpp::Node::make_shared("RosAdapter_${compname}");
</#macro>

<#macro mwstart>
    rclcpp::spin(node_handle);
</#macro>

<#macro initSubscriber sub compName>
    ${sub.getNameInTargetLanguage()} = node_handle->create_subscription<${sub.getTypeNameInTargetLanguage()}>("${sub.getTopicName()}", std::bind(&RosAdapter_${compName}::${sub.getMethodName()}, this, std::placeholders::_1));
    <#--${sub.getNameInTargetLanguage()} = node_handle.subscribe("${sub.getTopicName()}", 5, &RosAdapter_${compName}::${sub.getMethodName()}, this, ros::TransportHints().tcpNoDelay());-->
</#macro>

<#macro initPublisher pub>
    ${pub.getNameInTargetLanguage()} = node_handle->create_publisher<${pub.getTypeNameInTargetLanguage()}>("${pub.getTopicName()}");
<#--${pub.getNameInTargetLanguage()} = node_handle.advertise<${pub.getTypeNameInTargetLanguage()}>("${pub.getTopicName()}",5);-->
</#macro>

<#macro callback sub>
void ${sub.getMethodName()}(const ${sub.getTypeNameInTargetLanguage()}::SharedPtr msg){
    <#if !sub.isStructInterface() && !sub.isMatrixInterface() && sub.getRosConnectionSymbol().getMsgField().isPresent()>
    component->${sub.getPortNameInTargetLanguage()} = msg->${sub.getRosConnectionSymbol().getMsgField().get()};
    <#else>
       <#-- ${sub.getRos2SetStructInstruction()} -->
      
        <#if sub.isMatrixInterface()>
        ${sub.getRos2SetMatrixInstruction()}
        <#else>
        ${sub.getRos2SetStructInstruction()}
        </#if>

    </#if>
    ${sub.getMethodName()}_wasCalled = true;
}
</#macro>


<#macro publish pub>
    void publish${pub.getNameInTargetLanguage()}(){
    ${pub.getTypeNameInTargetLanguage()} tmpMsg;
    <#if !pub.isStructInterface() &&  pub.getRosConnectionSymbol().getMsgField().isPresent()>
        tmpMsg.${pub.getRosConnectionSymbol().getMsgField().get()} = component->${pub.getPortNameInTargetLanguage()};
    <#else>
        ${pub.getRos2SetStructInstruction()}
    </#if>
    ${pub.getNameInTargetLanguage()}->publish(tmpMsg);
    }
</#macro>
