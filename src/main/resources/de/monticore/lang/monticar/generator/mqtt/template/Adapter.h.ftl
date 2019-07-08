#pragma once

#include "${model.getEscapedCompName()}.h"
<@m.mwDefaultIncludes/>
<#list model.getIncludes() as incl>
    <@m.mwMsgInclude incl=incl/>
</#list>

using namespace std;

class <@m.mwIdent/>Adapter_${model.getEscapedCompName()} {
    
public:
    
    <@m.mwIdent/>Adapter_${model.getEscapedCompName()}();
    
    void init(${model.getEscapedCompName()}* comp);
    
    void publish_echoPublisher();
    
    void tick();
    
private:
    const string SERVER_ADDRESS = "${model.getServerAddress()}";
    const string PUB_ID = "publisher_cpp";
    const string SUB_ID = "subscriber_cpp";
    const string TOPIC = "/clock";
    
    ${model.getEscapedCompName()}* component = nullptr;
    Callback* _callback = nullptr;
    <@m.fieldSubscriber name="${model.getSubscriberName()}"/> = nullptr;
    <@m.fieldSubscriber name="${model.getPublisherName()}"/> = nullptr;
};
