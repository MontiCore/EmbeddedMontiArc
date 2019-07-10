<#import "SomeIPMacros.ftl" as m>
#pragma once
#include "${model.getEscapedCompName()}.h"
#include <iomanip>
#include <iostream>
#include <sstream>

#include <condition_variable>
#include <thread>

#include <vsomeip/vsomeip.hpp>

using namespace std;

class <@m.mwIdent/>Adapter_${model.getEscapedCompName()} {

public:

    <@m.mwIdent/>Adapter_${model.getEscapedCompName()}();

    void init(${model.getEscapedCompName()}* comp);

    void publish_echoPublisher();

    void tick();

    void on_message(const std::shared_ptr<vsomeip::message> &_response);

private:

    ${model.getEscapedCompName()}* component = nullptr;

	std::shared_ptr<vsomeip::application> _clockSubscriber;

	std::shared_ptr<vsomeip::application> _echoPublisher;
};
