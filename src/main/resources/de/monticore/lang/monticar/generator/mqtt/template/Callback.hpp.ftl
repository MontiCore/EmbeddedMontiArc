<#-- (c) https://github.com/MontiCore/monticore -->
<#import "MqttMacros.ftl" as m>
/* (c) https://github.com/MontiCore/monticore */

#ifndef Callback_hpp
#define Callback_hpp

#include "${model.getEscapedCompName()}"
<@m.mwDefaultInclude/>
using namespace std;
using namespace <@m.smallIdent/>;

class Callback : public virtual callback
{
    client& cli_;

public:
    Callback(client& cli, double* port);

    void connected(const string& cause);

    void connection_lost(const string& cause);

    void message_arrived(const_message_ptr msg);

private:
    double* port_;

};
#endif /* Callback_hpp */
