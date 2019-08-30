<#-- (c) https://github.com/MontiCore/monticore -->
<#import "MqttMacros.ftl" as m>
/* (c) https://github.com/MontiCore/monticore */

#ifndef CallbackQ_hpp
#define CallbackQ_hpp

#include "${model.getEscapedCompName()}.h"
<@m.mwDefaultInclude/>
using namespace std;
using namespace <@m.smallIdent/>;

class CallbackQ : public virtual callback
{
    client& cli_;

public:
    CallbackQ(client& cli, double* port);

    void connected(const string& cause);

    void connection_lost(const string& cause);

    void message_arrived(const_message_ptr msg);

private:
    double* port_;

};
#endif /* CallbackQ_hpp */
