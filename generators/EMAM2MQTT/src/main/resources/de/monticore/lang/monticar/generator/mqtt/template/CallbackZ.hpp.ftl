<#-- (c) https://github.com/MontiCore/monticore -->
<#import "MqttMacros.ftl" as m>
/* (c) https://github.com/MontiCore/monticore */

#ifndef CallbackZ_hpp
#define CallbackZ_hpp

#include "${model.getEscapedCompName()}.h"
<@m.mwDefaultInclude/>
using namespace std;
using namespace <@m.smallIdent/>;

class CallbackZ : public virtual callback
{
    client& cli_;

public:
    CallbackZ(client& cli, int* port);

    void connected(const string& cause);

    void connection_lost(const string& cause);

    void message_arrived(const_message_ptr msg);

private:
    int* port_;

};
#endif /* CallbackZ_hpp */
