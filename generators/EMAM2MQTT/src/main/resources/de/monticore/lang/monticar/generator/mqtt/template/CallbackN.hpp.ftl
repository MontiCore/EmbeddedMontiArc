<#-- (c) https://github.com/MontiCore/monticore -->
<#import "MqttMacros.ftl" as m>
/* (c) https://github.com/MontiCore/monticore */

#ifndef CallbackN_hpp
#define CallbackN_hpp

#include "${model.getEscapedCompName()}.h"
<@m.mwDefaultInclude/>
using namespace std;
using namespace <@m.smallIdent/>;

class CallbackN : public virtual callback
{
    client& cli_;

public:
    CallbackN(client& cli, int* port);

    void connected(const string& cause);

    void connection_lost(const string& cause);

    void message_arrived(const_message_ptr msg);

private:
    int* port_;

};
#endif /* CallbackN_hpp */
