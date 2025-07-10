<#-- (c) https://github.com/MontiCore/monticore -->
<#import "MqttMacros.ftl" as m>
/* (c) https://github.com/MontiCore/monticore */

#ifndef CallbackB_hpp
#define CallbackB_hpp

#include "${model.getEscapedCompName()}.h"
<@m.mwDefaultInclude/>
using namespace std;
using namespace <@m.smallIdent/>;

class CallbackB : public virtual callback
{
    client& cli_;

public:
    CallbackB(client& cli, bool* port);

    void connected(const string& cause);

    void connection_lost(const string& cause);

    void message_arrived(const_message_ptr msg);

private:
    bool* port_;

};
#endif /* CallbackB_hpp */
