
#ifndef Callback_hpp
#define Callback_hpp

#include "tests_a_compA"
#include "mqtt/client.h"
using namespace std;
using namespace mqtt;

class Callback : public virtual callback
{
    client& cli_;

public:
    Callback(client& cli, tests_a_compA* comp, EMAPortInstanceSymbol port);

    void connected(const string& cause);

    void connection_lost(const string& cause);

    void message_arrived(const_message_ptr msg);

private:
    tests_a_compA* comp_;

    EMAPortInstanceSymbol port_;

};
#endif /* Callback_hpp */
