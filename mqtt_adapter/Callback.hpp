#ifndef Callback_hpp
#define Callback_hpp

#include "tests_a_compA.h"
#include "mqtt/client.h"

using namespace std;

class Callback : public virtual mqtt::callback
{
    mqtt::client& cli_;
    
public:
    Callback(mqtt::client& cli, tests_a_compA* comp);
    
    void connected(const std::string& cause);
    
    void connection_lost(const std::string& cause);
    
    void message_arrived(mqtt::const_message_ptr msg);
    
private:
    tests_a_compA* comp_;
    
};

#endif /* Callback_hpp */
