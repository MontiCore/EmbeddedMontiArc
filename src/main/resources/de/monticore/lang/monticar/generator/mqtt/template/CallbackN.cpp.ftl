<#-- (c) https://github.com/MontiCore/monticore -->
<#import "MqttMacros.ftl" as m>
/* (c) https://github.com/MontiCore/monticore */

#include "CallbackN.hpp"

CallbackN::CallbackN(client& cli, int* port) : callback(), cli_(cli)
{
    port_ = port;
}

// Callback for when connected to broker
void CallbackN::connected(const string& cause)
{
    cout << "Connected" <<endl;
}

// Callback for when the connection is lost.
void CallbackN::connection_lost(const string& cause)
{
    cout << "\nConnection lost";
    if (!cause.empty())
        cout << ": " << cause << endl;
}

// Callback for when message is received
void CallbackN::message_arrived(const_message_ptr msg)
{
    cout << "Message received "<< msg->get_topic() << ": " << msg->get_payload_str() << endl;
    int value = std::stoi (msg->get_payload_str());
    *port_ = value;
}
