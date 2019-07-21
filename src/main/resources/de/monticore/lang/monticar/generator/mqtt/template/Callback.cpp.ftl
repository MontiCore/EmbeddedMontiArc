<#import "MqttMacros.ftl" as m>

#include "Callback.hpp"

Callback::Callback(client& cli, ${model.getEscapedCompName()}* comp, EMAPortInstanceSymbol port) : callback(), cli_(cli)
{
    comp_ = comp;
    port_ = port;
}

// Callback for when connected to broker
void Callback::connected(const string& cause)
{
    cout << "Connected" <<endl;
}

// Callback for when the connection is lost.
void Callback::connection_lost(const string& cause)
{
    cout << "\nConnection lost";
    if (!cause.empty())
        cout << ": " << cause << endl;
}

// Callback for when message is received
void Callback::message_arrived(const_message_ptr msg)
{
    cout << "Message received "<< msg->get_topic() << ": " << msg->get_payload_str() << endl;
    string::size_type sz;
    double value = std::stod (msg->get_payload_str(),&sz);
    comp_->port_.toString() = value;
}
