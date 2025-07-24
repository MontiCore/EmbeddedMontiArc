/* (c) https://github.com/MontiCore/monticore */
#include "CallbackQ.hpp"

CallbackQ::CallbackQ(mqtt::client& cli, double* port) : mqtt::callback(), cli_(cli)
{
    port_ = port;
}

// Callback for when connected to broker
void CallbackQ::connected(const std::string& cause)
{
    cout << "Connected" <<endl;
}

// Callback for when the connection is lost.
void CallbackQ::connection_lost(const std::string& cause)
{
    cout << "\nConnection lost";
    if (!cause.empty())
        cout << ": " << cause << endl;
}

// Callback for when message is received
void CallbackQ::message_arrived(mqtt::const_message_ptr msg)
{
    cout << "Message received "<< msg->get_topic() << ": " << msg->get_payload_str() << endl;
    std::string::size_type sz;
    double value = std::stod (msg->get_payload_str(),&sz);
    *port_ = value;
}
