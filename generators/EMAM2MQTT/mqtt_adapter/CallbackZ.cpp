/* (c) https://github.com/MontiCore/monticore */
#include "CallbackZ.hpp"

CallbackZ::CallbackZ(mqtt::client& cli, int* port) : mqtt::callback(), cli_(cli)
{
    port_ = port;
}

// Callback for when connected to broker
void CallbackZ::connected(const std::string& cause)
{
    cout << "Connected" <<endl;
}

// Callback for when the connection is lost.
void CallbackZ::connection_lost(const std::string& cause)
{
    cout << "\nConnection lost";
    if (!cause.empty())
        cout << ": " << cause << endl;
}

// Callback for when message is received
void CallbackZ::message_arrived(mqtt::const_message_ptr msg)
{
    cout << "Message received "<< msg->get_topic() << ": " << msg->get_payload_str() << endl;
    int value = std::stoi (msg->get_payload_str());
    *port_ = value;
}
