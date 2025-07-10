/* (c) https://github.com/MontiCore/monticore */

#include "CallbackB.hpp"

CallbackB::CallbackB(client& cli, bool* port) : callback(), cli_(cli)
{
    port_ = port;
}

// Callback for when connected to broker
void CallbackB::connected(const string& cause)
{
    cout << "Connected" <<endl;
}

// Callback for when the connection is lost.
void CallbackB::connection_lost(const string& cause)
{
    cout << "\nConnection lost";
    if (!cause.empty())
        cout << ": " << cause << endl;
}

// Callback for when message is received
void CallbackB::message_arrived(const_message_ptr msg)
{
    cout << "Message received "<< msg->get_topic() << ": " << msg->get_payload_str() << endl;
    bool value = ((std::stod (msg->get_payload_str())) > 1.0e-10);
    *port_ = value;
}
