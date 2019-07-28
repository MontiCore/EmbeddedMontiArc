#include "Callback.hpp"

Callback::Callback(mqtt::client& cli, tests_a_compA* comp) : mqtt::callback(), cli_(cli)
{
    comp_ = comp;
}

// Callback for when connected to broker
void Callback::connected(const std::string& cause)
{
    cout << "Connected" <<endl; 
}

// Callback for when the connection is lost.
void Callback::connection_lost(const std::string& cause)
{
    cout << "\nConnection lost";
    if (!cause.empty())
        cout << ": " << cause << endl;
}

// Callback for when message is received
void Callback::message_arrived(mqtt::const_message_ptr msg)
{
    cout << "Message received "<< msg->get_topic() << ": " << msg->get_payload_str() << endl;
    std::string::size_type sz;
    double value = std::stod (msg->get_payload_str(),&sz);
    comp_->mqttIn = value;
}



