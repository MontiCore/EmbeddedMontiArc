/* (c) https://github.com/MontiCore/monticore */
//  Created by Georg Vinogradov on 28.05.19.
//

#include <iostream>
#include <string>
#include <thread>
#include <unistd.h> // Sleep on unix

#include "mqtt/client.h"

const std::string SERVER_ADDRESS = "tcp://localhost:1883";
const std::string PUB_ID = "publisher_cpp";
const std::string SUB_ID = "subscriber_cpp";
const std::string TOPIC = "topic/hello";
const std::string PAYLOAD = "265";
const int QOS = 1;

using namespace std;

class callback : public virtual mqtt::callback
{
    mqtt::client& cli_;
    
    // Callback for when connected
    void connected(const std::string& cause) {}
    
    // Callback for when the connection is lost.
    // This will initiate the attempt to manually reconnect.
    void connection_lost(const std::string& cause) {
        cout << "\nConnection lost";
        if (!cause.empty())
            cout << ": " << cause << endl;
    }
    
    // Callback for when message is received
    void message_arrived(mqtt::const_message_ptr msg) {
        cout << "Message received "<< msg->get_topic() << ": " << msg->get_payload_str() << endl;
    }
    
public:
    callback(mqtt::client& cli) : cli_(cli) {}
};

void publish()
{
    cout << "\nInitialzing publisher" << endl;
    
    // Publisher
    mqtt::client publisher(SERVER_ADDRESS, PUB_ID);
    mqtt::connect_options pubConnOpts;
    pubConnOpts.set_keep_alive_interval(20);
    pubConnOpts.set_clean_session(true);
    
    try {
        cout << "\nPublisher connects to broker..." << endl;
        publisher.connect(pubConnOpts);
        
        cout << "\nPublisher prepares message: "<< PAYLOAD << endl;
        auto pubmsg = mqtt::make_message(TOPIC, PAYLOAD);
        sleep(2);
        cout << "\nPublisher sends message: "<< PAYLOAD << endl;
        pubmsg->set_qos(QOS);
        
        publisher.publish(pubmsg);
        
        cout << "\nPublisher disconnects..." << endl;
        publisher.disconnect();
    }
    catch (const mqtt::exception& exc) {
        cerr << exc.what() << endl;
    }
}

void subscribe()
{
    cout << "\nInitialzing subscriber" << endl;
    
    //Subscriber
    mqtt::client subscriber(SERVER_ADDRESS, SUB_ID);
    callback cb(subscriber); // Subscriber needs callback
    subscriber.set_callback(cb);
    mqtt::connect_options subConnOpts;
    subConnOpts.set_keep_alive_interval(20);
    subConnOpts.set_clean_session(false);
    subConnOpts.set_automatic_reconnect(true);
    
    try {
        cout << "\nSubscriber connects to broker..." << endl;
        subscriber.connect(subConnOpts);
        
        cout << "\nSubscriber subscribes to topic: "<< TOPIC << endl;
        subscriber.subscribe(TOPIC, QOS);
    }
    catch (const mqtt::exception& exc) {
        cerr << exc.what() << endl;
    }
    
    while (tolower(cin.get()) != 'q'); // Listens for messages until user interrupts
}

int main(int argc, char* argv[])
{
    thread sub (subscribe);
    publish();
    sub.join();
    return 0;
}
