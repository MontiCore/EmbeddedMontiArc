#pragma once
#include "autopilot.h"
//#include "program_interface.h"
#include <string>
#include <iostream>
#include <chrono>
#include "network.h"
#include "tcp_protocol.h"

#define PROGRAM_INTERFACE_TYPE "dynamic"
#define PROGRAM_PORT_COUNT 12
extern const char* PROGRAM_INTERFACE;

void usage(char *app_name);


void simulation_session(int socket);
struct SimulationSession {
    TimeMode time_mode = MEASURED;
    Socket socket;
    Autopilot autopilot;
    
    SimulationSession(int socket) : socket(socket) {}

    void run();
    void init(PacketReader &init_packet);

    void set_input(PacketReader &input_packet);
    void run_cycle(PacketReader &run_packet);
    void send_output(int port_id);
    void send_queue(int port_id);
    void send_time(double time);
    void send_ping();
};



class AdapterException : public std::exception {
    std::string description;
public:
    AdapterException(const std::string& description) : description("Adapter Exception:\n\t" + description) {}
    virtual const char* what() const throw()
    {
        return description.c_str();
    }
};
