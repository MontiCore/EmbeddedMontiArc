#pragma once
#include <string>
#include <iostream>
#include <chrono>
#include "network.h"
#include "tcp_protocol.h"

#include "./${mainModelName}.h"

#define PROGRAM_INTERFACE_TYPE "dynamic"
#define PROGRAM_PORT_COUNT 12
extern const char* PROGRAM_INTERFACE;

void usage(char *app_name);


void simulation_session(int socket);
struct SimulationSession {
    TimeMode time_mode = MEASURED;
    Socket socket;
    ${mainModelName} program_instance;
    
    SimulationSession(int socket) : socket(socket) {}

    void run();
    void init(PacketReader &init_packet);

    void set_input(PacketReader &input_packet);
    void run_cycle(PacketReader &run_packet);
    void send_output(int port_id);
    void send_socket_outputs(int port_id);
    void send_time(double time);

    void init_program_data();
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
