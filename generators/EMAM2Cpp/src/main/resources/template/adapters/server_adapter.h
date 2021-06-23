#pragma once

#include <string>
#include <iostream>
#include <chrono>

#include "network.h"
#include "tcp_protocol.h"


using HRClock = std::chrono::high_resolution_clock;
using TimePoint = HRClock::time_point;
using duration = std::chrono::duration<double>;



enum TimeMode {
    REALTIME,
    MEASURED
};

void usage(char *app_name);
void simulation_session(int socket);

// Used as singleton
struct SimulationSession {
    TimeMode time_mode = MEASURED;
    Socket socket;
    bool is_json_comm = false;
    
    SimulationSession(int socket) : socket(socket) {}

    void run();
    void init(BinaryReader &init_packet);

    void run_cycle(BinaryReader &run_packet);
    void send_outputs();
    void send_socket_outputs(int port_id);
    void send_time(double time);
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
