#include "server_adapter.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <string.h>
#include <memory>
#include "json.h"
<#if useDDC>
#include "ddc_mode.h"
</#if>

#if defined _WIN32 || defined _WIN64
#define IS_WIN
#else
#endif

using namespace std;

bool running = true;
constexpr int BUFFER_SIZE = 4096;
char buffer[BUFFER_SIZE];

void signal_handler(int signal);





int main(int argc, char** argv) {
    // No need for interupt handling on windows => Ctrl+C is handled by the OS
#ifndef IS_WIN
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGTERM, &sa, NULL) == -1) {
        perror("sigaction");
        exit(1);
    }
    if (sigaction(SIGINT, &sa, NULL) == -1) {
        perror("sigaction");
        exit(1);
    }
#endif

    if (argc == 3) {
        string cmd = argv[1];
        if (cmd.compare("server") == 0) {
            single_session_server(argv[2], simulation_session, running);
            return 0;
        }
<#if useDDC>
        else if (cmd.compare("ddc") == 0) {
            return ddc_mode(argv[2], running);
        }
</#if>
    }
    
    usage(argv[0]);
    return 0;
}

void signal_handler(int signal) {
    running = false;
}

void usage(char *app_name) {
    cout << "Usage:" << endl;
    cout << "  " << app_name << " server <port>" << endl;
<#if useDDC>
    cout << "or" << endl;
    cout << "  " << app_name << " ddc <reference_id>" << endl;
</#if>
}









/*
    SERVER MODE
*/

void simulation_session(int socket) {
    auto session = unique_ptr<SimulationSession>(new SimulationSession(socket));
    session->run();
}

void SimulationSession::run() {
    try {
        bool reset = false;
        while(true) {
            reset = false;
            PacketReader packet_in(buffer, BUFFER_SIZE, socket);
            if (packet_in.packet.id == PACKET_INIT){
                init(packet_in);
            } else {
                throw AdapterException("Expected INIT packet, but got: id=" + to_string(packet_in.packet.id) + " length=" + to_string(packet_in.packet.size));
            }

            if (time_mode == REALTIME) {

            } else if (time_mode == MEASURED) {
                PacketReader packet_in(buffer, BUFFER_SIZE, socket);
                while (packet_in.packet.id >= 0){
                    switch (packet_in.packet.id) {
                        case PACKET_END:
                            cout << "Ending connection." << endl;
                        return;
                        case PACKET_INIT:
                            // Allow re-init
                            reset = true;
                        break;
                        case PACKET_INPUT:
                            set_input(packet_in);
                            break;
                        case PACKET_RUN_CYCLE:
                            run_cycle(packet_in);
                            break;
                        case PACKET_REF_ID: // Ignore in direct server mode
                            break;
                        default:
                            throw AdapterException("Unknown packet: id=" + to_string(packet_in.packet.id) + " length=" + to_string(packet_in.packet.size));
                    }
                    if (reset || !running) {
                        break;
                    }
                    packet_in = PacketReader(buffer, BUFFER_SIZE, socket);
                }
            }
        }
    } catch (const std::exception &e) {
        auto msg = e.what();
        cerr << "Error: " << msg << endl;

        // Try sending the error to the simulator
        try {
            PacketWriter packet(buffer, BUFFER_SIZE, PACKET_ERROR);
            packet.write_str(msg);
            packet.send(socket);
        } catch (const std::exception &e2) {
            cerr << "Could not send error to simulator:\n\t" << e2.what() << endl;
        }
    }
}



void SimulationSession::init(PacketReader &init_packet) {
    auto mode = init_packet.read_str();
    if (mode.compare("measured") == 0) time_mode = MEASURED;
    else if (mode.compare("realtime") == 0) time_mode = REALTIME;
    else throw AdapterException("Unknown TimeMode: " + mode);

    cout << "Running in time mode '" << mode << "'." << endl;

    program_instance.init();
    init_program_data();
    cout << "Initialized Program." << endl;
    
    // Sending Program Interface
    PacketWriter interface_packet(buffer, BUFFER_SIZE, PACKET_INTERFACE);
    interface_packet.write_str(PROGRAM_INTERFACE);
    interface_packet.send(socket);

    cout << "Adapter initialization complete. Ready to run the program." << endl;
}

void SimulationSession::set_input(PacketReader &input_packet) {
    auto port_id = input_packet.read_u16();

    switch (port_id) {
<#list setInputCases as setInputCase>${setInputCase}</#list>
        default:
            cerr << "Invalid INPUT port ID: '"<< port_id << "'" << endl;
            throw exception();
    }
}

void SimulationSession::run_cycle(PacketReader &run_packet) {
    auto delta_secs = run_packet.read_f64();


    auto t1 = HRClock::now();
    // TODO set a "delta_sec" port if it exists
    program_instance.execute();
    auto t2 = HRClock::now();

    duration time = t2 - t1;

    // Send outputs
<#list sendOutputCalls as sendOutputCall>
    ${sendOutputCall}
</#list>

    // Send exec time
    send_time(time.count());
}

void SimulationSession::send_output(int port_id){
    // Write packet id
    PacketWriter packet(buffer, BUFFER_SIZE, PACKET_OUTPUT);

    // Write port id
    packet.write_u16(port_id);
    
    // Write port data
    switch (port_id) {
<#list sendOutputCases as sendOutputCase>${sendOutputCase}</#list>
        default:
            cerr << "Invalid output port ID: '"<< port_id << "'" << endl;
            throw exception();
    }

    // Send packet
    packet.send(socket);
}
void SimulationSession::send_time(double time) {
    PacketWriter writer(buffer, BUFFER_SIZE, PACKET_TIME);
    writer.write_f64(time);
    writer.send(socket);
}


// TODO: Should be handled in the component init() code
void SimulationSession::init_program_data() {
<#list initDataCalls as initDataCall>${initDataCall}</#list>
}




/*
    INTERFACE STRING
*/

const char* PROGRAM_INTERFACE = R"(
${interfaceDescription}
)";