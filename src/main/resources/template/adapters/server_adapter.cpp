#include "server_adapter.h"

#include <signal.h>
#include <string.h>
#include <memory>

#include "json.h"
#include "program.h"
#include "err_out.h"
#include "standard_err_out.h"

#if defined _WIN32 || defined _WIN64
#define IS_WIN
#else
#endif

using namespace std;

bool running = true;

void signal_handler(int signal);
int get_socket_id(PacketReader &input_packet, int max_count);



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

    // Use the "standard_err_out.h" functions for errors and messages from the autopilot
    ERR_OUT_set_functions(ERR_OUT_standard_throw_error, ERR_OUT_standard_print_cout, ERR_OUT_standard_print_cerr);

    if (argc == 2) {
        session_server(argv[1], simulation_session, running);
        return 0;
    }
    
    usage(argv[0]);
    return 0;
}

void signal_handler(int signal) {
    running = false;
}

void usage(char *app_name) {
    cout << "Usage:" << endl;
    cout << "  " << app_name << " <port>" << endl;
}









/*
    SERVER MODE
*/

void simulation_session(int socket) {
    auto session = unique_ptr<SimulationSession>(new SimulationSession(socket));
    session->run();
    close_sock(socket);
}

void SimulationSession::run() {
    PacketReader packet_in(autopilot::buffer, socket);
    try {
        bool reset = false;
        while(true) {
            reset = false;
            packet_in.receive();
            if (packet_in.id == PACKET_INIT){
                auto br = packet_in.getReader();
                init(br);
            } else {
                throw AdapterException("Expected INIT packet, but got: id=" + to_string(packet_in.id) + " length=" + to_string(packet_in.size));
            }

            if (time_mode == REALTIME) {

            } else if (time_mode == MEASURED) {
                PacketReader packet_in(autopilot::buffer, socket);
                packet_in.receive();
                while (packet_in.id >= 0) {
                    switch (packet_in.id) {
                        case PACKET_END:
                            cout << "Ending connection." << endl;
                        return;
                        case PACKET_INIT:
                            // Allow re-init
                            reset = true;
                        break;
                        case PACKET_INPUT_BINARY: {
                            is_json_comm = false;
                            auto br = packet_in.getReader();
                            autopilot::set_port_binary(br.read_u16(), br);
                        } break;
                        case PACKET_INPUT_JSON:
                            is_json_comm = true;
                            autopilot::set_port_json(packet_in.getReader().read_u16(), packet_in.getPayload()+2);
                            break;
                        case PACKET_RUN_CYCLE: {
                            auto br = packet_in.getReader();
                            run_cycle(br);
                        } break;
                        case PACKET_REF_ID: // Ignore in direct server mode
                            break;
                        default:
                            throw AdapterException("Unknown packet: id=" + to_string(packet_in.id) + " length=" + to_string(packet_in.size));
                    }
                    if (reset || !running) {
                        break;
                    }
                    packet_in.receive();
                }
            }
        }
    } catch (const std::exception &e) {
        auto msg = e.what();
        cerr << "Error: " << msg << endl;

        // Try sending the error to the simulator
        try {
            PacketWriter packet(autopilot::buffer, PACKET_ERROR);
            packet.bw.write_str(msg);
            packet.send(socket);
        } catch (const std::exception &e2) {
            cerr << "Could not send error to simulator:\n\t" << e2.what() << endl;
        }
    }
}



void SimulationSession::init(BinaryReader &init_packet) {
    auto mode = init_packet.read_str();
    if (mode.compare("measured") == 0) time_mode = MEASURED;
    else if (mode.compare("realtime") == 0) time_mode = REALTIME;
    else throw AdapterException("Unknown TimeMode: " + mode);

    cout << "Running in time mode '" << mode << "'." << endl;

    autopilot::init();
    cout << "Initialized Program." << endl;
    
    // Sending Program Interface
    PacketWriter interface_packet(autopilot::buffer, PACKET_INTERFACE);
    interface_packet.bw.write_str(autopilot::PROGRAM_INTERFACE);
    interface_packet.send(socket);

    cout << "Adapter initialization complete. Ready to run the program." << endl;
}



void SimulationSession::run_cycle(BinaryReader &run_packet) {
    auto delta_secs = run_packet.read_f64();


    auto t1 = HRClock::now();
    // TODO set a "delta_sec" port if it exists
    autopilot::execute(delta_secs);
    auto t2 = HRClock::now();
    
    send_outputs();

    // Send exec time
    duration time = t2 - t1;
    send_time(time.count());
}

void SimulationSession::send_outputs() {
    for (int i = 0; i < autopilot::PORT_COUNT; ++i) {
        if (autopilot::IS_OUTPUT[i]) {
            do {
                PacketWriter packet(autopilot::buffer, is_json_comm ? PACKET_OUTPUT_JSON : PACKET_OUTPUT_BINARY); // Packet ID
                packet.bw.write_u16(i); // Port ID
                if (is_json_comm) {
                    JsonWriter writer {autopilot::buffer};
                    autopilot::get_port_json(i, writer); // Writes to 'autopilot::buffer'
                } else {
                    autopilot::get_port_binary(i, packet.bw);
                }
                if (packet.buffer.position() == 5) break;
                packet.send(socket);
                if (!autopilot::IS_SOCKET[i]) break;
            } while(true);
        }
    }
}

void SimulationSession::send_time(double time) {
    PacketWriter writer(autopilot::buffer, PACKET_TIME);
    writer.bw.write_f64(time);
    writer.send(socket);
}


/*
    INTERFACE STRING
*/
