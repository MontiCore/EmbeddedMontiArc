#include "program_adapter.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <string.h>
#include <memory>
#include <ctime>
#include <cmath>
#include "json.h"

using namespace std;


#if defined _WIN32 || defined _WIN64
#define IS_WIN
#else
#endif





bool running = true;
constexpr int BUFFER_SIZE = 4096;
char buffer[BUFFER_SIZE];

void signal_handler(int signal);





int main(int argc, char** argv) {
    srand(time(nullptr));
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
            if (packet_in.packet.id == PACKET_PING) {
                // Send back PING packet
                send_ping();
                cout << "Received PING request." << endl;
                continue;
            } else if (packet_in.packet.id == PACKET_INIT){
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
                        case PACKET_PING:
                            send_ping();
                            cout << "Received PING request." << endl;
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

    autopilot.init();
    cout << "Initialized Autopilot." << endl;
    
    // Sending Program Interface
    PacketWriter interface_packet(buffer, BUFFER_SIZE, PACKET_INTERFACE);
    interface_packet.write_str(PROGRAM_INTERFACE);
    interface_packet.send(socket);

    cout << "Adapter initialization complete. Ready to run the program." << endl;
}

void SimulationSession::set_input(PacketReader &input_packet) {
    auto port_id = input_packet.read_u16();

    switch (port_id) {
        case 0: // true_velocity
            autopilot.true_velocity = input_packet.read_f64();
            break;
        case 1: // true_position
            autopilot.true_position.x = input_packet.read_f64();
            autopilot.true_position.y = input_packet.read_f64();
            break;
        case 2: // true_compass
            autopilot.true_compass = input_packet.read_f64();
            break;
        case 3: // trajectory_length
            autopilot.trajectory_x__size = input_packet.read_u32();
            autopilot.trajectory_y__size = autopilot.trajectory_x__size;
            break;
        case 4: { // trajectory_x
            auto length = input_packet.read_u16();
            for (auto i = 0; i < length; ++i) {
                autopilot.trajectory_x[i] = input_packet.read_f64();
            }
        } break;
        case 5: { // trajectory_y
            auto length = input_packet.read_u16();
            for (auto i = 0; i < length; ++i) {
                autopilot.trajectory_y[i] = input_packet.read_f64();
            }
        } break;
        case 6: // steering
            break;
        case 7: // gas
            break;
        case 8: // braking
            break;
        case 12: { // net_sock1
            SimplePacket1 p1;
            p1.addr = input_packet.read_str();
            p1.payload = (int)input_packet.read_u32();
            autopilot.netsock1_in.push(p1);
        } break;
        case 13: { // net_sock2
            SimplePacket2 p2;
            p2.addr = input_packet.read_str();
            p2.payload = input_packet.read_f64();
            autopilot.netsock2_in.push(p2);
        } break;
        default:
            cerr << "Invalid INPUT port ID: '"<< port_id << "'" << endl;
            throw exception();
    }
}

void SimulationSession::run_cycle(PacketReader &run_packet) {
    auto delta_secs = run_packet.read_f64();


    auto t1 = HRClock::now();
    autopilot.execute(delta_secs);
    auto t2 = HRClock::now();

    duration time = t2 - t1;

    // Send outputs
    send_output(9);
    send_output(10);
    send_output(11);
    send_queue(12);
    send_queue(13);

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
        case 9: // set_steering
            packet.write_f64(autopilot.set_steering);
            break;
        case 10: // set_gas
            packet.write_f64(autopilot.set_gas);
            break;
        case 11: // set_braking
            packet.write_f64(autopilot.set_braking);
            break;
        default:
            cerr << "Invalid output port ID: '"<< port_id << "'" << endl;
            throw exception();
    }

    // Send packet
    packet.send(socket);
}

void SimulationSession::send_queue(int port_id){
    switch (port_id) {
        case 12: { // net_sock1
            while (!autopilot.netsock1_out.empty()) {
                
                // Write packet id
                PacketWriter packet(buffer, BUFFER_SIZE, PACKET_OUTPUT);
                
                // Write port id
                packet.write_u16(port_id);
                
                // Write port data
                auto p1 = autopilot.netsock1_out.front();
                packet.write_str(p1.addr);
                packet.write_u32(p1.payload);
                
                // Send packet
                packet.send(socket);
                
                autopilot.netsock1_out.pop();
            }
        } break;
        case 13: { // net_sock2
            while (!autopilot.netsock2_out.empty()) {
                
                // Write packet id
                PacketWriter packet(buffer, BUFFER_SIZE, PACKET_OUTPUT);
                
                // Write port id
                packet.write_u16(port_id);
                
                // Write port data
                auto p2 = autopilot.netsock2_out.front();
                packet.write_str(p2.addr);
                packet.write_f64(p2.payload);
                
                // Send packet
                packet.send(socket);
                
                autopilot.netsock2_out.pop();
            }
        } break;
        default:
            cerr << "Invalid output port ID: '"<< port_id << "'" << endl;
            throw exception();
    }
}

void SimulationSession::send_time(double time) {
    PacketWriter writer(buffer, BUFFER_SIZE, PACKET_TIME);
    writer.write_f64(time);
    writer.send(socket);
}

void SimulationSession::send_ping() {
    PacketWriter writer(buffer, BUFFER_SIZE, PACKET_PING);
    writer.send(socket);
}















/*
    INTERFACE STRING
*/

const char* PROGRAM_INTERFACE = R"(
{
    "name":"basic_interface",
    "version":"1.0",
    "ports":[
        {"name":"true_velocity","data_type":{"type":"basic","base_type":"Q"},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"true_position","data_type":{"type":"basic","base_type":"vec2"},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"true_compass","data_type":{"type":"basic","base_type":"Q"},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"trajectory_length","data_type":{"type":"basic","base_type":"N"},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"trajectory_x","data_type":{"type":"vector","base_type":{"type":"basic","base_type":"Q"},"size":10},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"trajectory_y","data_type":{"type":"vector","base_type":{"type":"basic","base_type":"Q"},"size":10},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"steering","data_type":{"type":"basic","base_type":"Q"},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"gas","data_type":{"type":"basic","base_type":"Q"},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"braking","data_type":{"type":"basic","base_type":"Q"},"direction":"INPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"set_steering","data_type":{"type":"basic","base_type":"Q"},"direction":"OUTPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"set_gas","data_type":{"type":"basic","base_type":"Q"},"direction":"OUTPUT","allows_multiple_inputs":false,"optional":false},
        {"name":"set_braking","data_type":{"type":"basic","base_type":"Q"},"direction":"OUTPUT","allows_multiple_inputs":false,"optional":false},
        {
            "name":"net_sock1",
            "port_type": "SOCKET",
            "data_type": {
                "type": "simple_packet",
                "payloadType": { "type": "basic", "base_type": "Z" }
            },
            "direction": "IO",
            "tags": ["network"]
        },
        {
            "name":"net_sock2",
            "port_type": "SOCKET",
            "data_type": {
                "type": "simple_packet",
                "payloadType": { "type": "basic", "base_type": "Q" }
            },
            "direction": "IO",
            "tags": ["network"]
        }
    ]
}
)";