/**
 * (c) https://github.com/MontiCore/monticore
 */
#include <string>
#include <iostream>
#include <thread>
#include <vector>
#include <unordered_map>
#include <atomic>
#include <random>
#include <mutex>
#include "network.h"
#include "tcp_protocol.h"
#include "simulator/hardware_emulator.h"
#include "simulator/direct_software_simulator.h"

#if defined _WIN32 || defined _WIN64
#define IS_WIN
#else
#endif

/*
    Warning Ugly Code

    See https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/wikis/dev-docs/hardware_emulator/Hardware-Emulator-Server
    for the idea.
*/


class ServerException : public std::exception {
    std::string description;
public:
    ServerException(const std::string& description) : description("(Hardware-Emulator) Server Exception:\n\t" + description) {}
    virtual const char* what() const throw()
    {
        return description.c_str();
    }
};



void usage(char* app_name);
void signal_handler(int signal);
void session_callback(int socket);

struct EmulatorSession {
    DynamicBuffer buffer;
    int socket = 0;
    std::atomic<bool> running;
    bool json_data_exchange = false;
    std::unique_ptr<SoftwareSimulator> simulator;
    std::mt19937 gen;

    EmulatorSession(int seed) : gen(seed) {
        running.store(false);
    }

    void init(int socket) {
        running.store(true);
        this->socket = socket;
    }

    // Run the session
    void session();

    void init(PacketReader&packet_in);
    void reconnect(PacketReader &packet_in);

    void suspend();

    void set_input_json(PacketReader &packet_in);
    void set_input_binary(PacketReader &packet_in);
    void run_cycle(PacketReader &packet_in);
};


// Options
int MAX_THREADS = 8;

// -------


DynamicBuffer server_buffer;
using SessionPtr = std::unique_ptr<EmulatorSession>;
std::vector<SessionPtr> session_pool;

std::unordered_map<uint32_t, std::unique_ptr<SoftwareSimulator>> saved_simulators;
std::mutex saved_simulators_mutex;

bool server_running = true;


/*
    Entry point: Check arguments, port and start "Listening" (session_server())
*/
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

    if (argc < 2) {
        usage(argv[0]);
        return 0;
    }
    ERR_OUT_set_functions(SoftwareSimulator::ERR_OUT_throw_error, SoftwareSimulator::ERR_OUT_print_cout, SoftwareSimulator::ERR_OUT_print_cerr);

    for (int i = 2; i < argc; ++i) {
        std::string param = argv[i];
        if (param == "--max-threads" || param == "-t") {
            ++i;
            if (i >= argc) {
                std::cerr << "Missing argument for param " << param << std::endl;
                return -1;
            }
            MAX_THREADS = std::stol(argv[i]);
        }
        else {
            std::cerr << "Unknown param: " << param << std::endl;
            return -1;
        }
    }

    session_pool.resize(MAX_THREADS);

    auto port = argv[1];
    try {
        session_server(port, session_callback, server_running);
    }
    catch (std::exception & e) {
        std::cout << e.what() << std::endl;
    }

    return 0;
}

std::random_device main_dev{};
std::mt19937 main_gen{ main_dev() };

void session_callback(int socket) {
    auto seed = main_gen();
    for (auto& s : session_pool) {
        if (!s) s = std::make_unique<EmulatorSession>(seed);
        if (!s->running.load()) {
            // Found a free slot, start the thread
            s->init(socket);
            std::thread(&EmulatorSession::session, s.get()).detach();
            return;
        }
    }
    // Did not find a free slot
    try {
        PacketWriter packet(server_buffer, PACKET_ERROR);
        packet.bw.write_str("MAX allowed sessions (threads) reached: " + std::to_string(MAX_THREADS) + ", ignoring new connection");
        packet.send(socket);
    }
    catch (const std::exception & e2) {
        std::cerr << "Could not send error to simulator:\n\t" << e2.what() << std::endl;
    }
    close_sock(socket);
    return;
}

void signal_handler(int signal) {
    server_running = false;
}

void usage(char* app_name) {
    std::cout << "Usage:" << std::endl;
    std::cout << "  " << app_name << " <port> [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --max-threads or -t <number>" << std::endl;
}









/*
    A session thread:
    - Wait for INIT or RECONNECT
    - Setup/Get old emulator
    - Wait for data (INPUT) and RUN_CYCLE packets
*/
void EmulatorSession::session() {
    PacketReader packet_in(buffer, socket);
    try {
        packet_in.receive();
        if (packet_in.id == PACKET_INIT) {
            init(packet_in);
        }
        else if (packet_in.id == PACKET_RECONNECT) {
            reconnect(packet_in);
        }
        else {
            throw ServerException("Expected INIT packet, but got: id=" + std::to_string(packet_in.id) + " length=" + std::to_string(packet_in.size));
        }
        bool loop = true;
        while (loop) {
            packet_in.receive();
            if (packet_in.id < 0) break;
            switch (packet_in.id) {
            case PACKET_END:
                std::cout << "Ending connection." << std::endl;
                loop = false;
                break;
            case PACKET_INIT:
                throw ServerException("Received second INIT packet.");
                break;
            case PACKET_INPUT_JSON:
                set_input_json(packet_in);
                break;
            case PACKET_INPUT_BINARY:
                set_input_binary(packet_in);
                break;
            case PACKET_RUN_CYCLE:
                run_cycle(packet_in);
                break;
            case PACKET_SUSPEND:
                suspend();
                loop = false;
                break;
            default:
                throw ServerException("Unknown packet: id=" + std::to_string(packet_in.id) + " length=" + std::to_string(packet_in.size));
            }
            if (!server_running) {
                break;
            }
        }
    }
    catch (const std::exception & e) {
        auto msg = e.what();
        std::cerr << "Error: " << msg << std::endl;

        // Try sending the error to the simulator
        try {
            PacketWriter packet(buffer, PACKET_ERROR);
            packet.bw.write_str(msg);
            packet.send(socket);
        }
        catch (const std::exception & e2) {
            std::cerr << "Could not send error to simulator:\n\t" << e2.what() << std::endl;
        }
    }

    // Clean up
    simulator.reset();
    close_sock(socket);
    socket = 0;
    running.store(false);
}

void EmulatorSession::init(PacketReader&packet_in)
{
    // Send PACKET_REQUEST_CONFIG
    PacketWriter packet(buffer, PACKET_REQUEST_CONFIG);
    packet.send(socket);

    // Get CONFIG, ignore PACKET_REF_ID
    json config;
    bool repeat;
    do {
        repeat = false;
        packet_in.receive();
        switch (packet_in.id) {
        case PACKET_CONFIG:
            config = json::parse(packet_in.getReader().read_str());
            break;
        case PACKET_REF_ID:
            repeat = true;
            break;
        default:
            throw ServerException("Expected CONFIG packet, but got: id=" + std::to_string(packet_in.id) + " length=" + std::to_string(packet_in.size));
        }
    } while (repeat);

    auto softwares_folder = fs::current_path();
    simulator = std::unique_ptr<SoftwareSimulator>(SoftwareSimulator::alloc(config, softwares_folder));

    // Send interface
    PacketWriter packet_interface(buffer, PACKET_INTERFACE);
    packet_interface.bw.write_str(simulator->program_functions->get_interface());
    packet_interface.send(socket);
}

/*
    - Retake ownership of the emulator at received ID
    - Re-send dynamic-interface
*/
void EmulatorSession::reconnect(PacketReader &packet_in)
{
    auto token = packet_in.getReader().read_u32();
    {
        std::lock_guard<std::mutex> guard(saved_simulators_mutex);
        auto res = saved_simulators.find(token);
        if (res == saved_simulators.end()) throw ServerException("Could not restore emulator with token: "+std::to_string(token));

        simulator = std::unique_ptr<SoftwareSimulator>(res->second.release());
        saved_simulators.erase(res);
    }

    // Send interface
    PacketWriter packet_interface(buffer, PACKET_INTERFACE);
    packet_interface.bw.write_str(simulator->program_functions->get_interface());
    packet_interface.send(socket);
}

void EmulatorSession::suspend()
{
    auto token = 0;
    {
        std::lock_guard<std::mutex> guard(saved_simulators_mutex);
        do {
            token = gen();
        } while (token == 0 || saved_simulators.find(token) != saved_simulators.end());


        saved_simulators.emplace(token, simulator.release());
    }

    // Send token
    PacketWriter packet_interface(buffer, PACKET_EMU_ID);
    packet_interface.bw.write_u32(token);
    packet_interface.send(socket);
}

void EmulatorSession::set_input_json(PacketReader &packet_in)
{
    json_data_exchange = true;
    simulator->program_functions->set_port(packet_in.getReader().read_u16(), packet_in.getPayload() + 2, 1);
}

void EmulatorSession::set_input_binary(PacketReader &packet_in)
{
    json_data_exchange = false;
    auto port_id = packet_in.getReader().read_u16();
    // Buffer contains: entire packet = id + size + port_id + binary payload
    //                                  1    2      2         size-2
    // set_port() expects a binary buffer: size + binary payload
    //                                     4      size
    // Thus highjack the existing buffer to keep the binary payload in place
    auto new_buffer = packet_in.buffer.get_buffer() + 1;
    packet_in.buffer.go_to(1);
    BinaryWriter bw{ packet_in.buffer };
    bw.write_u32(packet_in.size-5);
    simulator->program_functions->set_port(port_id, new_buffer, 0);
}

void EmulatorSession::run_cycle(PacketReader &packet_in)
{
    auto delta_secs = packet_in.getReader().read_f64();

    simulator->start_timer();
    simulator->program_functions->execute(delta_secs);
    auto duration = simulator->get_timer_micro() * 0.000001;

    for (int i = 0; i < simulator->program_interface.ports.size(); ++i) {
        auto& p = simulator->program_interface.ports[i];
        if (p.is_output()) {
            do {
                PacketWriter packet(buffer, json_data_exchange ? PACKET_OUTPUT_JSON : PACKET_OUTPUT_BINARY); // Packet ID
                packet.bw.write_u16(i); // Port ID
                if (json_data_exchange) {
                    auto res = simulator->program_functions->get_port(i, 1);
                    if (res[0] == '\0') break;
                    packet.bw.write_str(res);
                }
                else {
                    auto res = simulator->program_functions->get_port(i, 0);
                    BinaryReader br = { res, 4 };
                    auto len = br.read_u32();
                    if (len == 0) break;
                    packet.bw.write_bytes(res+4, len);
                }
                packet.send(socket);
                if (p.port_type != PortType::SOCKET) break;
            } while (true);
        }
    }

    PacketWriter packet_interface(buffer, PACKET_TIME);
    packet_interface.bw.write_f64(duration);
    packet_interface.send(socket);

}
