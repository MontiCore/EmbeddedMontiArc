/**
 * (c) https://github.com/MontiCore/monticore
 */
#include <string>
#include <iostream>
#include <thread>
#include <vector>
#include <atomic>
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

// Options
int BUFFER_SIZE = 4096;
int MAX_THREADS = 32;

// -------

struct EmulatorSession {
    std::thread session_thread;
    std::vector<char> buffer;
    int socket;
    std::atomic<bool> done;
    bool json_data_exchange = false;

    EmulatorSession(int buff_size, int socket) : socket(socket) {
        buffer.assign(buff_size, '\0');
        done.store(false);
    }

    // Run the session
    void session();

    void init(PacketReader packet_in);
    void reconnect(PacketReader packet_in);

    void set_input_json(PacketReader packet_in);
    void set_input_binary(PacketReader packet_in);
    void run_cycle(PacketReader packet_in);
};

using SessionPtr = std::unique_ptr<EmulatorSession>;
std::vector<SessionPtr> sessions;
int session_count = 0;

std::vector<std::unique_ptr<SoftwareSimulator>> simulators;

bool running = true;


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

    for (int i = 2; i < argc; ++i) {
        std::string param = argv[i];
        if (param == "--max-threads" || param == "-t") {
            ++i;
            if (i >= argc) {
                std::cerr << "Missing argument for param " << param << std::endl;
                return -1;
            }
            MAX_THREADS = std::stol(argv[i]);
        } else if (param == "--buffer-size" || param == "-b") {
            ++i;
            if (i >= argc) {
                std::cerr << "Missing argument for param " << param << std::endl;
                return -1;
            }
            BUFFER_SIZE = std::stol(argv[i]);
        }
        else {
            std::cerr << "Unknown param: " << param << std::endl;
            return -1;
        }
    }


    auto port = argv[2];
    session_server(port, session_callback, running);

    return 0;
}

/*
    Gets called when a new connection is made to the server:
    - Check if thread limit reached + clean up old sessions if needed
    - Create the session thread
*/
void session_callback(int socket) {
    if (session_count >= MAX_THREADS) {
        // Try to clear done sessions
        for (auto& s : sessions) {
            if (s->done.load()) {
                s->session_thread.join();
                s.reset();
                --session_count;
            }
        }
    }
    if (session_count >= MAX_THREADS) {
        std::cerr << "MAX allowed sessions (threads) reached... ignoring new connection" << std::endl;
        close_sock(socket);
        return;
    }

    ++session_count;

    auto sess = new EmulatorSession(BUFFER_SIZE, socket);
    // Starts the new session
    sess->session_thread = std::thread(&EmulatorSession::session, sess);

    for (auto& s : sessions) {
        if (!s.operator bool()) {
            s = SessionPtr(sess);
            return;
        }
    }
    sessions.push_back(SessionPtr(sess));
}


void signal_handler(int signal) {
    running = false;
}

void usage(char* app_name) {
    std::cout << "Usage:" << std::endl;
    std::cout << "  " << app_name << " <port> [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --max-threads or -t <number>" << std::endl;
    std::cout << "  --buffer-size or -b <bytes> (default 4096)" << std::endl;
}









/*
    A session thread:
    - Wait for INIT or RECONNECT
    - Setup/Get old emulator
    - Wait for data (INPUT) and RUN_CYCLE packets
*/
void EmulatorSession::session() {
    try {
        while (true) {
            PacketReader packet_in1(buffer, socket);
            if (packet_in1.packet.id == PACKET_INIT) {
                init(packet_in1);
            }
            else if (packet_in1.packet.id == PACKET_RECONNECT) {
                reconnect(packet_in1);
            }
            else {
                throw ServerException("Expected INIT packet, but got: id=" + std::to_string(packet_in1.packet.id) + " length=" + std::to_string(packet_in1.packet.size));
            }
            PacketReader packet_in(buffer, socket);
            while (packet_in.packet.id >= 0) {
                switch (packet_in.packet.id) {
                case PACKET_END:
                    std::cout << "Ending connection." << std::endl;
                    return;
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
                default:
                    throw ServerException("Unknown packet: id=" + std::to_string(packet_in.packet.id) + " length=" + std::to_string(packet_in.packet.size));
                }
                if (!running) {
                    break;
                }
                packet_in = PacketReader(buffer, socket);
            }
        }
    }
    catch (const std::exception & e) {
        auto msg = e.what();
        std::cerr << "Error: " << msg << std::endl;

        // Try sending the error to the simulator
        try {
            PacketWriter packet(buffer, PACKET_ERROR);
            packet.write_str(msg);
            packet.send(socket);
        }
        catch (const std::exception & e2) {
            std::cerr << "Could not send error to simulator:\n\t" << e2.what() << std::endl;
        }
    }



    // Clean up
    close_sock(socket);
    socket = 0;
    done.store(true);
}

/*
    INIT:
    - Allocate new emulator slot
    - Send back ID
    - Get config -> load emulator (& autopilot)
    - Send back DynamicInterface
*/
void EmulatorSession::init(PacketReader packet_in)
{
    TODO handle remote autopilot;
    int id = TODO alloc slot;

    // Send back ID
    PacketWriter packet(buffer, PACKET_EMU_ID);
    packet.write_u8(id);
    packet.send(socket);

    // Get config
    PacketReader packet_in(buffer, socket);
    if (packet_in.packet.id != PACKET_CONFIG)
        throw ServerException("Expected CONFIG packet, but got: id=" + std::to_string(packet_in.packet.id) + " length=" + std::to_string(packet_in.packet.size));
        
    auto config = json::parse(packet_in.read_str());

    std::unique_ptr<SoftwareSimulator> simulator;
    if (!config.contains("backend")) throw_error("SoftwareSimulatorManager: Missing 'backend' entry in config.");
    auto backend = config["backend"];
    if (!backend.is_object()) throw_error("SoftwareSimulatorManager: 'backend' is not a JSON object.");
    if (!backend.contains("type")) throw_error("SoftwareSimulatorManager: 'backend' config has no 'type' entry.");
    std::string simulator_mode = backend["type"].get<std::string>();

    //Check simulator mode
    if (simulator_mode.compare("direct") == 0) {
        simulator = std::unique_ptr<SoftwareSimulator>(new DirectSoftwareSimulator());
    }
    else if (simulator_mode.compare("emu") == 0) {
        simulator = std::unique_ptr<SoftwareSimulator>(new HardwareEmulator());
    }
    else
        throw_error("SoftwareSimulatorManager: Config exception: unknown simulator mode: " + simulator_mode);

    //Initialize simulator
    simulator->init(config, softwares_folder);

    // Send interface
    PacketWriter packet2(buffer, PACKET_EMU_ID);
    packet2.write_str(simulator->program_interface->get_interface());
    packet2.send(socket);

    simulators[id] = std::move(simulator);
}

/*
    - Retake ownership of the emulator at received ID
    - Error if already removed
    - Re-send dynamic-interface
*/
void EmulatorSession::reconnect(PacketReader packet_in)
{
    TODO;
}

void EmulatorSession::set_input_json(PacketReader packet_in)
{
    json_data_exchange = true;
    TODO;
}

void EmulatorSession::set_input_binary(PacketReader packet_in)
{
    json_data_exchange = false;
    TODO;
}

void EmulatorSession::run_cycle(PacketReader packet_in)
{
    TODO;
}
