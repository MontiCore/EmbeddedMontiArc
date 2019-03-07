#pragma once
#include <thread>

#include "utility.h"
#include "computer/computer.h"

#include "function_value.h"
#include "config.h"

#include <unordered_map>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem::v1;

struct EmulatorManager;
struct HardwareEmulator {
    std::thread thread;
    
    struct Port {
        FunctionValue buffer;
        uint64_t function_address;
        std::string name;
        bool updated;
    };
    using PortMap = std::unordered_map<std::string, int>;
    PortMap port_id;
    Array<Port> input_ports;
    Array<Port> output_ports;
    
    uint64_t init_address;
    uint64_t execute_address;
    
    Computer computer;
    std::string os_name;
    std::string autopilot_name;
    fs::path path;
    
    MemoryRange buffer_slot;
    
    ulong simulation_time;
    
    bool call_success;
    
    std::string error_msg;
    
    bool computing() { //True when virtual computer still running in virtual time
        return simulation_time < computer.computing_time;
    }
    
    std::string querry( const char *msg );
    
    bool init( EmulatorManager &manager, const char *config );
    bool resolve_autopilot_os( EmulatorManager &manager );
    
    void exec();
    
    void add_time( ulong delta ); //Check if computer stopped running => update outputs
    
    //-1 => not found
    int get_port_id( const char *port_name );
    
    
    void call_input( uint func_id );
    void call_output( uint func_id );
    void call_void( uint64_t address, const char *name );
    
    bool resolve( const std::string &name, uint64_t &target );
    
    bool init_ports( Array<Port> &ports, const char *get_count, const char *get_name, const char *get_type,
                     const char *port_prefix );
                     
    void setup_debug( MessageParser &parser );
};