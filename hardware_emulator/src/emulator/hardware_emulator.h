#pragma once
#include <thread>

#include "utility.h"
#include "computer/computer.h"

#include "function_value.h"
#include "config.h"

#include <unordered_map>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem::v1;

struct CacheSettings {
    struct Cache {
        bool used;
        //Pico second times
        ulong write_time;
        ulong read_time;
        uint block_size;
        uint size; //Size is number of block entries
        Cache() : used( false ) {}
        Cache( ulong read_time, ulong write_time, uint size, uint block_size = 8 ) :
            write_time( write_time ), read_time( read_time ), size( size ), block_size( block_size ), used( true ) {}
    };
    Cache IL1;
    Cache DL1;
    Cache L2;
    Cache L3;
    
    CacheSettings() : IL1( 10000, 10000, 128 ), DL1( 10000, 10000, 128 ), L2( 100000, 100000, 1024 ), L3() {}
    
    void setup_computer( Computer &computer );
    
};

struct EmulatorManager;
struct HardwareEmulator {
    std::thread thread;
    
    struct Port {
        FunctionValue buffer;
        uint64_t function_address;
        std::string name;
        bool updated;
        
        void *real_function;
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
    
    MeanAvgCollector avg_runtime;
    
    CacheSettings cache_settings;
    
    bool call_success;
    
    Library real_program;
    bool test_real;
    void *real_exec;
    void *real_init;
    
    std::string error_msg;
    
    bool computing() { //True when virtual computer still running in virtual time
        return computer.time.micro_time > 0;
    }
    
    std::string querry( const char *msg );
    
    bool init( EmulatorManager &manager, const char *config );
    bool resolve_autopilot_os( EmulatorManager &manager );
    
    void exec( ulong micro_delta );
    
    
    //-1 => not found
    int get_port_id( const char *port_name );
    
    
    void call_input( uint func_id );
    void call_output( uint func_id );
    void call_execute();
    void call_init();
    
    bool resolve( const std::string &name, uint64_t &target );
    bool resolve_real( const std::string &name, void *&target );
    
    bool init_ports( Array<Port> &ports, const char *get_count, const char *get_name, const char *get_type,
                     const char *port_prefix );
                     
    void setup_debug( MessageParser &parser );
};