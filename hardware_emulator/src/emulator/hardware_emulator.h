/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#pragma once
#include <thread>
#include <fstream>
#include <chrono>

#include "utility.h"
#include "computer/computer.h"

#include "function_value.h"
#include "config.h"

#include <unordered_map>
//#include <filesystem>

struct PrecisionTimer {
        using HRClock = std::chrono::high_resolution_clock;
        using TimePoint = HRClock::time_point;
        static inline long time_delta( TimePoint start, TimePoint end ) {
            return std::chrono::duration_cast<std::chrono::microseconds>( end - start ).count();
        }
        static std::string microsecondTimeToString( ulong time ) {
            if ( time >= 1000000 )
                return std::to_string( time / 1000000 ) + "." + std::to_string( ( time / 1000 ) % 1000 ) + "s";
            else if ( time >= 1000 )
                return std::to_string( time / 1000 ) + "." + std::to_string( time % 1000 ) + "ms";
            else
                return std::to_string( time ) + "us";
        }
    protected:
        TimePoint m_start, m_end;
    public:
        std::string name;
        inline void start() {
            m_start = HRClock::now();
        }
        inline void end() {
            m_end = HRClock::now();
        }
        inline long getDelta() {
            return ( long )time_delta( m_start, m_end );
        }
        std::string print( std::string tab = std::string() ) {
            return microsecondTimeToString( getDelta() );
        }
};

//namespace fs = std::filesystem;
/*
    CacheSettings is used to configure the cache layout of the computer.
    Replace the IL1, DL1, L2 and L3 members with Cache objects to enable and configure the given cache level.

    if the next command of the MessageParser used to read the configuration of the autopilot starts with "cache_",
    handle_config() will read the config entry and set the according cache settings.

    setup_computer() will then apply the configuration to the computer.
*/
struct CacheSettings {
    struct Cache {
        bool used;
        //Pico second times
        uint write_ticks;
        uint read_ticks;
        uint block_size;
        uint size; //Size is number of block entries
        Cache() : used( false ) {}
        Cache( uint read_ticks, uint write_ticks, uint size, uint block_size = 8 ) :
            write_ticks( write_ticks ), read_ticks( read_ticks ), size( size ), block_size( block_size ), used( true ) {}
    };
    Cache IL1;
    Cache DL1;
    Cache L2;
    Cache L3;
    
    void handle_config( MessageParser &parser );
    
    void setup_computer( Computer &computer );
    
};

struct EmulatorManager;

/*
    The HardwareEmulator is the component handling the emulation of autopilots and their communication with the
    simulations server.

    It is initialized with init() and a configuration message.
    From the message, it will try to load an autopilot file, setup the computer and resolve the ports of the autopilot model.
    The current configuration commands supported are:

        autopilot=name                  Autopilot name without file extension   //Defaults to "AutopilotAdapter"
        os=name                         Currently 'linux' or 'windows'
        debug=flag1,flag2,flag3,...     The available flags are: 'code', 'mem', 'reg_update', 'syscalls', 'unsupported_syscalls',
                                        'call' or 'time'.                       //Defaults to no debug output
        cpu_frequency=frequency         Sets the CPU frequency in Hertz         //Defaults to 1MHz
        memory_frequency=frequency      Sets the Memory frequency in Hertz      //Defaults to 100KHz
        cache_NAME=size,read,write      Enables the NAME cache layer and sets its size (number of cached blocks), read and write times
                                        expressed in CPU cycles (ticks). NAME can be 'DL1' (data level 1), 'IL1' (instruction level 1),
                                        'L2' or 'L3'.                           //Defaults to no cache
        no_time                         Disables the time delaying of the autopilot
        test_real                       If the emulated autopilot has the same os as the emulator, the emulator will load
                                        the autopilot directly (as dynamic library) and compare the outputs with the
                                        outputs from the emulation.
        export                          Enables the output of simulation data in the export_tick() function (EXPERIMENTAL)

    The input_ports and output_ports array then contain descriptors for the different ports as well as a buffer for the port type.
    The port_map maps the name of an autopilot port to its Port object.

    The call_input(Port) function calls the setter of the port and passes the content of the Port buffer to the autopilot.
    The call_output(Port) function calls the getter of the port and fill the Port buffer with the results.
    call_init() and call_execute() call the init() and execute() functions of the autopilot.

    export_tick() is a data logging function that will write the deviation from the trajectory and the execution time of the
    autopilot code for the given tick. The output files are autopilot_time.txt and autopilot_dist.txt. This output is enabled
    with the 'export' configuration command.

    The exec() function performs an update tick of the simulation. It is responsible for setting the autopilot ports,
    executing it, reading the ports and applying the time delaying.
*/
struct HardwareEmulator {
    std::thread thread;
    
    struct Port {
        FunctionValue buffer;
        uint64_t function_address;
        std::string name;
        bool updated;
        
        void *real_function;
    };
    using PortMap = std::unordered_map<std::string, Port *>;
    PortMap port_map;
	std::vector<Port> input_ports;
	std::vector<Port> output_ports;
    
    uint64_t init_address;
    uint64_t execute_address;
    
    Computer computer;
    std::string os_name;
    std::string autopilot_name;
    //fs::path path;
    std::string autopilot_path;
    
    MemoryRange buffer_slot;
    
    MeanAvgCollector avg_runtime;
    
    CacheSettings cache_settings;
    
    PrecisionTimer timer;
    long real_time_accumulator = 0;
    
    bool call_success;
    //uint debug_tick_count;
    bool debug_time;
    bool export_data;
    ulong execution_time;
    ulong simulation_time;
    std::ofstream dist_output;
    std::ofstream time_output;
    
    Library real_program;
    bool test_real;
	bool no_emulation;
    void *real_exec;
    void *real_init;
    
    std::string error_msg;
    
    //True when virtual computer still running in virtual time
    bool computing() {
        return computer.time.micro_time > 0;
    }
    
    std::string query( const char *msg );
    
    bool init( EmulatorManager &manager, const char *config );
    bool resolve_autopilot_os( EmulatorManager &manager );
    
    void exec( ulong micro_delta );
    
    
    //nullptr if not found
    Port *get_port( const char *port_name );
    
    
    void call_input( Port &port );
    void call_output( Port &port );
    void call_execute();
    void call_init();
    
    bool resolve( const std::string &name, uint64_t &target );
    bool resolve_real( const std::string &name, void *&target );
    
    bool init_ports(std::vector<Port> &ports, const char *get_count, const char *get_name, const char *get_type,
                     const char *port_prefix );
                     
    void setup_debug( MessageParser &parser );
	bool setup_direct_emulation();
	bool init_ports_direct(std::vector<Port>& ports, const char* get_count, const char* get_name, const char* get_type,
		const char* port_prefix);
	void call_input_direct(Port& port);
	void call_output_direct(Port& port);
    
    void export_tick();
};
