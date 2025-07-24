/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include <fstream>

#include "software_simulator.h"

#include "utility/utility.h"
#include "computer/computer.h"

#include <unordered_map>

struct SoftwareSimulatorManager;

struct EmulatedProgramFunctions : public ProgramFunctions {
    Computer &computer;

    uint64_t addr_get_functions = 0;
    uint64_t addr_get_interface = 0;
    uint64_t addr_set_port = 0;
    uint64_t addr_get_port = 0;
    uint64_t addr_init = 0;
    uint64_t addr_exec = 0;

    EmulatedProgramFunctions(Computer &computer) : computer(computer) {}

    uint64_t resolve( const char *name );
    void load();
    
    void set_functions(uint64_t throw_error_ptr, uint64_t print_cout_ptr, uint64_t print_cerr_ptr);
    const char* get_interface();
    void set_port(int i, const char* data, int is_json);
    const char* get_port(int i, int is_json);
    
    void init();
    void execute(double delta_sec);

};

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
        test_real                       If the emulated autopilot has the same os as the emulator, the emulator will load
                                        the autopilot directly (as dynamic library) and compare the outputs with the
                                        outputs from the emulation.

*/
struct HardwareEmulator : public SoftwareSimulator {
    
    Computer computer;
    std::string os_name;
    CacheSettings cache_settings;

    bool debug_time = false;

    ulong timer_start;
    PrecisionTimer timer;
    
    //True when virtual computer still running in virtual time
    bool computing() {
        return computer.time.micro_time > 0;
    }
    
    json query_simulator(const json& query);
    
    void init_simulator(const json& config, const fs::path& software_folder);
    void resolve_autopilot_os(const fs::path& software_folder );

    void start_timer();
    ulong get_timer_micro();

    void parse_flags( const json &flags );

};
