/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#pragma once
#include <fstream>

#include "software_simulator.h"

#include "utility/utility.h"
#include "computer/computer.h"

#include "utility/config.h"

#include <unordered_map>

struct SoftwareSimulatorManager;

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

    uint64_t init_address = 0;
    uint64_t execute_address = 0;

    
    bool debug_time = false;
    ulong execution_time = 0;
    ulong simulation_time = 0;
    MeanAvgCollector avg_runtime;
    PrecisionTimer timer;
    
    std::string test_real_opt;
    bool test_real = false;
    std::unique_ptr<SoftwareSimulator> direct_simulator;
    
    //True when virtual computer still running in virtual time
    bool computing() {
        return computer.time.micro_time > 0;
    }
    
    std::string query_simulator( const char *msg );
    
    void init_simulator(SoftwareSimulatorManager&manager, const char *config );
    void resolve_autopilot_os(SoftwareSimulatorManager&manager );

    void run_cycle();
    ulong get_cycle_time();

    void exec();
    
    void call_init();
    
    void resolve( const std::string &name, uint64_t &target );
                     
    void setup_debug( MessageParser &parser );

    const char* get_string_by_id(const char* name, int id);
    int get_int(const char* name);

    Port* new_port_by_type(const PortInformation& info);
};
