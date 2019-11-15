/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#pragma once
#include "software_simulator.h"
#include <list>
#include "utility/utility.h"
/*
    The EmulatorManager is used to allocate and interact with autopilot emualtors.
    The allocated autopilots are addressed through the id received when allocating.
*/
struct SoftwareSimulatorManager {
    static SoftwareSimulatorManager instance;
    bool loaded = false;
	std::vector<std::unique_ptr<SoftwareSimulator>> simulators;
    uint simulator_count;
    
    //std::list<fs::directory_entry> entries;
    std::list<FS::File> available_softwares;
    std::string available_softwares_string;
    uint available_threads = 0;
    //fs::path path;
    FS::Directory softwares_folder;
    
    
    std::string error_msg;
    /*
        config is the configuration of the MANAGER
        Supported configuration:
    
        autopilots_folder=folder where the emualtors will look for autopilots.
    
    
        The contents of default_config will be added to the start of the config used to allocate an emulator.
    */
    void init( const char *config );
    
    /*
        config is the configuration of the EMULATOR
        Returns the allocated emulator id or -1 on error. On error, error_msg contains the error trace.
    */
    int alloc_simulator( const char *config );
    void free_simulator( int id );
    
    
    /*
        Supported queries:
    
        query:      get_error_msg
        response:   error_msg=msg
    
        query:      get_autopilots_folder
        response:   autopilots_folder=current folder where the emulator manager looks for autopilots
    
        query:      get_available_autopilots
        response:   available_autopilots=list of autopilots available in the autopilot folder
    
        query:      get_available_threads
        response:   available_threads=number of threads of the machine running the manager.
    
    */
    std::string query( const char *msg );
};
