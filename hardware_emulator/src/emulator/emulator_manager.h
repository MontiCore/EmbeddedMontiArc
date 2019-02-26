#pragma once
#include "hardware_emulator.h"
#include <list>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem::v1;

struct EmulatorManager {    
    static EmulatorManager instance;
    Array<std::unique_ptr<HardwareEmulator>> emulators;
    uint emulator_count;
    
    std::list<fs::directory_entry> entries;
    std::string available_autopilots;
    uint available_threads;
    std::string path;
    
    std::string error_msg;
    bool init();
    
    int alloc_emulator( const char *config );
    void free_emulator( int id );
    
    void start_tick( long long time_delta );
    void end_tick();
    
    std::string querry( const char *msg );
};



