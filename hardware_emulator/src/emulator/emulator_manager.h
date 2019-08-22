/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
#pragma once
#include "hardware_emulator.h"
#include <list>
#include "utility.h"
//#include <filesystem>

//namespace fs = std::filesystem;

/*
    The EmulatorManager is used to allocate and interact with autopilot emualtors.
    The allocated autopilots are addressed through the id received when allocating.
*/
struct EmulatorManager {
    static EmulatorManager instance;
    Array<std::unique_ptr<HardwareEmulator>> emulators;
    uint emulator_count;
    
    //std::list<fs::directory_entry> entries;
    std::list<FS::File> available_autopilots;
    std::string available_autopilots_string;
    uint available_threads;
    //fs::path path;
    std::string autopilots_folder;
    
    std::string default_config;
    
    std::string error_msg;
    /*
        config is the configuration of the MANAGER
        Supported configuration:
    
        autopilots_folder=folder where the emualtors will look for autopilots.
    
    
        The contents of default_config will be added to the start of the config used to allocate an emulator.
    */
    bool init( const char *config, const char *default_config );
    
    /*
        config is the configuration of the EMULATOR
        Returns the allocated emulator id or -1 on error. On error, error_msg contains the error trace.
    */
    int alloc_emulator( const char *config );
    void free_emulator( int id );
    
    //Start the emulation on all the local emulators. This function is non-blocking
    void start_tick( long long time_delta );
    //Collects all the emulators and returns when all the emulators are finished.
    void end_tick();
    
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



