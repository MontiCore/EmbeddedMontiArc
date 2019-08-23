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
#include "emulator_manager.h"
#include "config.h"
#include <thread>

EmulatorManager EmulatorManager::instance;

void worker( HardwareEmulator *emu, long long time_delta ) {
    emu->exec( time_delta );
}

bool EmulatorManager::init( const char *config, const char *default_config ) {
    this->default_config = default_config;
    available_threads = std::thread::hardware_concurrency();
    available_autopilots_string = "";
    
    //path = fs::current_path();
    autopilots_folder = FS::current_path();
    MessageParser parser( config );
    while ( parser.has_next() ) {
        if ( parser.is_cmd( "autopilots_folder" ) )
            //path = fs::canonical( parser.get_string() );
            autopilots_folder = FS::canonical(parser.get_string());
        else
            parser.unknown();
    }
    
    Log::info << Log::tag << "autopilots_folder: " << autopilots_folder << "\n";
    Log::info << Log::tag << "Default config:\n" << default_config;
    
    for ( const auto &file : FS::directory_files( autopilots_folder ) ) {
        //std::cout << p << "\t" << p.filename() << "\t" << ext/*p.extension()*/ << std::endl;
        if ( file.extension.compare( ".so" ) == 0 || file.extension.compare( ".dll" ) == 0 ) {
            available_autopilots.emplace_back( file );
            if ( available_autopilots_string.size() > 0 )
                available_autopilots_string += ';';
            available_autopilots_string += file.name;
        }
    }
    emulators.drop();
    emulator_count = 0;
    return true;
}

int EmulatorManager::alloc_emulator( const char *config ) {
    if ( emulator_count >= emulators.size() )
        emulators.resize( emulators.size() + 5 );
    for ( auto i : urange( emulators.size() ) ) {
        if ( !emulators[i] ) {
            auto *emu = new HardwareEmulator();
            auto conf = default_config + config;
            auto res = emu->init( *this, conf.c_str() );
            if ( res ) {
                emulators[i] = std::unique_ptr<HardwareEmulator>( emu );
                emulator_count++;
                Log::info << Log::tag << "Emulator allocated with id " << i << "\n";
                return i;
            }
            else {
                error_msg = emu->error_msg;
                delete emu;
                return -1;
            }
        }
    }
    error_msg = "Unknown Error";
    return -1;
}

void EmulatorManager::free_emulator( int id ) {
    Log::info << Log::tag << "Emulator " << id << " freed\n";
    emulator_count--;
    emulators[id].reset();
}

void EmulatorManager::start_tick( long long time_delta ) {
    for ( auto &pt : emulators ) {
        if ( pt )
            pt->thread = std::thread( worker, pt.get(), time_delta );
    }
}

void EmulatorManager::end_tick() {
    for ( auto &pt : emulators ) {
        if ( pt )
            pt->thread.join();
    }
}

std::string EmulatorManager::query( const char *msg ) {
    MessageParser parser( msg );
    MessageBuilder builder;
    while ( parser.has_next() ) {
        if ( parser.is_cmd( "get_error_msg" ) )
            builder.add( "error_msg", error_msg );
        else if ( parser.is_cmd( "get_available_autopilots" ) )
            builder.add( "available_autopilots", available_autopilots_string );
        else if ( parser.is_cmd( "get_available_threads" ) )
            builder.add( "available_threads", std::to_string( available_threads ) );
        else if ( parser.is_cmd( "get_autopilots_folder" ) )
            builder.add( "autopilots_folder", autopilots_folder );
        else
            parser.unknown();
    }
    return builder.res;
}
