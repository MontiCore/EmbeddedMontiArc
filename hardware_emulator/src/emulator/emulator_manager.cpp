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
    available_autopilots = "";
    
    path = fs::canonical( "./" );
    MessageParser parser( config );
    while ( parser.has_next() ) {
        if ( parser.is_cmd( "autopilots_folder" ) )
            path = fs::canonical( parser.get_string() );
        else
            parser.unknown();
    }
    
    Log::info << Log::tag << "autopilots_folder path: " << path << "\n";
    Log::info << Log::tag << "Default config:\n" << default_config;
    
    for ( const auto &entry : fs::directory_iterator( path ) ) {
        if ( entry.status().type() == fs::file_type::regular ) {
            auto &p = entry.path();
            std::string ext = p.extension().generic_string();
            //std::cout << p << "\t" << p.filename() << "\t" << ext/*p.extension()*/ << std::endl;
            if ( ext.compare( ".so" ) == 0 || ext.compare( ".dll" ) == 0 ) {
                entries.emplace_back( p );
                if ( available_autopilots.size() > 0 )
                    available_autopilots += ';';
                available_autopilots += p.filename().generic_string();
            }
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
            builder.add( "available_autopilots", available_autopilots );
        else if ( parser.is_cmd( "get_available_threads" ) )
            builder.add( "available_threads", std::to_string( available_threads ) );
        else if ( parser.is_cmd( "get_autopilots_folder" ) )
            builder.add( "autopilots_folder", path.string() );
        else
            parser.unknown();
    }
    return builder.res;
}
