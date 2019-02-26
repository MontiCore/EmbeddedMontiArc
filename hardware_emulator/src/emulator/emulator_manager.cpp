#include "emulator_manager.h"
#include "config.h"
#include <thread>

EmulatorManager EmulatorManager::instance;

void worker( HardwareEmulator *emu ) {
    emu->exec();
}

bool EmulatorManager::init() {
    available_threads = std::thread::hardware_concurrency();
    available_autopilots = "";
    //std::cout << "Available threads: " << available_threads << std::endl;
    //std::cout << "Available autopilots: " << std::endl;
    path = "./";
    //std::experimental::filesystem::v1::__cxx11::directory_entry e;
    //std::experimental::filesystem::v1::__cxx11::directory_iterator it;

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
            auto res = emu->init( *this, config );
            if ( res ) {
                emulators[i] = std::unique_ptr<HardwareEmulator>( emu );
                emulator_count++;
                return i;
            }
            else {
                error_msg = emu->error_msg;
                delete emu;
                return -1;
            }
        }
    }
    return -1;
}

void EmulatorManager::free_emulator( int id ) {
    emulator_count--;
    emulators[id].reset();
}

void EmulatorManager::start_tick( long long time_delta ) {
    for ( auto &pt : emulators ) {
        if ( pt )
            pt->add_time( time_delta );
    }
    for ( auto &pt : emulators ) {
        if ( pt )
            pt->thread = std::thread( worker, pt.get() );
    }
}

void EmulatorManager::end_tick() {
    for ( auto &pt : emulators ) {
        if ( pt )
            pt->thread.join();
    }
    
    //Write outputs if any
}

std::string EmulatorManager::querry( const char *msg ) {
    MessageParser parser( msg );
    MessageBuilder builder;
    while ( parser.has_next() ) {
        if ( parser.is_cmd( "get_error_msg" ) )
            builder.add( "error_msg", error_msg );
        if ( parser.is_cmd( "get_available_autopilots" ) )
            builder.add( "available_autopilots", available_autopilots );
        if ( parser.is_cmd( "get_available_threads" ) )
            builder.add( "available_threads", std::to_string( available_threads ) );
    }
    return builder.res;
}
