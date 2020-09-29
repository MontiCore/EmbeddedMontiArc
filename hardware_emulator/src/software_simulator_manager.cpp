/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "software_simulator_manager.h"

#include "hardware_emulator.h"
#include "direct_software_simulator.h"
#include "utility/config.h"
#include <thread>

SoftwareSimulatorManager SoftwareSimulatorManager::instance;


SoftwareSimulatorManager::SoftwareSimulatorManager()
{
    softwares_folder = FS::current_directory();
    available_threads = std::thread::hardware_concurrency();
    simulator_count = 0;
}

void SoftwareSimulatorManager::init(const json& config) {
    /*if (loaded){
        Log::info << Log::tag << "SoftwareSimulatorManager::init() on already loaded manager: skipping.\n";
        return;
    }*/
    available_softwares_string = "";
    
    //path = fs::current_path();
    softwares_folder = FS::current_directory();

    if (config.is_object()) {
        if (config.contains("softwares_folder")) {
            auto& entry = config["softwares_folder"];
            if (entry.is_string()) {
                softwares_folder = FS::Directory(entry.get<std::string>());
                if (!softwares_folder.is_absolute()) softwares_folder = (FS::current_directory() + softwares_folder).canonical();
            }
        }
    }

    Log::info << Log::tag << "softwares_folder: " << softwares_folder.to_string() << "\n";
    
    for ( const auto &file : softwares_folder.get_files()) {
        //std::cout << p << "\t" << p.filename() << "\t" << ext/*p.extension()*/ << std::endl;
        auto extension = file.get_extension();
        if (extension.compare( ".so" ) == 0 || extension.compare( ".dll" ) == 0 ) {
            available_softwares.emplace_back( file );
            if ( available_softwares_string.size() > 0 )
                available_softwares_string += ';';
            available_softwares_string += file.get_name();
        }
    }
}

int SoftwareSimulatorManager::alloc_simulator( const json& config) {
    std::unique_ptr<SoftwareSimulator> simulator;
    std::string simulator_mode = config["emulator_type"].get<std::string>();
    
    //Check simulator mode
    if (simulator_mode.compare("direct") == 0) {
        simulator = std::unique_ptr<SoftwareSimulator>(new DirectSoftwareSimulator());
    }
    else if (simulator_mode.compare("emu") == 0) {
        simulator = std::unique_ptr<SoftwareSimulator>(new HardwareEmulator());
    }
    else
        throw_error("SoftwareSimulatorManager: Config exception: unknown simulator mode: " + simulator_mode);

    //Initialize simulator
    simulator->init(config, softwares_folder);

    //Get id and store simulator
    if ( simulator_count >= simulators.size() )
        simulators.resize( simulators.size() + 5 );
    for ( auto i : irange( (int) simulators.size() ) ) {
        if ( !simulators[i] ) {
            simulators[i] = std::move(simulator);
            simulator_count++;
            return i;
        }
    }

    throw_error("Unknown Error"); //Should not reach this point
}

void SoftwareSimulatorManager::free_simulator( int id ) {
    simulator_count--;
    simulators[id].reset();
}


json SoftwareSimulatorManager::query(const json& query) {
    json output;

    for (auto& e : query.items()) {
        auto &key = e.key();
        if (key.compare("get_error_msg") == 0) output["error_msg"] = error_msg;
        else if (key.compare("get_available_autopilots") == 0) output["available_autopilots"] = available_softwares_string;
        else if (key.compare("get_available_threads") == 0) output["available_threads"] = available_threads;
        else if (key.compare("get_softwares_folder") == 0) output["softwares_folder"] = softwares_folder.to_string();
        else output["unknown_query"].push_back(key);
    }

    return output;
}

