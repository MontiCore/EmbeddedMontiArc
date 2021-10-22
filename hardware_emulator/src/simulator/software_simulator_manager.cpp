/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "software_simulator_manager.h"

#include "simulator/hardware_emulator.h"
#include "simulator/direct_software_simulator.h"
#include <thread>
#include <clocale>

SoftwareSimulatorManager SoftwareSimulatorManager::instance;


SoftwareSimulatorManager::SoftwareSimulatorManager()
{
    available_threads = std::thread::hardware_concurrency();
    simulator_count = 0;
    setlocale(LC_ALL, "C");
}

void SoftwareSimulatorManager::init(const json& config) {
    /*if (loaded){
        Log::info << Log::tag << "SoftwareSimulatorManager::init() on already loaded manager: skipping.\n";
        return;
    }*/
    available_softwares_string = "";
    
    //path = fs::current_path();
    softwares_folder = fs::current_path();

    if (config.is_object()) {
        if (config.contains("softwares_folder")) {
            auto& entry = config["softwares_folder"];
            if (entry.is_string()) {
                softwares_folder = fs::path(entry.get<std::string>());
                if (!softwares_folder.is_absolute()) softwares_folder = (fs::current_path() / softwares_folder).lexically_normal();
            }
        }
    }

    Log::info.log_tag("softwares_folder: %s", softwares_folder.string().c_str());

    for (const auto& subpath : fs::recursive_directory_iterator(softwares_folder)) {
        if (fs::is_directory(subpath)) continue;
        auto p = subpath.path();
        auto extension = p.extension();
        if (extension.compare(".so") == 0 || extension.compare(".dll") == 0) {
            available_softwares.emplace_back(p);
            if (available_softwares_string.size() > 0)
                available_softwares_string += ';';
            available_softwares_string += p.stem().string();
        }
    }
    
}

int SoftwareSimulatorManager::alloc_simulator( const json& config) {
    std::unique_ptr<SoftwareSimulator> simulator(SoftwareSimulator::alloc(config, softwares_folder));
    
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
        else if (key.compare("get_softwares_folder") == 0) output["softwares_folder"] = softwares_folder.string();
        else output["unknown_query"].push_back(key);
    }

    return output;
}

