/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "software_simulator.h"
#include "hardware_emulator.h"
#include "direct_software_simulator.h"

SoftwareSimulator* SoftwareSimulator::alloc(const json& config, fs::path& softwares_folder)
{
    SoftwareSimulator *simulator = nullptr;
    if (!config.contains("backend")) throw_error("SoftwareSimulatorManager: Missing 'backend' entry in config.");
    auto backend = config["backend"];
    if (!backend.is_object()) throw_error("SoftwareSimulatorManager: 'backend' is not a JSON object.");
    if (!backend.contains("type")) throw_error("SoftwareSimulatorManager: 'backend' config has no 'type' entry.");
    std::string simulator_mode = backend["type"].get<std::string>();

    //Check simulator mode
    if (simulator_mode.compare("direct") == 0) {
        simulator = new DirectSoftwareSimulator();
    }
    else if (simulator_mode.compare("emu") == 0) {
        simulator = new HardwareEmulator();
    }
    else
        throw_error("SoftwareSimulatorManager: Config exception: unknown simulator mode: " + simulator_mode);

    //Initialize simulator
    simulator->init(config, softwares_folder);
    return simulator;
}

void SoftwareSimulator::init(const json& config, const fs::path& software_folder)
{
    std::string software_name = config["software_name"].get<std::string>();

    if (software_name.size() == 0)
        throw_error(Error::hardware_emu_software_load_error("Missing the name of the Software program to load."));


    auto path = fs::path(software_name);
    software_path = path.is_absolute() ? path : software_folder / path;
    program_name = software_path.stem().string();

    init_simulator(config, software_folder);

    interface_string = program_functions->get_interface();
    parse_interface(interface_string.c_str(), program_interface);

    std::string expected_version = "2.0";
    if (program_interface.version != expected_version) throw_error("Expected version '" + expected_version + "' for program interface but got version '" + program_interface.version + "'.");

    for (int i = 0; i < program_interface.ports.size(); ++i) {
        port_id[program_interface.ports[i].name] = i;
    }

}


json SoftwareSimulator::query(const json& query)
{
    //TODO
    return query_simulator(query);
}

int SoftwareSimulator::get_port_id(const std::string& name) const
{
    const auto& e = port_id.find(name);
    if (e != port_id.end()) return e->second;
    return -1;
}

int SoftwareSimulator::get_port_id_expected(const std::string& name) const
{
    const auto& e = port_id.find(name);
    if (e != port_id.end()) return e->second;
    throw_error("Could not get port_id for port '" + name + "'.");
}


void SoftwareSimulator::ERR_OUT_throw_error(const char* type, const char* message) {
    throw_error(message);
}
void SoftwareSimulator::ERR_OUT_print_cout(const char* message) {
    Log::info.log_tag("%s", message);
}
void SoftwareSimulator::ERR_OUT_print_cerr(const char* message) {
    Log::err.log_tag("%s", message);
}
