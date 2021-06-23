/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "software_simulator.h"

void SoftwareSimulator::init(const json& config, const fs::path& software_folder)
{
    std::string software_name = config["software_name"].get<std::string>();

    if (software_name.size() == 0)
        throw_error(Error::hardware_emu_software_load_error("Missing the name of the Software program to load."));


    auto path = fs::path(software_name);
    software_path = path.is_absolute() ? path : software_folder / path;
    program_name = software_path.stem().string();

    init_simulator(config, software_folder);

    auto interface_json = json::parse(program_interface->get_interface());
    std::string expected_version = "2.0";
    if (!interface_json.contains("version")) throw_error("could not get 'version' entry in program interface.");
    auto& v = interface_json["version"];
    if (!v.is_string()) throw_error("Expected string for 'version' entry in program interface.");
    auto version = v.get<std::string>();
    if (version != expected_version) throw_error("Expected version '" + expected_version + "' for program interface but got version '" +  version+ "'.");

    if (!interface_json.contains("ports")) throw_error("could not get 'ports' entry in program interface.");
    auto& ports = interface_json["ports"];
    if (!ports.is_array()) throw_error("Expected array for 'ports' entry in program interface.");

    int pid = 0;
    for (auto& e : ports) {
        if (!e.is_object()) throw_error("Expected object type for entries in the 'ports' array of the program interface.");
        if (!e.contains("name")) throw_error("Expected 'name' entry in 'ports' entry of the program interface");
        auto& n = e["name"];
        if (!n.is_string()) throw_error("Expected string type for 'name' entry in 'ports' entry of the program interface");
        port_id[n.get<std::string>()] = pid;
        ++pid;
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
