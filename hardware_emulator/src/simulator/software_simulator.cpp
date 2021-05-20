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

}


json SoftwareSimulator::query(const json& query)
{
    //TODO
    return query_simulator(query);
}
