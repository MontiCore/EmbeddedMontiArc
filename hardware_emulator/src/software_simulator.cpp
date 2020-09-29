/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "software_simulator.h"

#include "utility/config.h"
#include "software_simulator_manager.h"

void SoftwareSimulator::init(const json& config, const FS::Directory& software_folder)
{
    std::string software_name = config["software_name"].get<std::string>();

    if (software_name.size() == 0)
        throw_error(Error::hardware_emu_software_load_error("Missing the name of the Software program to load."));


    auto path = FS::File(software_name);
    software_path = path.folder.is_absolute() ? path : software_folder+ path;
    program_name = software_path.get_name();

    init_simulator(config, software_folder);

}


json SoftwareSimulator::query(const json& query)
{
    //TODO
    return query_simulator(query);
}
