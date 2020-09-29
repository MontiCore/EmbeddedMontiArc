#include "direct_software_simulator.h"

json DirectSoftwareSimulator::query_simulator(const json& query)
{
    return std::string();
}

void DirectProgramInterface::load(Library &software){
    real_init = (DirectProgramInterface::InitFunc)software.get_function( FUNC_NAME_INIT );
    real_exec = (DirectProgramInterface::ExecFunc)software.get_function( FUNC_NAME_EXECUTE );
    real_get_interface = (DirectProgramInterface::GetInterfaceFunc)software.get_function( FUNC_NAME_GET_INTERFACE );
    real_set_port = (DirectProgramInterface::SetPortFunc)software.get_function( FUNC_NAME_SET_PORT );
    real_get_port = (DirectProgramInterface::GetPortFunc)software.get_function( FUNC_NAME_GET_PORT );
}

void DirectSoftwareSimulator::init_simulator(const json& config, const FS::Directory& software_folder)
{
    software.init(software_path);

    DirectProgramInterface* prog_interface = new DirectProgramInterface();
    prog_interface->load(software);
    prog_interface->init();

    program_interface = std::unique_ptr<ProgramInterface>(prog_interface);

    Log::info << Log::tag << "Initiated software in direct mode: " << program_name << "\n";
}

void DirectSoftwareSimulator::start_timer()
{
    timer.start();
}

ulong DirectSoftwareSimulator::get_timer_micro()
{
    timer.end();
    return timer.getDelta();
}
