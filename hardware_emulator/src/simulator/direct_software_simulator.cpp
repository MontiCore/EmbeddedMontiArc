/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "direct_software_simulator.h"


class AutopilotException : public std::exception {
    std::string msg;
public:
    AutopilotException(const char* type, const char* message) : msg(((std::string("[AP] Throw called in autopilot: [") + type) + "] ") + message) {}
    virtual const char* what() const throw()
    {
        return msg.c_str();
    }
};


void ERR_OUT_native_throw_error(const char* type, const char* message) {
    throw AutopilotException(type, message);
}
void ERR_OUT_native_print_cout(const char* message) {
    Log::ap.log_tag("[cout] %s", message);
}
void ERR_OUT_native_print_cerr(const char* message) {
    Log::ap.log_tag("[cerr] %s", message);
}


json DirectSoftwareSimulator::query_simulator(const json& query)
{
    return std::string();
}

void DirectProgramInterface::load(Library &software){
    real_set_functions = (set_functions_func)software.get_function(FUNC_NAME_ERR_OUT_SET_FUNCTIONS, true);
    real_init = (DirectProgramInterface::InitFunc)software.get_function( FUNC_NAME_INIT );
    real_exec = (DirectProgramInterface::ExecFunc)software.get_function( FUNC_NAME_EXECUTE );
    real_get_interface = (DirectProgramInterface::GetInterfaceFunc)software.get_function( FUNC_NAME_GET_INTERFACE );
    real_set_port = (DirectProgramInterface::SetPortFunc)software.get_function( FUNC_NAME_SET_PORT );
    real_get_port = (DirectProgramInterface::GetPortFunc)software.get_function( FUNC_NAME_GET_PORT );
}

void DirectSoftwareSimulator::init_simulator(const json& config, const fs::path& software_folder)
{
    software.init(software_path);

    DirectProgramInterface* prog_interface = new DirectProgramInterface();
    prog_interface->load(software);
    if (prog_interface->real_set_functions != nullptr) prog_interface->real_set_functions(ERR_OUT_native_throw_error, ERR_OUT_native_print_cout, ERR_OUT_native_print_cerr);
    prog_interface->init();

    program_interface = std::unique_ptr<ProgramInterface>(prog_interface);

    Log::info.log_tag("Initiated software in direct mode: %s", program_name.c_str());
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
