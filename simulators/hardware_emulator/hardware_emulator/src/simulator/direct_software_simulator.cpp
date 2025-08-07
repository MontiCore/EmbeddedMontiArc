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

void DirectProgramFunctions::load(Library &software){
    real_set_functions = (set_functions_func)software.get_function(FUNC_NAME_ERR_OUT_SET_FUNCTIONS, true);
    real_init = (DirectProgramFunctions::InitFunc)software.get_function( FUNC_NAME_INIT );
    real_exec = (DirectProgramFunctions::ExecFunc)software.get_function( FUNC_NAME_EXECUTE );
    real_get_interface = (DirectProgramFunctions::GetInterfaceFunc)software.get_function( FUNC_NAME_GET_INTERFACE );
    real_set_port = (DirectProgramFunctions::SetPortFunc)software.get_function( FUNC_NAME_SET_PORT );
    real_get_port = (DirectProgramFunctions::GetPortFunc)software.get_function( FUNC_NAME_GET_PORT );
}

void DirectSoftwareSimulator::init_simulator(const json& config, const fs::path& software_folder)
{
    software.init(software_path);

    DirectProgramFunctions* prog_func = new DirectProgramFunctions();
    prog_func->load(software);
    if (prog_func->real_set_functions != nullptr) prog_func->real_set_functions(ERR_OUT_native_throw_error, ERR_OUT_native_print_cout, ERR_OUT_native_print_cerr);
    prog_func->init();

    program_functions = std::unique_ptr<ProgramFunctions>(prog_func);

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
